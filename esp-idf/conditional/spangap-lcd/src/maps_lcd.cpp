/**
 * maps_lcd — on-device offline map viewer (the maps program's LCD slice).
 *
 * Draws pre-baked slippy-map tiles read off the SD card, with a GPS position
 * marker. Nothing is rendered or fetched on the device: tiles are baked on a
 * computer (the tilebake toolchain) and copied to the SD card as
 * /sdcard/maps/<z>/<x>/<y>.jpg (decoded on the worker via esp_jpeg) or .bin
 * (raw 256x256 little-endian RGB565 — LVGL's native canvas order, byte-swapped
 * for the ST7789 on flush). .jpg is tried first, .bin is the fallback.
 *
 * Features:
 *   - Follows the GPS fix, or free-pan (drag / trackball-drag) with a
 *     "centre me" button to re-lock to GPS.
 *   - Overzoom fallback: a missing tile at the display zoom is filled by
 *     upscaling the nearest existing lower-zoom ancestor, so a coarse base map
 *     "shows through" where local detail is absent instead of going grey.
 *
 * Split of work:
 *   - worker task (this file): owns the tile cache, the slippy-map math, the
 *     view centre / follow state. Subscribes to gps.* and s.maps.*; the lcd
 *     task feeds it pan deltas + a recentre flag. On any change it fetches the
 *     visible tiles (or ancestors) from SD into a PSRAM LRU cache, then hands a
 *     composite request to the lcd task.
 *   - lcd task: owns the LVGL canvas + the centre-me button + the drag handler.
 *     composite() blits cached tiles (scaled for overzoom) into the canvas,
 *     draws the marker, toggles a status label.
 * The cache is shared (s_mux); pan/recentre flags are shared (s_ctrlMux).
 *
 *   - Waypoints: pan so the crosshair sits on a spot (or stay on the GPS fix),
 *     press the save button, type a name (T-Deck keyboard) and it is stored in
 *     <parent of tiledir>/waypoints.json — i.e. next to the maps/ folder on the
 *     SD card. They draw as amber pins with a name label, distinct from the red
 *     GPS dot. Saving with the crosshair on an existing pin renames it; the
 *     dialog then also offers Delete. `maps wp` lists / adds / deletes from the CLI.
 *
 * Config:    s.maps.zoom, s.maps.tiledir
 * Ephemeral: maps.state   ("ready" | "no tiles" | "no fix" | "no sd")
 *
 * This whole file lives under conditional/spangap-lcd/, compiled only when the
 * lcd straddle is staged, so no #if is needed. The maps program (worker, CLI,
 * launcher, Settings pane) registers via the when:-gated mapsLcdRegister hook.
 */
#include "maps.h"
#include "mem.h"
#include "spangap.h"

#include "lcd.h"          /* pulls in lvgl.h */
#include "lcd_input.h"    /* lcdInputGroup — keyboard focus group */
#include "lcd_app.h"      /* LcdApp + lcdInstall */
#include "maps_app.h"     /* MapsApp — this straddle's services: class */
#include "fs.h"
#include "storage.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "jpeg_decoder.h"

#include <sys/stat.h>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <strings.h>

static const char* TAG = "maps";

#define TILE         256
#define TILE_BYTES   (TILE * TILE * 2)
#define MAPS_CACHE   16         /* display tiles + overzoom ancestors + pan margin */
#define MAPS_MAXK    6          /* max ancestor levels for overzoom fallback */
#define ZOOM_MIN     1
#define ZOOM_MAX     19

#define WP_MAX       200        /* saved waypoints */
#define WP_NAME      63         /* name bytes (textarea caps it at WP_NAME_CHARS chars) */
#define WP_NAME_CHARS 24
#define WP_VIS       24         /* waypoints drawn per frame */
#define WP_LABELS    8          /* of those, how many get a name label (nearest first) */
#define WP_NEAR_PX   14         /* crosshair "is on this pin" radius, screen px */
#define WP_FILE      "waypoints.json"

/* ─────────────── worker/config state (worker task owns) ─────────────── */

static TaskHandle_t  s_worker = nullptr;
static volatile bool s_dirty  = true;
static volatile bool s_open   = false;   /* set on the lcd task in mapsApp */

static double s_lat = 0, s_lon = 0;      /* last GPS fix */
static bool   s_haveCenter = false;
static double s_viewLat = 0, s_viewLon = 0;   /* map centre (== GPS when following) */
static bool   s_haveView = false;
static bool   s_follow = true;

/* control flags: lcd task -> worker */
static portMUX_TYPE s_ctrlMux = portMUX_INITIALIZER_UNLOCKED;
static long         s_panDx = 0, s_panDy = 0;   /* accumulated drag, screen px */
static bool         s_recenter = false;
static int          s_zoomReq  = 0;             /* accumulated +/- button steps */

/* waypoint save/delete request: lcd task (or CLI) -> worker, under s_ctrlMux */
enum { WPR_NONE = 0, WPR_SAVE, WPR_DELETE, WPR_DELPT };   /* DELPT: delete the one named/located exactly */
static int    s_wpReq = WPR_NONE;
static char   s_wpReqName[WP_NAME + 1] = {};
static double s_wpReqLat = 0, s_wpReqLon = 0;

/* ─────────────── waypoints (guarded by s_wpMux) ─────────────── */

struct Waypoint { char name[WP_NAME + 1]; double lat, lon; };
PSRAM_BSS static Waypoint s_wp[WP_MAX] = {};
static int               s_wpCount = 0;
static SemaphoreHandle_t s_wpMux = nullptr;
static std::string       s_wpPath;          /* file the list was loaded from ("" = not loaded) */

/* ─────────────── lcd-task-owned widgets (set in mapsApp) ─────────────── */

static lv_obj_t* s_canvas    = nullptr;
static lv_obj_t* s_label     = nullptr;
static uint16_t* s_canvasBuf = nullptr;
static int       s_W = 0, s_H = 0;       /* canvas size, px */
static int       s_stridePx  = 0;        /* row pitch, uint16 units */

/* rounded zoom-level pill, flashed for 2s whenever the effective zoom changes */
static lv_obj_t*   s_zoomWidget    = nullptr;
static lv_obj_t*   s_zoomLabel     = nullptr;
static lv_timer_t* s_zoomHideTimer = nullptr;

static lv_obj_t* s_layer = nullptr;                   /* app root, parent of the dialog */
static lv_obj_t* s_wpLbl[WP_LABELS] = {};             /* waypoint name labels */
static lv_obj_t* s_dlg = nullptr;                     /* save-waypoint dialog (null = closed) */
static lv_obj_t* s_dlgTa = nullptr;
static lv_obj_t* s_list      = nullptr;                   /* waypoint list panel (null = closed) */
static lv_obj_t* s_listTitle = nullptr;
static lv_obj_t* s_listEmpty = nullptr;
static int       s_listCount = 0;
static double    s_lastViewLat = 0, s_lastViewLon = 0;   /* centre of the last composite */
static bool      s_lastHaveView = false;
static char      s_lastNear[WP_NAME + 1] = {};        /* pin under the crosshair, if any */

/* ─────────────── tile cache (shared, guarded by s_mux) ─────────────── */

struct TileBuf { int z, x, y; bool valid; uint16_t* px; uint32_t lru; };
PSRAM_BSS static TileBuf          s_cache[MAPS_CACHE] = {};
static SemaphoreHandle_t s_mux = nullptr;
static uint32_t          s_lruClock = 0;

/* composite request, heap-passed to the lcd task */
enum { MAPS_OK = 0, MAPS_NOFIX, MAPS_NOTILES, MAPS_NOSD };
struct WpVis { short x, y; char name[WP_NAME + 1]; };
struct Composite {
    long tlx, tly; int z; int state; long markerX, markerY; bool haveFix; bool zoomChanged;
    double viewLat, viewLon;       /* map centre (what a save would store) */
    bool   cross;                  /* free-panned: draw the crosshair */
    int    nvis; WpVis vis[WP_VIS];            /* visible waypoints, nearest-to-centre first */
    char   nearName[WP_NAME + 1];  /* pin under the crosshair ("" = none) */
    char   toast[40];              /* one-shot pill text ("" = none) */
};

/* ─────────────── slippy-map math ─────────────── */

/* lon/lat (deg) -> global pixel coordinates at zoom z (256 px/tile). */
static void lonlatToGlobalPx(double lon, double lat, int z, double* gx, double* gy) {
    double n  = (double)(1u << z);
    double xf = (lon + 180.0) / 360.0 * n;
    double lr = lat * M_PI / 180.0;
    double yf = (1.0 - log(tan(lr) + 1.0 / cos(lr)) / M_PI) / 2.0 * n;
    *gx = xf * TILE;
    *gy = yf * TILE;
}

/* inverse: global pixel -> lon/lat (deg). */
static void globalPxToLonLat(double gx, double gy, int z, double* lon, double* lat) {
    double n = (double)(1u << z);
    *lon = gx / TILE / n * 360.0 - 180.0;
    double m = M_PI * (1.0 - 2.0 * (gy / TILE / n));
    *lat = atan(sinh(m)) * 180.0 / M_PI;
}

/* ─────────────── waypoints: storage ─────────────── */

/* The list lives in <parent of s.maps.tiledir>/waypoints.json — next to the
 * maps/ folder, on the same card (the tile dir defaults to /sdcard/maps). */
static std::string wpFilePath(const std::string& tiledir) {
    std::string d = tiledir;
    while (d.size() > 1 && d.back() == '/') d.pop_back();
    size_t sl = d.rfind('/');
    std::string parent = (sl == std::string::npos) ? std::string("/sdcard")
                       : (sl == 0 ? std::string("/") : d.substr(0, sl));
    if (parent.back() != '/') parent += '/';
    return parent + WP_FILE;
}

/* Copy src -> dst (cap WP_NAME bytes, never cutting a UTF-8 sequence), dropping
 * control chars and trimming surrounding spaces. */
static void wpCopyName(char* dst, const char* src) {
    while (*src == ' ') src++;
    size_t n = 0;
    for (; *src && n < WP_NAME; src++)
        if ((unsigned char)*src >= 0x20 && *src != 0x7f) dst[n++] = *src;
    while (n && dst[n - 1] == ' ') n--;
    while (n && n == WP_NAME && ((unsigned char)dst[n - 1] & 0xC0) == 0x80) n--;   /* tail of a cut sequence */
    if (n && (unsigned char)dst[n - 1] >= 0xC0) n--;                                /* orphaned lead byte */
    dst[n] = 0;
}

/* ---- tiny JSON reader: {"waypoints":[{"name":"..","lat":..,"lon":..},..]} ---- */

static const char* jsSkip(const char* p) { while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n' || *p == ',') p++; return p; }

/* Parse a JSON string at *pp (on the opening quote) into out (cap outCap-1). */
static bool jsString(const char** pp, char* out, size_t outCap) {
    const char* p = *pp;
    if (*p != '"') return false;
    p++;
    size_t n = 0;
    while (*p && *p != '"') {
        char ch = *p++;
        if (ch == '\\' && *p) {
            ch = *p++;
            if (ch == 'n' || ch == 't' || ch == 'r') ch = ' ';
            else if (ch == 'u') { for (int i = 0; i < 4 && *p; i++) p++; ch = '?'; }
        }
        if (out && n + 1 < outCap) out[n++] = ch;
    }
    if (*p != '"') return false;
    if (out) out[n] = 0;
    *pp = p + 1;
    return true;
}

static bool wpParse(const char* buf) {
    const char* p = strchr(buf, '[');
    if (!p) return false;
    p++;
    s_wpCount = 0;
    for (;;) {
        p = jsSkip(p);
        if (*p != '{') break;
        p++;
        Waypoint w = {}; bool haveLat = false, haveLon = false;
        for (;;) {
            p = jsSkip(p);
            if (*p == '}') { p++; break; }
            char key[16];
            if (!jsString(&p, key, sizeof key)) return s_wpCount > 0;
            p = jsSkip(p);
            if (*p != ':') return s_wpCount > 0;
            p = jsSkip(p + 1);
            if (*p == '"') {
                char tmp[WP_NAME * 2];
                if (!jsString(&p, tmp, sizeof tmp)) return s_wpCount > 0;
                if (!strcmp(key, "name")) wpCopyName(w.name, tmp);
            } else {
                char* end;
                double v = strtod(p, &end);
                if (end == p) return s_wpCount > 0;
                p = end;
                if (!strcmp(key, "lat")) { w.lat = v; haveLat = true; }
                else if (!strcmp(key, "lon")) { w.lon = v; haveLon = true; }
            }
        }
        if (haveLat && haveLon && s_wpCount < WP_MAX) s_wp[s_wpCount++] = w;
    }
    return true;
}

/* Load `path` into s_wp (caller holds s_wpMux). A missing file is an empty list. */
static void wpLoad(const std::string& path) {
    s_wpCount = 0;
    s_wpPath = path;
    struct stat st;
    if (fs_stat(path.c_str(), &st) != 0) return;
    if (st.st_size <= 0 || st.st_size > 128 * 1024) {
        if (st.st_size > 0) warn("%s too large (%ld B), ignored\n", path.c_str(), (long)st.st_size);
        return;
    }
    size_t len = (size_t)st.st_size;
    char* buf = (char*)heap_caps_malloc(len + 1, MALLOC_CAP_SPIRAM);
    if (!buf) return;
    int f = fs_open(path.c_str(), "rb");
    size_t got = 0;
    if (f >= 0) { got = fs_read(buf, 1, len, f); fs_close(f); }
    buf[got] = 0;
    if (!got || !wpParse(buf)) warn("%s: could not parse, starting empty\n", path.c_str());
    free(buf);
    info("loaded %d waypoint(s) from %s", s_wpCount, path.c_str());
}

/* Write s_wp to s_wpPath via a temp file + rename so a pull-out mid-write keeps
 * the old list (caller holds s_wpMux). Serialised into one PSRAM buffer, then a
 * single fs_write — all SD access goes through the fs worker (this task's stack
 * is in PSRAM, so no raw stdio). */
static bool wpSave() {
    size_t cap = (size_t)s_wpCount * (WP_NAME * 2 + 64) + 64;
    char* buf = (char*)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
    if (!buf) return false;
    size_t n = 0;
    n += snprintf(buf + n, cap - n, "{\"waypoints\":[\n");
    for (int i = 0; i < s_wpCount && n + WP_NAME * 2 + 64 <= cap; i++) {
        char esc[WP_NAME * 2 + 1]; size_t e = 0;
        for (const char* q = s_wp[i].name; *q; q++) {
            if (*q == '"' || *q == '\\') esc[e++] = '\\';
            esc[e++] = *q;
        }
        esc[e] = 0;
        n += snprintf(buf + n, cap - n, "  {\"name\":\"%s\",\"lat\":%.7f,\"lon\":%.7f}%s\n",
                      esc, s_wp[i].lat, s_wp[i].lon, i + 1 < s_wpCount ? "," : "");
    }
    n += snprintf(buf + n, cap - n, "]}\n");

    std::string tmp = s_wpPath + ".tmp";
    int f = fs_open(tmp.c_str(), "wb");
    bool ok = false;
    if (f >= 0) {
        ok = (fs_write(buf, 1, n, f) == n) && (fs_sync(f) == 0);
        fs_close(f);
    } else {
        warn("cannot write %s\n", tmp.c_str());
    }
    free(buf);
    if (ok) {
        fs_remove(s_wpPath.c_str());   /* FATFS rename won't overwrite */
        ok = fs_rename(tmp.c_str(), s_wpPath.c_str()) == 0;
    }
    if (!ok) { warn("saving %s failed\n", s_wpPath.c_str()); fs_remove(tmp.c_str()); }
    return ok;
}

/* Index of the waypoint nearest (cx,cy) on screen within maxPx, or -1. */
static int wpNearest(int zoom, long tlx, long tly, double cx, double cy, double maxPx) {
    int best = -1; double bd = maxPx * maxPx;
    for (int i = 0; i < s_wpCount; i++) {
        double gx, gy;
        lonlatToGlobalPx(s_wp[i].lon, s_wp[i].lat, zoom, &gx, &gy);
        double dx = gx - tlx - cx, dy = gy - tly - cy;
        double d2 = dx * dx + dy * dy;
        if (d2 <= bd) { bd = d2; best = i; }
    }
    return best;
}

/* ─────────────── tile cache (worker task) ─────────────── */

static bool tileInCache(int z, int x, int y) {
    bool hit = false;
    xSemaphoreTake(s_mux, portMAX_DELAY);
    for (auto& t : s_cache)
        if (t.valid && t.z == z && t.x == x && t.y == y) { t.lru = ++s_lruClock; hit = true; break; }
    xSemaphoreGive(s_mux);
    return hit;
}

/* Decode a JPEG tile file into px (256x256 RGB565). false if missing/bad. */
static bool decodeJpg(const char* path, uint16_t* px) {
    struct stat st;
    if (fs_stat(path, &st) != 0 || st.st_size <= 0 || st.st_size > 256 * 1024) return false;
    size_t jlen = (size_t)st.st_size;
    uint8_t* jbuf = (uint8_t*)heap_caps_malloc(jlen, MALLOC_CAP_SPIRAM);
    if (!jbuf) return false;
    int f = fs_open(path, "rb");
    bool rok = (f >= 0) && (fs_read(jbuf, 1, jlen, f) == jlen);
    if (f >= 0) fs_close(f);
    if (!rok) { free(jbuf); return false; }

    esp_jpeg_image_cfg_t cfg = {};
    cfg.indata      = jbuf;
    cfg.indata_size = jlen;
    cfg.outbuf      = (uint8_t*)px;
    cfg.outbuf_size = TILE_BYTES;
    cfg.out_format  = JPEG_IMAGE_FORMAT_RGB565;
    cfg.out_scale   = JPEG_IMAGE_SCALE_0;
    cfg.flags.swap_color_bytes = 0;   /* little-endian to match the LVGL canvas */
    esp_jpeg_image_output_t out = {};
    esp_err_t e = esp_jpeg_decode(&cfg, &out);
    free(jbuf);
    if (e != ESP_OK) {
        /* The file exists and was read fine, but the decoder rejected it — without
         * this, that looks identical to "no tiles for this area". esp_jpeg
         * (TJpgDec) decodes baseline JPEG only: progressive / arithmetic-coded /
         * CMYK / 12-bit tiles fail here. Throttled, because a screenful of bad
         * tiles is retried every GPS update (decode failures aren't cached). */
        static bool     warned = false;
        static uint32_t lastMs = 0;
        uint32_t now = millis();
        if (!warned || now - lastMs > 30000) {
            warn("%s: JPEG decode failed (%s) — esp_jpeg/TJpgDec is baseline-only "
                 "(progressive/arithmetic/CMYK/12-bit rejected); re-encode tiles as "
                 "baseline JPEG\n", path, esp_err_to_name(e));
            warned = true; lastMs = now;
        }
        return false;
    }
    return true;
}

/* Load (z,x,y) into the cache: tries <z>/<x>/<y>.jpg (decoded), then .bin (raw).
 * Returns true if present (cached or just loaded), false if missing / read /
 * alloc / decode failed. File I/O + decode happen outside the lock; only the
 * slot swap is locked. */
static bool ensureTile(const char* dir, int z, int x, int y) {
    if (tileInCache(z, x, y)) return true;

    uint16_t* px = (uint16_t*)heap_caps_malloc(TILE_BYTES, MALLOC_CAP_SPIRAM);
    if (!px) return false;

    char path[192];
    snprintf(path, sizeof(path), "%s/%d/%d/%d.jpg", dir, z, x, y);
    bool ok = decodeJpg(path, px);
    if (!ok) {
        snprintf(path, sizeof(path), "%s/%d/%d/%d.bin", dir, z, x, y);
        int f = fs_open(path, "rb");
        if (f >= 0) { ok = (fs_read(px, 1, TILE_BYTES, f) == TILE_BYTES); fs_close(f); }
    }
    if (!ok) { free(px); return false; }

    xSemaphoreTake(s_mux, portMAX_DELAY);
    int slot = 0; uint32_t best = UINT32_MAX;
    for (int i = 0; i < MAPS_CACHE; i++) {
        if (!s_cache[i].valid) { slot = i; break; }
        if (s_cache[i].lru < best) { best = s_cache[i].lru; slot = i; }
    }
    if (s_cache[slot].valid && s_cache[slot].px) free(s_cache[slot].px);
    s_cache[slot] = { z, x, y, true, px, ++s_lruClock };
    xSemaphoreGive(s_mux);
    return true;
}

/* Cover one display cell: the exact tile if present, else the nearest existing
 * lower-zoom ancestor (overzoom fallback). */
static bool ensureCell(const char* dir, int z, int x, int y) {
    for (int k = 0; k <= MAPS_MAXK && z - k >= 0; k++)
        if (ensureTile(dir, z - k, x >> k, y >> k)) return true;
    return false;
}

/* Is there a *real* (non-overzoom) tile at zoom z for the tile under (lat,lon)?
 * A cache hit proves it exists; otherwise stat the .jpg/.bin on SD. This is what
 * caps zoom to where we actually have data (overzoom ancestors don't count). */
static bool realTileAt(const char* dir, int z, double lat, double lon) {
    if (z < ZOOM_MIN || z > ZOOM_MAX) return false;
    double gx, gy;
    lonlatToGlobalPx(lon, lat, z, &gx, &gy);
    int n  = 1 << z;
    int tx = (int)floor(gx / TILE), ty = (int)floor(gy / TILE);
    if (tx < 0 || ty < 0 || tx >= n || ty >= n) return false;
    if (tileInCache(z, tx, ty)) return true;
    char path[192]; struct stat st;
    snprintf(path, sizeof path, "%s/%d/%d/%d.jpg", dir, z, tx, ty);
    if (fs_stat(path, &st) == 0 && st.st_size > 0) return true;
    snprintf(path, sizeof path, "%s/%d/%d/%d.bin", dir, z, tx, ty);
    if (fs_stat(path, &st) == 0 && st.st_size > 0) return true;
    return false;
}

/* ─────────────── compositing (lcd task) ─────────────── */

static inline void putPx(int x, int y, uint16_t c) {
    if (x < 0 || y < 0 || x >= s_W || y >= s_H) return;
    s_canvasBuf[(size_t)y * s_stridePx + x] = c;
}

/* Blit a sub-region (sx,sy,sw,sh) of a 256-wide tile into the canvas at
 * (destX,destY), nearest-neighbour upscaled by `scale`, clipped to the canvas.
 * scale==1 is the normal 1:1 path (memcpy per row). */
static void blitScaled(const uint16_t* px, int sx, int sy, int sw, int sh,
                       long destX, long destY, int scale) {
    long dw = (long)sw * scale, dh = (long)sh * scale;
    long dx0 = destX < 0 ? 0 : destX;
    long dy0 = destY < 0 ? 0 : destY;
    long dx1 = (destX + dw < s_W) ? destX + dw : s_W;
    long dy1 = (destY + dh < s_H) ? destY + dh : s_H;
    if (dx1 <= dx0 || dy1 <= dy0) return;
    for (long dy = dy0; dy < dy1; dy++) {
        int srow = sy + (int)((dy - destY) / scale);
        const uint16_t* s = px + (size_t)srow * TILE;
        uint16_t* d = s_canvasBuf + (size_t)dy * s_stridePx;
        if (scale == 1) {
            memcpy(d + dx0, s + sx + (dx0 - destX), (size_t)(dx1 - dx0) * 2);
        } else {
            for (long dx = dx0; dx < dx1; dx++)
                d[dx] = s[sx + (int)((dx - destX) / scale)];
        }
    }
}

static void drawMarker(int cx, int cy) {
    for (int dy = -5; dy <= 5; dy++)
        for (int dx = -5; dx <= 5; dx++) {
            int d2 = dx * dx + dy * dy;
            if (d2 <= 25) putPx(cx + dx, cy + dy, 0xFFFF);
            if (d2 <= 9)  putPx(cx + dx, cy + dy, 0xF800);
        }
}

/* Waypoint pin: amber teardrop with a dark outline, tip on (cx,cy). Deliberately
 * unlike the red GPS dot so the two never read as the same thing. */
static void drawPin(int cx, int cy) {
    const int hy = cy - 10;                       /* head centre */
    for (int dy = -7; dy <= 0; dy++) {            /* tail: tapers from the head down to the tip */
        int y = cy + dy;
        int hw = (-dy) * 5 / 7;                   /* half-width, 0 at tip */
        for (int dx = -hw - 1; dx <= hw + 1; dx++) {
            bool edge = (dx < -hw || dx > hw || dy == 0);
            putPx(cx + dx, y, edge ? 0x0000 : 0xFD20);
        }
    }
    for (int dy = -7; dy <= 7; dy++)
        for (int dx = -7; dx <= 7; dx++) {
            int d2 = dx * dx + dy * dy;
            if (d2 <= 41) putPx(cx + dx, hy + dy, 0x0000);
            if (d2 <= 26) putPx(cx + dx, hy + dy, 0xFD20);
            if (d2 <= 4)  putPx(cx + dx, hy + dy, 0xFFFF);
        }
}

/* Crosshair at the map centre while free-panned: shows exactly which point a
 * save would store. White halo + dark line so it reads on any tile colour. */
static void drawCrosshair(int cx, int cy) {
    for (int i = 3; i <= 10; i++) {
        for (int s = -1; s <= 1; s += 2) {
            putPx(cx + s * i, cy - 1, 0xFFFF); putPx(cx + s * i, cy + 1, 0xFFFF);
            putPx(cx - 1, cy + s * i, 0xFFFF); putPx(cx + 1, cy + s * i, 0xFFFF);
            putPx(cx + s * i, cy, 0x0000);
            putPx(cx, cy + s * i, 0x0000);
        }
    }
}

/* Rounded zoom-level pill (lcd task). Shown for 2s on any effective-zoom change
 * — +/- buttons, pinch, or auto zoom-out on leaving coverage — then hidden by a
 * one-shot timer. A repeat change just restarts the 2s window. */
static void zoomHideCb(lv_timer_t* /*t*/) {
    if (s_zoomWidget) lv_obj_add_flag(s_zoomWidget, LV_OBJ_FLAG_HIDDEN);
    s_zoomHideTimer = nullptr;   /* one-shot: LVGL frees it after this fires */
}
static void showPill(const char* text) {
    if (!s_zoomWidget || !s_zoomLabel) return;
    lv_label_set_text(s_zoomLabel, text);
    lv_obj_clear_flag(s_zoomWidget, LV_OBJ_FLAG_HIDDEN);
    if (s_zoomHideTimer) {
        lv_timer_reset(s_zoomHideTimer);
    } else {
        s_zoomHideTimer = lv_timer_create(zoomHideCb, 2000, nullptr);
        lv_timer_set_repeat_count(s_zoomHideTimer, 1);
    }
}
static void showZoomIndicator(int z) {
    char buf[12];
    snprintf(buf, sizeof buf, "z%d", z);
    showPill(buf);
}

static void composite(void* arg) {
    Composite* c = (Composite*)arg;
    if (!s_canvas || !s_canvasBuf) { free(c); return; }

    const uint16_t BG = 0x8410;   /* mid-grey */
    for (int y = 0; y < s_H; y++) {
        uint16_t* row = s_canvasBuf + (size_t)y * s_stridePx;
        for (int x = 0; x < s_W; x++) row[x] = BG;
    }

    int present = 0;
    if (c->state == MAPS_OK) {
        int z = c->z, n = 1 << z;
        int tx0 = (int)floor(c->tlx / (double)TILE), tx1 = (int)floor((c->tlx + s_W - 1) / (double)TILE);
        int ty0 = (int)floor(c->tly / (double)TILE), ty1 = (int)floor((c->tly + s_H - 1) / (double)TILE);
        xSemaphoreTake(s_mux, portMAX_DELAY);
        for (int tx = tx0; tx <= tx1; tx++)
            for (int ty = ty0; ty <= ty1; ty++) {
                if (tx < 0 || ty < 0 || tx >= n || ty >= n) continue;
                long destX = (long)tx * TILE - c->tlx;
                long destY = (long)ty * TILE - c->tly;
                /* exact tile, else nearest cached ancestor upscaled */
                for (int k = 0; k <= MAPS_MAXK && z - k >= 0; k++) {
                    int az = z - k, ax = tx >> k, ay = ty >> k;
                    const uint16_t* px = nullptr;
                    for (auto& t : s_cache)
                        if (t.valid && t.px && t.z == az && t.x == ax && t.y == ay) { px = t.px; break; }
                    if (!px) continue;
                    int sub = TILE >> k;
                    int sx = (tx & ((1 << k) - 1)) * sub;
                    int sy = (ty & ((1 << k) - 1)) * sub;
                    blitScaled(px, sx, sy, sub, sub, destX, destY, 1 << k);
                    present++;
                    break;
                }
            }
        xSemaphoreGive(s_mux);

        for (int i = c->nvis - 1; i >= 0; i--)    /* far-to-near so the nearest pin is on top */
            drawPin(c->vis[i].x, c->vis[i].y);
        if (c->haveFix && c->markerX >= 0 && c->markerX < s_W && c->markerY >= 0 && c->markerY < s_H)
            drawMarker((int)c->markerX, (int)c->markerY);
        if (c->cross) drawCrosshair(s_W / 2, s_H / 2);
    }

    /* waypoint name labels (nearest WP_LABELS), positioned beside each pin's head */
    for (int i = 0; i < WP_LABELS; i++) {
        lv_obj_t* l = s_wpLbl[i];
        if (!l) continue;
        if (c->state == MAPS_OK && i < c->nvis) {
            lv_label_set_text(l, c->vis[i].name);
            lv_obj_set_pos(l, lv_obj_get_x(s_canvas) + c->vis[i].x + 9, lv_obj_get_y(s_canvas) + c->vis[i].y - 21);
            lv_obj_clear_flag(l, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(l, LV_OBJ_FLAG_HIDDEN);
        }
    }
    s_lastHaveView = (c->state == MAPS_OK);
    s_lastViewLat = c->viewLat; s_lastViewLon = c->viewLon;
    snprintf(s_lastNear, sizeof s_lastNear, "%s", c->nearName);

    const char* msg = nullptr;
    char nofix[112];
    if (c->state == MAPS_NOSD) {
        msg = "No SD card";
    } else if (c->state == MAPS_NOFIX) {
        /* Acquisition screen: nudge the user to improve the sky view, with the live
         * satellite count + best signal so progress toward a fix is visible. */
        snprintf(nofix, sizeof nofix,
                 "Make the device see more of the sky\n\n"
                 "Satellites seen: %d\nSignal to noise: %d",
                 storageGetInt("gps.sats_view", 0), storageGetInt("gps.snr", 0));
        msg = nofix;
    } else if (c->state == MAPS_NOTILES || (c->state == MAPS_OK && present == 0)) {
        msg = "No tiles here\n(copy to /sdcard/maps)";
    }
    if (s_label) {
        if (msg) { lv_label_set_text(s_label, msg); lv_obj_clear_flag(s_label, LV_OBJ_FLAG_HIDDEN); }
        else     { lv_obj_add_flag(s_label, LV_OBJ_FLAG_HIDDEN); }
    }

    if (c->toast[0])          showPill(c->toast);
    else if (c->zoomChanged)  showZoomIndicator(c->z);

    lv_obj_invalidate(s_canvas);
    free(c);
}

/* ─────────────── rebuild (worker task) ─────────────── */

static void rebuild(void) {
    if (!s_open) return;   /* program not on screen yet — nothing to draw */

    int base = storageGetInt("s.maps.zoom", 15);
    if (base < ZOOM_MIN) base = ZOOM_MIN;
    if (base > ZOOM_MAX) base = ZOOM_MAX;
    std::string dir = storageGetStr("s.maps.tiledir", "/sdcard/maps");

    std::string la = storageGetStr("gps.lat", "");
    std::string lo = storageGetStr("gps.lon", "");
    bool fix = !la.empty() && !lo.empty();
    if (fix) {
        s_lat = strtod(la.c_str(), nullptr);
        s_lon = strtod(lo.c_str(), nullptr);
        s_haveCenter = true;
    }

    /* consume pan / recentre / zoom-step from the lcd task */
    long dx, dy; bool recenter; int zoomReq;
    int wpReq; char wpName[WP_NAME + 1]; double wpReqLat, wpReqLon;
    portENTER_CRITICAL(&s_ctrlMux);
    wpReq = s_wpReq; s_wpReq = WPR_NONE;
    memcpy(wpName, s_wpReqName, sizeof wpName);
    wpReqLat = s_wpReqLat; wpReqLon = s_wpReqLon;
    dx = s_panDx; dy = s_panDy; s_panDx = 0; s_panDy = 0;
    recenter = s_recenter; s_recenter = false;
    zoomReq = s_zoomReq; s_zoomReq = 0;
    portEXIT_CRITICAL(&s_ctrlMux);
    if (recenter)            s_follow = true;
    if ((dx || dy) && s_haveView) s_follow = false;

    Composite* c = (Composite*)gp_alloc(sizeof(Composite));
    if (!c) return;
    *c = {};
    c->z = base; c->markerX = -1; c->markerY = -1; c->haveFix = fix;

    if (!sdAvailable()) { c->state = MAPS_NOSD;  storageSet("maps.state", "no sd");  lcdRun(composite, c); return; }
    if (!s_haveCenter)  { c->state = MAPS_NOFIX; storageSet("maps.state", "no fix"); lcdRun(composite, c); return; }

    /* view centre: GPS when following, else free-panned (pan is in old-zoom px) */
    if (s_follow) { s_viewLat = s_lat; s_viewLon = s_lon; s_haveView = true; }
    else if (dx || dy) {
        double gx, gy;
        lonlatToGlobalPx(s_viewLon, s_viewLat, base, &gx, &gy);
        gx -= dx; gy -= dy;     /* drag the map under the finger */
        globalPxToLonLat(gx, gy, base, &s_viewLon, &s_viewLat);
    }
    if (!s_haveView) { s_viewLat = s_lat; s_viewLon = s_lon; s_haveView = true; }

    /* Effective zoom, capped to where we actually have tiles around the view.
     * A +/- button step or pinch sets the desired level (base + request); we
     * never zoom IN past the deepest real tile here, and if we've drifted out of
     * coverage at the current level we drop down until a real tile reappears.
     * The capped level is persisted so the slider, the indicator and the next
     * rebuild all agree on it. */
    int zoom = base + zoomReq;
    if (zoom < ZOOM_MIN) zoom = ZOOM_MIN;
    if (zoom > ZOOM_MAX) zoom = ZOOM_MAX;
    while (zoom > base     && !realTileAt(dir.c_str(), zoom, s_viewLat, s_viewLon)) zoom--;
    while (zoom > ZOOM_MIN && !realTileAt(dir.c_str(), zoom, s_viewLat, s_viewLon)) zoom--;
    if (zoom != base) storageSet("s.maps.zoom", zoom);

    /* Flash the zoom pill only when the level actually changed since last draw
     * (the -1 sentinel suppresses a flash on the first render after opening). */
    static int s_renderedZoom = -1;
    c->z = zoom;
    c->zoomChanged = (s_renderedZoom != -1 && s_renderedZoom != zoom);
    s_renderedZoom = zoom;

    double vgx, vgy;
    lonlatToGlobalPx(s_viewLon, s_viewLat, zoom, &vgx, &vgy);
    long tlx = (long)llround(vgx) - s_W / 2;
    long tly = (long)llround(vgy) - s_H / 2;
    c->tlx = tlx; c->tly = tly; c->state = MAPS_OK;
    c->viewLat = s_viewLat; c->viewLon = s_viewLon; c->cross = !s_follow;

    /* ── waypoints: (re)load for this tiledir, apply a pending save/delete, collect the visible ── */
    {
        std::string wpPath = wpFilePath(dir);
        xSemaphoreTake(s_wpMux, portMAX_DELAY);
        if (wpPath != s_wpPath) wpLoad(wpPath);

        const double cx = s_W / 2.0, cy = s_H / 2.0;
        if (wpReq != WPR_NONE) {
            int hit = wpNearest(zoom, tlx, tly, cx, cy, WP_NEAR_PX);
            char nm[WP_NAME + 1]; wpCopyName(nm, wpName);
            if (wpReq == WPR_SAVE) {
                if (!nm[0]) snprintf(nm, sizeof nm, "WP %d", s_wpCount + 1);
                if (hit >= 0) {                                        /* on an existing pin: rename it */
                    snprintf(s_wp[hit].name, sizeof s_wp[hit].name, "%s", nm);
                } else if (s_wpCount >= WP_MAX) {
                    snprintf(c->toast, sizeof c->toast, "Waypoints full (%d)", WP_MAX);
                    nm[0] = 0;
                } else {
                    Waypoint& w = s_wp[s_wpCount++];
                    snprintf(w.name, sizeof w.name, "%s", nm);
                    w.lat = s_viewLat; w.lon = s_viewLon;
                }
                if (nm[0]) {
                    if (wpSave()) snprintf(c->toast, sizeof c->toast, "Saved: %.24s", nm);
                    else          snprintf(c->toast, sizeof c->toast, "Save failed (SD?)");
                }
            } else if (wpReq == WPR_DELPT) {                           /* from the list: exact match */
                int di = -1;
                for (int i = 0; i < s_wpCount; i++)
                    if (!strcmp(s_wp[i].name, nm) && fabs(s_wp[i].lat - wpReqLat) < 1e-6 && fabs(s_wp[i].lon - wpReqLon) < 1e-6) { di = i; break; }
                if (di >= 0) {
                    memmove(&s_wp[di], &s_wp[di + 1], (size_t)(s_wpCount - di - 1) * sizeof(Waypoint));
                    s_wpCount--;
                    if (wpSave()) snprintf(c->toast, sizeof c->toast, "Deleted: %.24s", nm);
                    else          snprintf(c->toast, sizeof c->toast, "Delete failed (SD?)");
                }
            } else if (hit >= 0) {                                     /* WPR_DELETE */
                snprintf(nm, sizeof nm, "%s", s_wp[hit].name);
                memmove(&s_wp[hit], &s_wp[hit + 1], (size_t)(s_wpCount - hit - 1) * sizeof(Waypoint));
                s_wpCount--;
                if (wpSave()) snprintf(c->toast, sizeof c->toast, "Deleted: %.24s", nm);
                else          snprintf(c->toast, sizeof c->toast, "Delete failed (SD?)");
            }
        }

        int nearIdx = wpNearest(zoom, tlx, tly, cx, cy, WP_NEAR_PX);
        if (nearIdx >= 0) snprintf(c->nearName, sizeof c->nearName, "%s", s_wp[nearIdx].name);

        /* visible pins, kept sorted nearest-to-centre (insertion into a tiny array) */
        double dist[WP_VIS];
        for (int i = 0; i < s_wpCount; i++) {
            double gx, gy;
            lonlatToGlobalPx(s_wp[i].lon, s_wp[i].lat, zoom, &gx, &gy);
            double mx = gx - tlx, my = gy - tly;
            if (mx < -12 || mx > s_W + 12 || my < -4 || my > s_H + 24) continue;
            double d = (mx - cx) * (mx - cx) + (my - cy) * (my - cy);
            int pos = c->nvis;
            if (pos == WP_VIS) { if (d >= dist[WP_VIS - 1]) continue; pos = WP_VIS - 1; }
            else c->nvis++;
            while (pos > 0 && dist[pos - 1] > d) { dist[pos] = dist[pos - 1]; c->vis[pos] = c->vis[pos - 1]; pos--; }
            dist[pos] = d;
            c->vis[pos].x = (short)llround(mx); c->vis[pos].y = (short)llround(my);
            snprintf(c->vis[pos].name, sizeof c->vis[pos].name, "%s", s_wp[i].name);
        }
        xSemaphoreGive(s_wpMux);
    }

    /* marker = GPS pixel relative to top-left (may fall off-screen when panned) */
    double ggx, ggy;
    lonlatToGlobalPx(s_lon, s_lat, zoom, &ggx, &ggy);
    c->markerX = (long)llround(ggx) - tlx;
    c->markerY = (long)llround(ggy) - tly;

    int n   = 1 << zoom;
    int tx0 = (int)floor(tlx / (double)TILE), tx1 = (int)floor((tlx + s_W - 1) / (double)TILE);
    int ty0 = (int)floor(tly / (double)TILE), ty1 = (int)floor((tly + s_H - 1) / (double)TILE);
    int present = 0;
    for (int tx = tx0; tx <= tx1; tx++)
        for (int ty = ty0; ty <= ty1; ty++) {
            if (tx < 0 || ty < 0 || tx >= n || ty >= n) continue;
            if (ensureCell(dir.c_str(), zoom, tx, ty)) present++;
        }

    storageSet("maps.state", present ? "ready" : "no tiles");
    lcdRun(composite, c);
}

static void wakeWorker(void) {
    s_dirty = true;
    if (s_worker) xTaskNotifyGive(s_worker);
}

static void onChange(const char* /*k*/, const char* /*v*/) { wakeWorker(); }

static void mapsWorker(void*) {
    info("task up");
    itsClientInit(2);
    /* Only the fix position re-centres the map. Subscribing to the whole "gps"
     * scope pulled in the ~1 Hz telemetry churn (sats/snr/dop/utc/fix_age) too,
     * which overflowed this worker's inbox while it was busy reading SD tiles —
     * a "notify drop" warn storm. lat/lon are all rebuild() actually reads. */
    storageSubscribeChanges("gps.lat", onChange);  /* centre */
    storageSubscribeChanges("gps.lon", onChange);
    storageSubscribeChanges("s.maps",  onChange);  /* zoom / tiledir */
    for (;;) {
        if (s_dirty) { s_dirty = false; rebuild(); }
        itsPoll(portMAX_DELAY);
    }
}

/* ─────────────── input (lcd task) ─────────────── */

/* Drag to pan (touch or trackball-press-drag). Accumulate the indev movement
 * vector; the worker applies it and re-locks follow off. */
static void mapPressCb(lv_event_t* e) {
    lv_indev_t* indev = lv_event_get_indev(e);
    if (!indev) return;
    lv_point_t v;
    lv_indev_get_vect(indev, &v);
    if (v.x == 0 && v.y == 0) return;
    portENTER_CRITICAL(&s_ctrlMux);
    s_panDx += v.x; s_panDy += v.y;
    portEXIT_CRITICAL(&s_ctrlMux);
    wakeWorker();
}

/* Edge-pan from the trackball (lcdProgramScrollHandler): the lcd component hands
 * us the pan delta when the cursor is driven into a screen edge, in the same sign
 * convention as the finger-drag vector above — so the map canvas, which isn't an
 * LVGL scroll container, pans on a touchless deck just as it does under a finger. */
static void mapsScrollCb(int dx, int dy) {
    portENTER_CRITICAL(&s_ctrlMux);
    s_panDx += dx; s_panDy += dy;
    portEXIT_CRITICAL(&s_ctrlMux);
    wakeWorker();
}

static void mapCenterCb(lv_event_t* /*e*/) {
    portENTER_CRITICAL(&s_ctrlMux);
    s_recenter = true;
    portEXIT_CRITICAL(&s_ctrlMux);
    wakeWorker();
}

/* +/- zoom buttons: post a step to the worker, which applies it capped to the
 * tiles available around the current view (see rebuild). */
static void mapZoomInCb(lv_event_t* /*e*/) {
    portENTER_CRITICAL(&s_ctrlMux);
    s_zoomReq += 1;
    portEXIT_CRITICAL(&s_ctrlMux);
    wakeWorker();
}
static void mapZoomOutCb(lv_event_t* /*e*/) {
    portENTER_CRITICAL(&s_ctrlMux);
    s_zoomReq -= 1;
    portEXIT_CRITICAL(&s_ctrlMux);
    wakeWorker();
}

/* Pinch-to-zoom (lcd task, via the lcd gesture callback). Two fingers: live-
 * scale the canvas for feedback; on release step s.maps.zoom by the nearest
 * power-of-two of the pinch ratio. The worker then refetches at the new integer
 * zoom; overzoom fallback keeps the view non-grey across the change. */
static bool   s_pinchActive   = false;
static double s_pinchInitDist = 1;
static double s_pinchRatio    = 1;
static int    s_pinchZoom     = 15;

static void mapsGesture(const lcd_touch_pt_t* pts, int count) {
    if (count >= 2) {
        double dx = pts[0].x - pts[1].x, dy = pts[0].y - pts[1].y;
        double dist = sqrt(dx * dx + dy * dy);
        if (dist < 1) dist = 1;
        if (!s_pinchActive) {
            s_pinchActive   = true;
            s_pinchInitDist = dist;
            s_pinchZoom     = storageGetInt("s.maps.zoom", 15);
            if (s_canvas) {
                lv_obj_set_style_transform_pivot_x(s_canvas, (pts[0].x + pts[1].x) / 2, 0);
                lv_obj_set_style_transform_pivot_y(s_canvas, (pts[0].y + pts[1].y) / 2, 0);
            }
        }
        s_pinchRatio = dist / s_pinchInitDist;
        if (s_canvas) {
            int sc = (int)(256.0 * s_pinchRatio);    /* LVGL scale: 256 = 1.0x */
            if (sc < 64) sc = 64; else if (sc > 1024) sc = 1024;
            lv_obj_set_style_transform_scale(s_canvas, sc, 0);
        }
    } else if (s_pinchActive) {
        s_pinchActive = false;
        if (s_canvas) lv_obj_set_style_transform_scale(s_canvas, 256, 0);   /* back to 1.0x */
        int steps = (int)lround(log2(s_pinchRatio));
        s_pinchRatio = 1;
        if (steps != 0) {
            int nz = s_pinchZoom + steps;
            if (nz < ZOOM_MIN) nz = ZOOM_MIN; else if (nz > ZOOM_MAX) nz = ZOOM_MAX;
            if (nz != s_pinchZoom) storageSet("s.maps.zoom", nz);   /* -> worker rebuild */
        }
    }
}

/* ─────────────── waypoint dialog (lcd task) ─────────────── */

static void wpPost(int kind, const char* name, double lat = 0, double lon = 0) {
    portENTER_CRITICAL(&s_ctrlMux);
    s_wpReq = kind;
    s_wpReqLat = lat; s_wpReqLon = lon;
    snprintf(s_wpReqName, sizeof s_wpReqName, "%s", name ? name : "");
    portEXIT_CRITICAL(&s_ctrlMux);
    wakeWorker();
}

static void wpDlgClose(void) {
    if (s_dlg) { lv_obj_delete(s_dlg); s_dlg = nullptr; s_dlgTa = nullptr; }
}
static void wpDlgSave(lv_event_t*) {
    if (s_dlgTa) wpPost(WPR_SAVE, lv_textarea_get_text(s_dlgTa));
    wpDlgClose();
}
static void wpDlgDelete(lv_event_t*) { wpPost(WPR_DELETE, nullptr); wpDlgClose(); }
static void wpDlgCancel(lv_event_t*) { wpDlgClose(); }
static void wpDlgKey(lv_event_t* e) {
    if (lv_event_get_key(e) == LV_KEY_ESC) wpDlgClose();
}

static lv_obj_t* wpDlgBtn(lv_obj_t* row, const char* text, lv_event_cb_t cb) {
    lv_obj_t* b = lv_button_create(row);
    lv_obj_set_height(b, 28);
    lv_obj_t* l = lv_label_create(b);
    lv_label_set_text(l, text);
    lv_obj_center(l);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
    if (lcdInputGroup()) lv_group_add_obj(lcdInputGroup(), b);
    return b;
}

/* Save button: opens a small panel at the top of the map (so the on-screen
 * keyboard on touch-only devices, which docks at the bottom, doesn't cover it)
 * with the centre coordinates, a one-line name field and Save / Delete / Cancel.
 * On a T-Deck the field is focused straight away, so you just type and press
 * Enter. If the crosshair is on an existing pin the field is prefilled with its
 * name: Enter renames it, Delete removes it. */
static void mapSaveCb(lv_event_t*) {
    if (s_dlg || s_list || !s_layer) return;
    if (!s_lastHaveView) return;   /* nothing on screen to anchor a waypoint to yet */

    s_dlg = lv_obj_create(s_layer);
    lv_obj_set_size(s_dlg, s_W - 16, LV_SIZE_CONTENT);
    lv_obj_align(s_dlg, LV_ALIGN_TOP_MID, 0, 6);
    lv_obj_set_style_pad_all(s_dlg, 6, 0);
    lv_obj_set_style_pad_row(s_dlg, 4, 0);
    lv_obj_set_flex_flow(s_dlg, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(s_dlg, LV_OBJ_FLAG_SCROLLABLE);

    bool editing = s_lastNear[0] != 0;
    lv_obj_t* title = lv_label_create(s_dlg);
    lv_label_set_text(title, editing ? "Edit waypoint" : "Save waypoint");

    char pos[48];
    snprintf(pos, sizeof pos, "%.5f, %.5f", s_lastViewLat, s_lastViewLon);
    lv_obj_t* coord = lv_label_create(s_dlg);
    lv_label_set_text(coord, pos);

    s_dlgTa = lv_textarea_create(s_dlg);
    lv_textarea_set_one_line(s_dlgTa, true);
    lv_textarea_set_max_length(s_dlgTa, WP_NAME_CHARS);
    lv_textarea_set_placeholder_text(s_dlgTa, "Name");
    lv_obj_set_width(s_dlgTa, lv_pct(100));
    if (editing) lv_textarea_set_text(s_dlgTa, s_lastNear);
    lv_obj_add_event_cb(s_dlgTa, wpDlgSave, LV_EVENT_READY, nullptr);   /* Enter */
    lv_obj_add_event_cb(s_dlgTa, wpDlgKey, LV_EVENT_KEY, nullptr);      /* Esc */
    if (lcdInputGroup()) { lv_group_add_obj(lcdInputGroup(), s_dlgTa); lv_group_focus_obj(s_dlgTa); }
    lcdKeyboardAttach(s_dlgTa);   /* no keys on the device -> a tap types, the check fires READY */

    lv_obj_t* row = lv_obj_create(s_dlg);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(row, 6, 0);
    wpDlgBtn(row, "Save", wpDlgSave);
    if (editing) wpDlgBtn(row, "Delete", wpDlgDelete);
    wpDlgBtn(row, "Cancel", wpDlgCancel);
}

/* ── waypoint list: every saved pin, with a delete button per row ── */

struct WpRow { double lat, lon; char name[WP_NAME + 1]; bool armed; lv_obj_t* row; lv_obj_t* btnLbl; };

static void wpListClose(void) {
    if (s_list) { lv_obj_delete(s_list); s_list = nullptr; s_listTitle = s_listEmpty = nullptr; }
}
static void wpListCloseCb(lv_event_t*) { wpListClose(); }
static void wpListKey(lv_event_t* e) {
    if (lv_event_get_key(e) == LV_KEY_ESC) wpListClose();
}
static void wpRowFree(lv_event_t* e) { free(lv_event_get_user_data(e)); }

static void wpListRefreshTitle(void) {
    if (s_listTitle) {
        char t[32];
        snprintf(t, sizeof t, "Waypoints (%d)", s_listCount);
        lv_label_set_text(s_listTitle, t);
    }
    if (s_listEmpty) {
        if (s_listCount == 0) lv_obj_clear_flag(s_listEmpty, LV_OBJ_FLAG_HIDDEN);
        else                  lv_obj_add_flag(s_listEmpty, LV_OBJ_FLAG_HIDDEN);
    }
}

/* First press arms the button ("Sure?"), second deletes — an Enter on a
 * focused trash button must not be enough on its own. */
static void wpRowDelCb(lv_event_t* e) {
    WpRow* r = (WpRow*)lv_event_get_user_data(e);
    lv_obj_t* btn = lv_event_get_target_obj(e);
    if (!r->armed) {
        r->armed = true;
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xC02020), 0);
        lv_obj_set_width(btn, 56);
        lv_label_set_text(r->btnLbl, "Sure?");
        return;
    }
    wpPost(WPR_DELPT, r->name, r->lat, r->lon);
    lv_obj_t* row = r->row;      /* r is freed by the row's delete event */
    s_listCount--;
    lv_obj_delete_async(row);
    wpListRefreshTitle();
}

static void mapListCb(lv_event_t*) {
    if (s_list || s_dlg || !s_layer || !s_lastHaveView) return;

    s_list = lv_obj_create(s_layer);
    lv_obj_set_size(s_list, s_W - 16, s_H - 16);
    lv_obj_center(s_list);
    lv_obj_set_style_pad_all(s_list, 6, 0);
    lv_obj_set_style_pad_row(s_list, 4, 0);
    lv_obj_set_flex_flow(s_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(s_list, LV_OBJ_FLAG_SCROLLABLE);

    /* header: title + Close */
    lv_obj_t* head = lv_obj_create(s_list);
    lv_obj_remove_style_all(head);
    lv_obj_set_size(head, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(head, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(head, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    s_listTitle = lv_label_create(head);
    lv_obj_t* close = lv_button_create(head);
    lv_obj_set_height(close, 28);
    lv_obj_t* cl = lv_label_create(close);
    lv_label_set_text(cl, "Close");
    lv_obj_center(cl);
    lv_obj_add_event_cb(close, wpListCloseCb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(close, wpListKey, LV_EVENT_KEY, nullptr);

    /* scrolling body */
    lv_obj_t* body = lv_obj_create(s_list);
    lv_obj_remove_style_all(body);
    lv_obj_set_width(body, lv_pct(100));
    lv_obj_set_flex_grow(body, 1);
    lv_obj_set_flex_flow(body, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(body, 4, 0);
    lv_obj_add_flag(body, LV_OBJ_FLAG_SCROLLABLE);

    s_listEmpty = lv_label_create(body);
    lv_label_set_text(s_listEmpty, "No waypoints saved yet");
    lv_obj_add_flag(s_listEmpty, LV_OBJ_FLAG_HIDDEN);

    s_listCount = 0;
    xSemaphoreTake(s_wpMux, portMAX_DELAY);
    for (int i = 0; i < s_wpCount; i++) {
        WpRow* r = (WpRow*)calloc(1, sizeof(WpRow));
        if (!r) break;
        r->lat = s_wp[i].lat; r->lon = s_wp[i].lon;
        snprintf(r->name, sizeof r->name, "%s", s_wp[i].name);

        lv_obj_t* row = lv_obj_create(body);
        r->row = row;
        lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_style_pad_all(row, 4, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(row, wpRowFree, LV_EVENT_DELETE, r);

        lv_obj_t* col = lv_obj_create(row);
        lv_obj_remove_style_all(col);
        lv_obj_set_height(col, LV_SIZE_CONTENT);
        lv_obj_set_flex_grow(col, 1);
        lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
        lv_obj_t* nm = lv_label_create(col);
        lv_label_set_long_mode(nm, LV_LABEL_LONG_DOT);
        lv_obj_set_width(nm, lv_pct(100));
        lv_label_set_text(nm, r->name);
        lv_obj_t* co = lv_label_create(col);
        char cb[40];
        snprintf(cb, sizeof cb, "%.5f, %.5f", r->lat, r->lon);
        lv_label_set_text(co, cb);
        lv_obj_set_style_text_color(co, lv_color_hex(0x808080), 0);

        lv_obj_t* del = lv_button_create(row);
        lv_obj_set_size(del, 36, 28);
        lv_obj_t* dl = lv_label_create(del);
        lv_label_set_text(dl, LV_SYMBOL_TRASH);
        lv_obj_center(dl);
        r->btnLbl = dl;
        lv_obj_add_event_cb(del, wpRowDelCb, LV_EVENT_CLICKED, r);
        lv_obj_add_event_cb(del, wpListKey, LV_EVENT_KEY, nullptr);
        if (lcdInputGroup()) lv_group_add_obj(lcdInputGroup(), del);
        s_listCount++;
    }
    xSemaphoreGive(s_wpMux);
    wpListRefreshTitle();

    /* Close first in the focus order and focused: the safe default for Enter. */
    if (lcdInputGroup()) { lv_group_add_obj(lcdInputGroup(), close); lv_group_focus_obj(close); }
}

/* ─────────────── launcher program (lcd task) ─────────────── */

/* A white-background control button, same size/shape/corner as the centre-me
 * button, aligned to the bottom-left with the given vertical offset. */
static lv_obj_t* makeGreyBtn(lv_obj_t* parent, const char* label, int yoff, lv_event_cb_t cb) {
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_size(b, 26, 26);
    lv_obj_align(b, LV_ALIGN_BOTTOM_LEFT, 8, yoff);
    lv_obj_set_style_bg_color(b, lv_color_white(), 0);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* l = lv_label_create(b);
    lv_label_set_text(l, label);
    lv_obj_set_style_text_color(l, lv_color_black(), 0);   /* dark glyph stays legible on white */
    lv_obj_center(l);
    return b;
}

static void mapsBuild(lv_obj_t* layer) {
    int W = lv_obj_get_content_width(layer);
    int H = lv_obj_get_content_height(layer);
    if (W <= 0) W = 320;
    if (H <= 0) H = 240;

    uint32_t stride = lv_draw_buf_width_to_stride((uint32_t)W, LV_COLOR_FORMAT_RGB565);
    s_canvasBuf = (uint16_t*)heap_caps_malloc((size_t)stride * H, MALLOC_CAP_SPIRAM);
    if (!s_canvasBuf) { err("canvas alloc failed (%dx%d)", W, H); return; }
    s_W = W; s_H = H; s_stridePx = (int)(stride / 2);

    s_canvas = lv_canvas_create(layer);
    lv_canvas_set_buffer(s_canvas, s_canvasBuf, W, H, LV_COLOR_FORMAT_RGB565);
    lv_canvas_fill_bg(s_canvas, lv_color_hex(0x808080), LV_OPA_COVER);
    lv_obj_center(s_canvas);
    lv_obj_add_flag(s_canvas, LV_OBJ_FLAG_CLICKABLE);    /* so it gets press/drag */
    lv_obj_add_event_cb(s_canvas, mapPressCb, LV_EVENT_PRESSING, nullptr);
    lcdProgramScrollHandler(mapsScrollCb);   /* trackball edge-pan -> map pan */

    s_layer = layer;

    /* waypoint name labels: a small pool, repositioned on every composite */
    for (int i = 0; i < WP_LABELS; i++) {
        lv_obj_t* l = lv_label_create(layer);
        lv_label_set_long_mode(l, LV_LABEL_LONG_DOT);
        lv_obj_set_style_max_width(l, 120, 0);
        lv_obj_set_style_bg_color(l, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(l, LV_OPA_80, 0);
        lv_obj_set_style_text_color(l, lv_color_black(), 0);
        lv_obj_set_style_radius(l, 3, 0);
        lv_obj_set_style_pad_hor(l, 3, 0);
        lv_obj_add_flag(l, LV_OBJ_FLAG_HIDDEN);
        s_wpLbl[i] = l;
    }

    s_label = lv_label_create(layer);
    lv_label_set_text(s_label, "Waiting for GPS...");
    lv_obj_set_style_text_align(s_label, LV_TEXT_ALIGN_CENTER, 0);   /* multi-line status reads centred */
    lv_obj_center(s_label);

    /* "centre me" button (re-lock to GPS) */
    lv_obj_t* btn = lv_button_create(layer);
    lv_obj_set_size(btn, 26, 26);   /* ~60% of the old 44px */
    lv_obj_align(btn, LV_ALIGN_BOTTOM_RIGHT, -8, -8);
    lv_obj_add_event_cb(btn, mapCenterCb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* bl = lv_label_create(btn);
    lv_label_set_text(bl, LV_SYMBOL_GPS);
    lv_obj_center(bl);

    /* "waypoint list" button, above the save button */
    lv_obj_t* lbtn = lv_button_create(layer);
    lv_obj_set_size(lbtn, 26, 26);
    lv_obj_align(lbtn, LV_ALIGN_BOTTOM_RIGHT, -8, -72);
    lv_obj_add_event_cb(lbtn, mapListCb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* ll = lv_label_create(lbtn);
    lv_label_set_text(ll, LV_SYMBOL_LIST);
    lv_obj_center(ll);

    /* "save waypoint" button, above centre-me */
    lv_obj_t* sbtn = lv_button_create(layer);
    lv_obj_set_size(sbtn, 26, 26);
    lv_obj_align(sbtn, LV_ALIGN_BOTTOM_RIGHT, -8, -40);
    lv_obj_add_event_cb(sbtn, mapSaveCb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* sl = lv_label_create(sbtn);
    lv_label_set_text(sl, LV_SYMBOL_PLUS);
    lv_obj_center(sl);

    /* zoom +/- buttons, stacked in the bottom-left ('+' above '-'). The worker
     * caps the step to the tiles actually present around the view. */
    makeGreyBtn(layer, "+", -40, mapZoomInCb);
    makeGreyBtn(layer, "-", -8,  mapZoomOutCb);

    /* rounded zoom-level pill above the buttons, hidden until a zoom change */
    s_zoomWidget = lv_obj_create(layer);
    lv_obj_remove_style_all(s_zoomWidget);
    lv_obj_set_size(s_zoomWidget, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(s_zoomWidget, lv_color_hex(0x303030), 0);
    lv_obj_set_style_bg_opa(s_zoomWidget, LV_OPA_80, 0);
    lv_obj_set_style_radius(s_zoomWidget, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_pad_left(s_zoomWidget, 10, 0);
    lv_obj_set_style_pad_right(s_zoomWidget, 10, 0);
    lv_obj_set_style_pad_top(s_zoomWidget, 4, 0);
    lv_obj_set_style_pad_bottom(s_zoomWidget, 4, 0);
    lv_obj_clear_flag(s_zoomWidget, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(s_zoomWidget, LV_ALIGN_BOTTOM_LEFT, 8, -74);
    s_zoomLabel = lv_label_create(s_zoomWidget);
    lv_obj_set_style_text_color(s_zoomLabel, lv_color_white(), 0);
    lv_label_set_text(s_zoomLabel, "z15");
    lv_obj_center(s_zoomLabel);
    lv_obj_add_flag(s_zoomWidget, LV_OBJ_FLAG_HIDDEN);

    static bool s_gestureHooked = false;   /* fixed handler set: register once */
    if (!s_gestureHooked) { lcdTouchAddGestureHandler(mapsGesture); s_gestureHooked = true; }
}

/* MapsApp — the launcher program as an LcdApp (and thus a Service). onCreate
 * builds the canvas once; onShow (re)arms multi-touch + the render worker on
 * every open; onClose tears the canvas down so a later reopen rebuilds (the old
 * static-pointer reopen shortcut dangled after the layer was deleted, hanging
 * the render worker). Declared in maps_app.h (global, so the generated
 * services: trampoline can `new` it); defined here where the file-static viewer
 * state lives. */
MapsApp::MapsApp() : LcdApp({ .name = "Maps", .iconBasename = "maps" }) {}

void MapsApp::onCreate(lv_obj_t* root) { mapsBuild(root); }

void MapsApp::onShow() {
    storageSet("lcd.multi_touch", 1);   /* pinch gestures while open */
    s_open = true; wakeWorker();
}

void MapsApp::onClose() {
    s_open = false;
    storageSet("lcd.multi_touch", 0);
    if (s_zoomHideTimer) { lv_timer_delete(s_zoomHideTimer); s_zoomHideTimer = nullptr; }
    s_canvas = nullptr; s_label = nullptr;
    s_layer = nullptr; s_dlg = nullptr; s_dlgTa = nullptr;
    s_list = nullptr; s_listTitle = nullptr; s_listEmpty = nullptr;
    for (auto& l : s_wpLbl) l = nullptr;
    s_lastHaveView = false;
    s_zoomWidget = nullptr; s_zoomLabel = nullptr;
    if (s_canvasBuf) { free(s_canvasBuf); s_canvasBuf = nullptr; }
}

/* ─────────────── CLI + settings ─────────────── */

static void cliMaps(const char* args) {
    if (args && strcmp(args, "help") == 0) { cliPrintf("%-*s map status; center to recentre on GPS\n", CLI_HELP_COL, "maps [center]"); return; }
    if (args && cliWantsHelp(args)) {
        cliPrintf("%-*s map status\n",        CLI_HELP_COL, "maps");
        cliPrintf("%-*s recentre on GPS\n",   CLI_HELP_COL, "maps center");
        cliPrintf("%-*s list / add at view centre / delete waypoints\n", CLI_HELP_COL, "maps wp [add|del <name>]");
        return;
    }
    if (args && strncmp(args, "wp", 2) == 0 && (args[2] == 0 || args[2] == ' ')) {
        const char* a = args + 2;
        while (*a == ' ') a++;
        if (!strncmp(a, "add ", 4)) {
            if (!s_haveView) { cliPrintf("no map centre yet (open Maps, wait for a fix)\n"); return; }
            wpPost(WPR_SAVE, a + 4);
            cliPrintf("saving at view centre %.6f, %.6f\n", s_viewLat, s_viewLon);
            return;
        }
        if (!strncmp(a, "del ", 4)) {
            const char* nm = a + 4;
            bool found = false;
            xSemaphoreTake(s_wpMux, portMAX_DELAY);
            for (int i = 0; i < s_wpCount && !found; i++)
                if (!strcasecmp(s_wp[i].name, nm)) {
                    memmove(&s_wp[i], &s_wp[i + 1], (size_t)(s_wpCount - i - 1) * sizeof(Waypoint));
                    s_wpCount--; found = true;
                }
            bool ok = found && wpSave();
            xSemaphoreGive(s_wpMux);
            wakeWorker();
            cliPrintf(found ? (ok ? "deleted\n" : "deleted in memory, SD write failed\n") : "no such waypoint\n");
            return;
        }
        xSemaphoreTake(s_wpMux, portMAX_DELAY);
        if (s_wpPath.empty()) cliPrintf("(not loaded yet — open Maps once)\n");
        else cliPrintf("%d waypoint(s) in %s\n", s_wpCount, s_wpPath.c_str());
        for (int i = 0; i < s_wpCount; i++) cliPrintf("  %-24s %.6f, %.6f\n", s_wp[i].name, s_wp[i].lat, s_wp[i].lon);
        xSemaphoreGive(s_wpMux);
        return;
    }
    if (args && strcmp(args, "center") == 0) { mapCenterCb(nullptr); cliPrintf("recentred\n"); return; }

    cliPrintf("state:   %s\n", storageGetStr("maps.state", "-").c_str());
    cliPrintf("sd:      %s\n", sdAvailable() ? "mounted" : "none");
    cliPrintf("zoom:    %d\n", storageGetInt("s.maps.zoom", 15));
    cliPrintf("follow:  %s\n", s_follow ? "yes" : "no (panned)");
    cliPrintf("tiledir: %s\n", storageGetStr("s.maps.tiledir", "/sdcard/maps").c_str());
    if (s_haveCenter) cliPrintf("gps:     %.6f, %.6f\n", s_lat, s_lon);
    else              cliPrintf("gps:     (no fix yet)\n");
    if (s_haveView)   cliPrintf("view:    %.6f, %.6f\n", s_viewLat, s_viewLon);
}

/* ─────────────── boot-task wiring (appInit) ─────────────── */

/* MapsApp::appInit — the boot-task half of bring-up, run once by
 * LcdApp::onInit() right after it hops the launcher-tile install onto the lcd
 * task. This whole file lives under conditional/spangap-lcd/, compiled only when
 * the lcd straddle is staged, so no #if is needed — no lcd, no MapsApp, no
 * services: registration (matching the old no-op mapsInit in non-LCD builds).
 * Storage defaults and the Settings pane are generated from the settings: block
 * in straddle.yaml, not wired here. */
void MapsApp::appInit() {
    /* GPS is the board's call, not ours: hw-lilygo-tdeck defaults s.gps.enable on (it
     * has the hardware), and the user owns it thereafter. Forcing it on at every
     * maps init clobbered that choice on every boot. */
    s_mux = xSemaphoreCreateMutex();
    s_wpMux = xSemaphoreCreateMutex();
    cliRegisterCmd("maps", cliMaps);
    s_worker = spawnTask(mapsWorker, TAG, 8192, nullptr, 1, 1, STACK_PSRAM);
}
