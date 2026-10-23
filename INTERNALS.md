# maps — internals

Maintainer reference for the on-device viewer and the `tilebake` pipeline. The
[README](README.md) is the operator guide. The desktop-renders / device-blits
division and the raw-tile idea follow
[`esp32_offline_osm`](https://github.com/mryndzionek/esp32_offline_osm); the
device code, the LVGL compositor, and `tilebake` are original to this straddle.

## Tile format

`/<z>/<x>/<y>` standard slippy-map tiles, 256×256, in one of two encodings;
the worker tries them in order:

- **`.jpg`** — baseline JPEG (what `tilebake` writes by default, quality 80).
  Decoded on the worker via `esp_jpeg` (TJpgDec). **Baseline only** —
  progressive / arithmetic / CMYK / 12-bit JPEGs are rejected by the decoder.
- **`.bin`** — raw little-endian RGB565, exactly `256*256*2 = 131072` bytes, no
  header. LVGL's native canvas order (byte-swapped for the ST7789 on flush —
  `esp32_offline_osm` instead stores its raw tiles in the panel's direct byte
  order for a straight-to-panel path; this viewer matches the LVGL canvas and
  lets the flush path do the swap, and tells the JPEG decoder to emit
  little-endian too). Bigger on disk but zero-decode; the fallback when no `.jpg`
  is present.

Slippy coords are **global**: tile `(z, x, y)` is the same geographic square no
matter which source produced it. The device just reads whatever tile is at that
path; all the work of merging maps into one coherent tree happens at bake time
(see the bake pipeline's combine step).

**SD cluster-size gotcha:** a tile tree is 10⁵+ tiny files. On a card with large
FAT clusters (exFAT on big SDXC cards defaults to 64–128 KB) each ~7 KB tile
consumes a whole cluster, so `du` on the card can read ~10× the real data. Format
the card **FAT32 with small clusters** (8 KB ≈ one cluster per tile); the
firmware's own `format sd [KB]` / `CONFIG_SPANGAP_SDCARD_ALLOC_KB` default to
8 KB for the same reason (the device's FatFs has no exFAT support, so big cards
must be FAT32 anyway).

## Two halves

- **Worker task** (`maps_lcd.cpp`) — owns the PSRAM tile cache (LRU, ~16 tiles),
  the slippy-map math (`lon/lat ↔ global pixel`), and the view-centre / follow /
  zoom state. Subscribes to `gps.lat`/`gps.lon` and `s.maps.*`; on any change it
  fetches the visible tiles (or overzoom ancestors) from SD into the cache and
  hands a composite request to the LCD task.
- **LCD task** — owns the LVGL canvas, the control buttons, the drag/pinch
  handlers and the zoom pill. `composite()` blits cached tiles (scaled for
  overzoom) into the canvas, draws the marker, toggles the status label.

Why split: SD tile reads are slow (fs worker); doing them on the LCD task would
stutter the UI. The cache is shared under `s_mux`; control flags (pan delta,
recentre, zoom-step) and waypoint edits pass lcd→worker under `s_ctrlMux`; the
waypoint list is shared under `s_wpMux` (see Waypoints).

## On-device controls

- **Follow / pan** — follows the GPS fix; a drag free-pans and unlocks follow;
  the bottom-right "centre me" button re-locks. Two input paths feed the same pan
  accumulator: a finger/trackball-press drag on the canvas (`mapPressCb` on
  `LV_EVENT_PRESSING`, using the indev movement vector), and a trackball edge-pan
  (`lcdProgramScrollHandler` → `mapsScrollCb`) where the lcd component hands over a
  delta when the cursor is driven into a screen edge — so a touchless deck pans
  too, even though the canvas isn't an LVGL scroll container.
- **Zoom** — pinch (two-finger; live-scales the canvas for feedback, then steps
  the zoom by the nearest power-of-two of the pinch ratio on release) **and**
  grey `+`/`−` buttons bottom-left. The buttons post a *relative* step to the
  worker (`s_zoomReq`); pinch instead writes the new *absolute* level straight to
  `s.maps.zoom`, which the worker observes via its `s.maps` subscription. Either
  way the worker re-clamps the result to where real tiles exist (below).
- **Zoom capped to available data** — the worker uses the level nearest the
  desired one where a real (non-overzoom) tile exists at the view centre
  (`realTileAt`, a cache hit or an `fs_stat`), searching at and below the
  desired level first. So you can't zoom in past where the SD actually has
  detail; if you pan out of a high-zoom area's coverage it **auto-zooms-out**
  until a real tile reappears; and if nothing at or below has a tile, the
  coarsest level above that does is used, so zooming out stops at the edge of
  the baked range. Outside the baked area altogether the desired level stands,
  so `+`/`−` keep working while you pan back; that every-level miss is
  remembered for the view's tile at the desired level, so panning inside it
  doesn't re-stat the whole zoom range on each step. The effective level is
  persisted to `s.maps.zoom`.
- **Zoom pill** — a rounded `z<NN>` overlay (bottom-left, above the buttons)
  flashes for 2 s on any effective-zoom change (button, pinch, or auto-zoom-out),
  hidden by a one-shot LVGL timer.
- **Overzoom fallback** — a tile missing at the display zoom is filled by
  upscaling the nearest existing lower-zoom ancestor (searched up to 6 levels up,
  `MAPS_MAXK`), so a coarse base "shows through" instead of going grey (also how
  the bake's edge-dropped tiles render).

## Waypoints

```
  dialog / list / CLI ──WpReq──▶ queue (s_ctrlMux, 8 deep) ──▶ worker
                                                                 │ wpApply under s_wpMux
                                                                 │ wpSerialise under s_wpMux
                                                                 ▼ wpWrite, lock released
                                               <parent of tiledir>/waypoints.json
```

- **One writer.** The list (`s_wp`, PSRAM, `WP_MAX` = 200) lives under
  `s_wpMux`; the LCD task (list panel) and the CLI (`maps wp`) only read it,
  holding the lock just long enough to copy. Every change and every file write
  is the worker's, applied from a small request queue during `rebuild()`, after
  the view is known. Edits wait in the queue while a frame can't get that far
  (no SD, no fix). The SD write happens after the lock is released, so the LCD
  task never waits on the card.
- **Requests name what the requester saw.** The save dialog fixes the point
  (view centre) and the pin under the crosshair (name + coordinates) when it
  opens; rename and delete then act on that pin by name and position
  (`wpFind`), and a new pin goes where the dialog said, whatever the map has
  done since. The list deletes the same way. The CLI is the exception: `add`
  saves at the worker's view centre when it handles the request, and `del`
  takes a name, case-insensitively (`wpFindName`).
- **File.** `wpLoad` parses with cJSON and takes `name`/`lat`/`lon`, skipping
  entries without both coordinates and ignoring other fields. `wpWrite` writes
  `.tmp`, syncs, removes the old file and renames (FATFS rename won't replace);
  a pull-out between remove and rename leaves only a complete `.tmp`, which
  `wpLoad` takes and renames into place when the file itself is missing (a
  `.tmp` that doesn't parse is an unfinished write and is removed). A file that
  is there but doesn't parse, or holds more than `WP_MAX`, sets `s_wpBroken`:
  edits are refused rather than overwrite what the user may still want out of it.
- **Drawing.** The worker hands the composite the `WP_VIS` pins nearest the
  centre (screen positions + names) and the one within `WP_NEAR_PX` of the
  crosshair; the LCD task draws the pins into the canvas and positions a pool of
  `WP_LABELS` name labels on the nearest.

## GPS as ephemeral storage

The map reads ephemeral `gps.*` keys (no `s.` prefix; lost on reboot, synced
live). Today they're written by hw-lilygo-tdeck's `gps.cpp`; any future "GPS service"
abstraction just needs to write the same keys.

- `gps.lat` / `gps.lon` — WGS84 degrees (the fix the map centres on)
- `gps.sats_view` / `gps.snr` — shown on the acquisition screen before a fix

## Cache policy

LRU, default 16 tiles. Sized for the visible grid + overzoom ancestors + a pan
margin. JPEG decode failures are *not* cached (so a transient bad tile is
retried), and the decoder logs a throttled warning rather than silently looking
like "no tiles here".

## The bake pipeline (`tilebake`)

Tiles are produced on a workstation, never on the device. `tilebake` runs the
whole chain in one Docker image (built from `tilebake-image/`, tagged with a
content hash of its inputs so an edit auto-rebuilds it). It has **two modes** —
each map is rendered to vectors on its own (`MODE=render`), then all the maps are
**combined in one render pass** and baked (`MODE=combine`):

```
per map:   .osm.pbf ─(osmium polygon clip)→ planetiler ─→ OpenMapTiles vector .mbtiles
combined:  all .mbtiles ─multi-source style→ tileserver-gl (GL render, Xvfb) ─→ raster PNGs
                                            ─maketiles.py→ /<z>/<x>/<y>.{jpg,bin} device tree
```

The combine step is the whole point: merging maps as **vectors** (not stacked
rasters) is what lets a tile straddling two detailed maps carry both their data,
and lets a coarser map show through across a border as crisp *overzoomed* vectors
rather than a blurry raster upscale. See [§ Combine](#combine--one-render-over-all-maps).

- **planetiler** turns each pbf into an OMT-schema vector `.mbtiles`. It
  auto-downloads Natural Earth + water polygons once (~1 GB, cached under
  `.tilebake-cache/data`, then offline). `--bounds` = the map's bbox.
  - `--maxzoom` is the vector ceiling planetiler stores, **hard-capped at 16**
    (planetiler errors above that). A folder ceiling above z16 is still honoured
    for the *device* tiles: only what planetiler stores is clamped to 16, and the
    combine bake overzooms those z16 vectors up to the requested ceiling
    (tileserver-gl serves above a vector source's maxzoom natively), so the device
    tree still reaches the requested zoom — the same overzoom mechanism as the
    cross-border show-through. Caveat from the data, not the tool:
    planetiler/OpenMapTiles cap useful vector zoom around **z15** and OSM-via-OMT
    carries no real map detail above **~z14**, so a ceiling past there only yields
    *larger* tiles, not *more* detail (the deepest meaningful zoom is the data's,
    ~z14).
  - We deliberately set **no `--minzoom`** (always render from z0). A vector
    source with a *non-zero* minzoom renders **blank** at exactly that bottom
    level in tileserver-gl/MapLibre (no parent tile to compose it from), which
    would clobber the layer beneath. Rendering from z0 keeps every baked zoom
    above the source floor.
- **osmium polygon clip** (render mode, only for a coarse map with deeper maps
  over it) drops the source's data wherever a deeper map covers, *before*
  planetiler — so the coarse vector tiles are born without that data and can't
  draw it under the detailed map. The clip is an osmosis `.poly` whose outer ring
  is the map's own footprint and whose **holes** are the deeper maps' footprints,
  built host-side by `mkclip.py`. Each footprint is the map's sibling `.poly`
  (exact) or its bbox rectangle (fallback). osmium honours the hole rings.
- **tileserver-gl** GL-renders the vector tiles to raster PNGs on loopback. In
  combine mode it serves **every** map's `.mbtiles` at once under a generated
  multi-source style (see below), reusing the image's bundled `basic-preview`
  layer set + Noto fonts. Needs an X display, so the entrypoint runs Xvfb.
- **maketiles.py** fetches the raster tiles for the zoom range over the bbox and
  writes the device tree. Also usable standalone against any `{z}/{x}/{y}` URL or
  rendered tree (it refuses `tile.openstreetmap.org` per the tile policy).
- **bbox** is read straight from each pbf's header (`pbfbbox.py`, a ~30-line
  protobuf parse — no osmium, instant on multi-GB files). osmium's full scan is
  only an in-container fallback for extracts lacking a header bbox.

### Per-map vector cache + incremental

Each map renders once to its own `.tilebake-cache/render/<dir>/tiles.mbtiles`,
stamped with a signature (`source size+mtime | vector ceiling | clip-poly hash |
image hash`). A re-run skips a map whose signature still matches, so changing one
extract re-renders only that one; changing the set of *deeper* maps changes a
coarse map's clip-poly hash and re-renders just it; the image-hash term means a
pipeline change invalidates every cache — pass `--keep-renders` to override that
when you know an `entrypoint.sh`/image edit can't change render output (it ignores
the image-hash term and also adopts a completed-but-unstamped render a crash left
behind, so only genuinely changed maps re-render). Format/quality are **not** in the
signature — they apply later, at combine, so a quality tweak re-bakes without
re-rendering any vectors.

### Combine — one render over all maps

The combine pass (`MODE=combine`) is where the maps actually merge. `tilebake`
writes a manifest of the caches, coarsest-ceiling first, and the container:

1. **Builds a multi-source style** (`style_compose.py`) — it takes the bundled
   single-source OMT style and clones its data-layer stack **once per source**,
   coarsest→deepest, each clone bound to that source. So a z13 tile renders the
   world fill (overzoomed from z7), then Europe (overzoomed from z9), then Germany
   (native z13), then Berlin… each drawing on top of the last.
2. **Serves all the `.mbtiles` together** in one tileserver-gl, then **bakes** the
   composite with `maketiles.py` over each map's bbox and zoom band.

Two properties fall out of merging as vectors rather than rasters:

- **Detail wins by data presence, not by overwrite.** Because every coarse map was
  polygon-clipped to exclude deeper footprints, there's no coarse data inside a
  detailed map to draw — the stack is a clean partition. A tile shared by two
  detailed maps (Netherlands ∪ Germany) gets *both* their data in the one render.
- **The background across a border is crisp.** Outside a detailed map the next
  coarser source shows through, and being a vector source MapLibre **overzooms** it
  (renders its z9 geometry at z13) — sharp lines, not the blurry, re-compressed
  raster upscale a per-tile raster composite was stuck with.

The world fill (empty folder) is never clipped: it's the universal land/water base
that every deeper map draws on top of. Each source bakes over `[zmin..ceiling]` of
its own bbox; low zooms are re-fetched by several sources (identical bytes, cheap),
deep zooms only by the maps that reach them, so there's no giant-union blow-up.

`--out` is written in-container (the raster render must run where tileserver-gl
is), so it must be a path Docker can bind-mount — bake to a normal dir, then copy
onto the SD card. Before any of this, `tilebake` prints a rough per-map + total
size estimate (tile count × a calibrated per-tile mean) and waits for confirmation.

## Why this straddle isn't in the RNS family

It has nothing to do with Reticulum — the first **non-RNS feature straddle** on
the platform, an end-to-end example of an LCD program built on spangap-core +
spangap-lcd and nothing else. Published under `reticulous/` (GitHub org) for
discoverability; the dep graph is RNS-free.
