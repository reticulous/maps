# maps

## What is this?

**maps** is an offline slippy-map viewer for spangap LCD devices: an LVGL
launcher app that blits pre-baked map tiles (JPEG, with a raw RGB565 fallback)
read from SD, centred on the live GPS fix, with drag-to-pan and pinch/button
zoom. Nothing is rendered or fetched on the device — tiles are baked on a
workstation (see [Tile baking](#tile-baking)) and copied to the SD card.
Independent of the RNS family — published under `reticulous/` for
discoverability, but any LCD buildable straddle can consume it. Doubles as the
docs' "first non-RNS feature straddle" walkthrough.

## Origins

The desktop-renders / device-only-blits split follows
[`esp32_offline_osm`](https://github.com/mryndzionek/esp32_offline_osm): the
heavy vector→raster rendering and the OSM dataset stay on a computer; the MCU
only does file-read, JPEG-decode, and blit. The on-SD raw format differs (this
viewer stores tiles in LVGL's native little-endian RGB565, not that project's
direct-to-panel byte order) — see [INTERNALS.md](INTERNALS.md).

## What this straddle owns

```
maps/
├── esp-idf/
│   ├── include/maps.h                public API: mapsInit (a no-op)
│   ├── src/maps.cpp                  the straddle init hook — mapsInit() does nothing
│   └── conditional/spangap-lcd/
│       └── src/maps_lcd.cpp          the entire viewer: render worker task, LVGL
│                                     compositor, CLI verb, and the LcdApp launcher.
│                                     Compiled only when the lcd straddle is staged.
└── tilebake/
    ├── tilebake                      one-container pbf -> device tiles (wraps the image below)
    └── tilebake-image/               the container build context
        ├── Dockerfile
        ├── entrypoint.sh             in-container orchestrator: render (clip->planetiler) | combine (tileserver->maketiles)
        ├── maketiles.py              the tiler: bake rendered rasters into the device SD tree (also standalone)
        ├── pbfbbox.py                read a pbf's header bounding box (no osmium)
        ├── style_compose.py          build the multi-source style that stacks all maps in one render
        └── mkclip.py                 build a map's clip polygon (own footprint minus deeper maps; host-side)
```

This is an on-device viewer only — there is no browser UI.

## How others use it

Include `reticulous/maps` in an LCD build and a **Maps** tile appears in the
launcher. There is nothing to call: the viewer is a single `services:` entry
(`MapsApp`, an `LcdApp`), so the build's generated boot registration constructs
it, installs its launcher tile, and runs its boot wiring (render worker, CLI
verb, cache mutex) in `MapsApp::appInit()`. The entry is `when:`-gated on
`spangap-lcd`, so the whole viewer compiles only when the shell is staged; in a
non-LCD build there is no map.

The map reads its tiles from SD at the default path
`/sdcard/maps/<z>/<x>/<y>.jpg` (or `.bin`), 256×256, read on demand and cached
in PSRAM by the render worker. The GPS fix arrives through ephemeral `gps.*`
storage keys, so there is no compile-time GPS dependency — the GNSS chip lives
in the consuming buildable straddle (today hw-lilygo-tdeck's `gps.cpp`) until a GPS
service abstraction earns its own straddle.

**Controls:** follows the GPS fix; drag (touch or trackball) to free-pan, the
bottom-right button re-centres. Zoom with pinch or the bottom-left `+`/`−`
buttons — zoom is capped to where the SD actually has tiles (it won't zoom into
blur, and auto-zooms-out if you leave a detailed area's coverage), and a `z<NN>`
pill flashes for 2 s on each change. See [INTERNALS.md](INTERNALS.md).

## Waypoints

Pan until the crosshair sits on a spot (or stay on the GPS fix), press the
save button (the floppy above the re-centre button), type a name, press Enter.
An empty name becomes the lowest free `WP <n>`. The spot is
saved to `waypoints.json` in the **parent of the tile dir** — with the default
`/sdcard/maps` that is `/sdcard/waypoints.json`, right next to `maps/`:

```json
{"waypoints":[
  {"name":"Camp","lat":52.5200000,"lon":13.4050000}
]}
```

Waypoints draw as amber pins with a name label, distinct from the red GPS dot.
With the crosshair on an existing pin the dialog is prefilled: Enter renames it,
**Delete** removes it.
The list button (above the save button) shows every saved waypoint with its
coordinates; the trash button on a row deletes it after a second press
("Sure?"). Esc closes the list. The file is plain JSON, so you can also edit it
on a computer: other fields are ignored, as are entries without both `lat` and
`lon`, and a rewrite keeps only `name`, `lat` and `lon`. A file the device can't
read as a waypoint list shows no pins, one holding more than the cap shows the
first 200, and neither is ever overwritten: edits are refused with "Fix
waypoints.json first" until the file is repaired or removed. Cap: 200 waypoints, 24-character names.

## Storage variables

Settings live under `s.maps.*` and are owned by the straddle.yaml `settings:`
block, which generates both the Settings → Apps → Maps pane (LCD + web) and the storage
defaults. Runtime state is published under `maps.*`; the GPS fix is read from
ephemeral `gps.*`.

### Settings

| Key | Default | Meaning |
|---|---|---|
| `s.maps.zoom` | `15` | Slippy zoom level, clamped 1–19. Written back when the effective zoom is capped down to the tiles actually present. |
| `s.maps.tiledir` | `/sdcard/maps` | SD path holding the `<z>/<x>/<y>.{jpg,bin}` tile tree. |

### Runtime state (written)

| Key | Values |
|---|---|
| `maps.state` | `ready` / `no tiles` / `no fix` / `no sd` — ephemeral, shown read-only in the Settings pane. |

### GPS fix (read, ephemeral)

| Key | Meaning |
|---|---|
| `gps.lat` / `gps.lon` | WGS84 degrees — the fix the map centres on. |
| `gps.sats_view` / `gps.snr` | Satellites in view / best signal, shown on the acquisition screen before a fix. |

While the app is open it sets ephemeral `lcd.multi_touch` to `1` (and back to
`0` on close) so the board reports both fingers for pinch.

## CLI

```
maps           map status — state, sd, zoom, follow/pan, tiledir, GPS & view centres
maps center    recentre on the GPS fix
maps wp        list waypoints; `maps wp add <name>` saves at the view centre, `maps wp del <name>` removes
```

Run on-device through `spangap cli "<command>"`.

## Dependencies

- [spangap-core](../spangap-core) — fs (SD reads), storage (`s.maps.*`, `gps.*`).
- [spangap-lcd](../spangap-lcd) — the LVGL launcher and `LcdApp` lifecycle the viewer hooks into.

## Tile baking

Tiles are baked on a workstation, not the device. There are two entry
points, depending on what you already have:

**You have rendered raster tiles (a `z/x/y` tree or your own tile
server URL):** use `tilebake/tilebake-image/maketiles.py` directly. It takes a bbox and
zoom range and produces the `/<z>/<x>/<y>.{jpg,bin}` tree the device
reads. It is *not* a renderer — it will not turn an `.osm.pbf` into
tiles, and it refuses to scrape `tile.openstreetmap.org`.

**You only have raw OSM data (an `.osm.pbf` extract, e.g. from
Geofabrik):** use `tilebake/tilebake`. It builds and runs a single
container that does the whole chain — renders the pbf to vector tiles
(planetiler), rasterises them through an internal tileserver-gl, and
bakes the device tree via `maketiles.py` — so all you supply is the
pbf, a bbox and a zoom range:

```
./tilebake/tilebake --out ./tiles --pbf berlin-latest.osm.pbf \
    --bbox 13.10 52.35 13.75 52.60 --zoom 10 16
```

`--out` is required (created if absent — there is no implicit output
dir). Before writing anything, tilebake prints a rough per-map and
total disk estimate and waits for a `y`; pass `--yes` to skip.

**Batch / layered maps:** drop the `--bbox`/`--zoom` and point it at a
`./pbf/<ceiling>/` tree — one folder per map, named by its top zoom
(e.g. `7/` empty for a world fill, `13/germany.osm.pbf`,
`16/berlin.osm.pbf`). Each map renders once to its own vector cache
(`./.tilebake-cache/render/<dir>/tiles.mbtiles`) and only re-renders when
its source, the pipeline, or its clip changes, so re-runs skip untouched
maps. `--out` is then baked by **combining every cache in one
tileserver-gl render pass**, so the maps merge as *vectors*, not as
stacked rasters. The win over a raster composite: a tile shared by two
detailed maps (Netherlands meeting Germany) gets **both** their data, and
the coarser map shows through across a detailed map's border as **crisp
overzoomed vectors** instead of a blurry upscale. To stop the coarse
map's few-point roads/rivers from drawing *alongside* the detailed ones
inside its area, each coarse map is **polygon-clipped at render time** to
exclude the footprint of every deeper map — using the map's `.poly`
binding polygon (Geofabrik ships one per extract; drop it next to the
`.pbf`) for an exact cut, or its bbox if there's no `.poly`. The world
fill (an empty folder) is never clipped — it's the universal land/water
base. See the script header.

Needs Docker. The first run downloads planetiler's global source data
(water polygons, natural earth — ~1 GB) into `./.tilebake-cache`;
subsequent runs reuse it and are offline. See
[tilebake-image/](tilebake/tilebake-image/) for the image and the
in-container orchestrator.

Copy the resulting tree to your SD card under `maps/` and the device
picks it up.

### Sizing & licensing

You bound storage by bbox + zoom range, not by tiling the planet, so a city
across the zooms you actually use is comfortable on an SD card. As a rule of
thumb a city to z16 runs a few hundred MB of JPEG; each further zoom level is
roughly ×4 the tiles and disk (z18 ≈ 16× a z16 set), so **z16 is a sane cap for
a handheld**. Raw `.bin` trades ~7–13× the size for zero on-device decode.

OSM data is **ODbL**: redistributing the baked tiles (or a derived street
index) carries the "© OpenStreetMap contributors" attribution and the
share-alike terms. The tiles are free to use — that's just the obligation that
rides along.

## Read next

- [INTERNALS.md](INTERNALS.md) — tile format, worker/lcd split,
  on-device controls + zoom capping, cache policy, and the `tilebake`
  bake pipeline (per-map polygon-clipped vector render → one multi-source
  tileserver-gl pass → maketiles, with the per-map vector cache).
