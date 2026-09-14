#!/usr/bin/env bash
# In-container orchestrator for tilebake. Two modes, selected by $MODE:
#
#   MODE=render   one map  pbf -> (optional polygon clip) -> OpenMapTiles vector
#                 .mbtiles. Just the vector build; no raster, no device tiles.
#                 The .mbtiles is the durable, cacheable per-map product.
#
#   MODE=combine  many maps' vector .mbtiles -> ONE seamless raster bake. All the
#                 maps are served together by a single tileserver-gl, under a
#                 generated style that stacks the OpenMapTiles layer set once per
#                 source (coarsest->deepest). maketiles then fetches the composite
#                 over each map's bbox into the /<z>/<x>/<y>.{jpg,bin} device tree.
#
# Why the split: each map renders to vectors independently (so an unchanged map is
# skipped on re-run), but the *combining* must happen in the vector domain, in one
# render pass — that's the only way a tile straddling two detailed maps gets BOTH
# their data, and the only way the coarser map shows through across a border as
# CRISP overzoomed vectors instead of a blurry raster upscale. Per-map polygon
# clipping (done in render mode, below) is what keeps the coarse map's few-point
# roads/rivers from drawing *alongside* the detailed map inside its area: the
# coarse data is physically removed where a deeper map covers, so stacking the
# sources is a clean partition, not an overdraw.
#
# Render-mode env:
#   IN_PBF    source .osm.pbf                          (required unless WORLD=1)
#   OUT       dir to write tiles.mbtiles into          (required, mounted rw)
#   ZMAX      vector ceiling (always rendered from z0)  (required)
#   BBOX      "MINLON MINLAT MAXLON MAXLAT"            (default: read from the pbf)
#   CLIP_POLY osmosis .poly to clip the source to      (optional; holes honoured)
#   WORLD     1 -> global Natural-Earth fill, no pbf/clip
#   JAVA_MEM  e.g. 4g -> planetiler -Xmx4g             (default: planetiler decides)
#
# Combine-mode env:
#   MANIFEST  JSON listing the sources, coarsest-first (required, see below)
#   OUT       device tile tree to (re)build            (required, mounted rw)
#   FMT       jpg | bin                                (default jpg)
#   QUALITY   JPEG quality                             (default 80)
#   STYLE     force a tileserver style id              (default: auto-discover)
#
# planetiler caches its global source data (water polygons, natural earth) under
# /cache so re-runs are offline; that dir is meant to be a mounted volume.
set -euo pipefail

MODE="${MODE:-render}"
PORT=8080
CACHE=/cache
RUN=/tmp/run            # ephemeral: style bundle + symlinks to the source mbtiles

VERBOSE="${VERBOSE:-0}"
log()  { printf '\n\033[1;36m== %s\033[0m\n' "$*" >&2; }
vlog() { [ "$VERBOSE" = 1 ] && printf '   %s\n' "$*" >&2 || true; }

# ---------------------------------------------------------------------------
# MODE=render — one pbf -> one vector .mbtiles, optionally polygon-clipped first.
# ---------------------------------------------------------------------------
do_render() {
    : "${OUT:?set OUT}"; : "${ZMAX:?set ZMAX}"
    mkdir -p "$CACHE" "$OUT"
    local src
    # planetiler's pass-1 node map and the clipped-pbf copy can run to tens of GB
    # for a country extract. They MUST land on the bind-mounted host cache, not the
    # container's /tmp — on Docker Desktop /tmp is the VM's small disk image and
    # fills mid-render ("No space left on device"). A per-run subdir under /cache
    # keeps concurrent renders from colliding; trap-cleaned on exit.
    # NB: WORK is intentionally NOT 'local' — the EXIT trap runs after this
    # function returns, in global scope, so a local would be out of scope there
    # (and unbound under set -u, aborting the run).
    WORK=$(mktemp -d "$CACHE/.work.XXXXXX")
    trap 'rm -rf "${WORK:-}"' EXIT

    if [ "${WORLD:-0}" = 1 ]; then
        # World fill: no source pbf — render the globe from the cached Natural
        # Earth / water-polygon data (coherent to ~z7). Feed planetiler the
        # bundled empty pbf (it insists on an OSM input) and force global bounds.
        src=/opt/empty.osm.pbf
        BBOX="${BBOX:--180 -85 180 85}"
    else
        : "${IN_PBF:?set IN_PBF, or WORLD=1 for a global low-zoom bake}"
        src="$IN_PBF"
        # bbox: the one passed, else the pbf's own header extent (pbfbbox.py — a
        # direct header parse, instant on multi-GB files). osmium's full scan is
        # only the fallback for an extract carrying no header bbox.
        if [ -z "${BBOX:-}" ]; then
            BBOX=$(python3 /opt/pbfbbox.py "$IN_PBF" || true)
            if [ -z "$BBOX" ]; then
                local raw; raw=$(osmium fileinfo -e -g data.bbox "$IN_PBF")
                BBOX=$(printf '%s' "$raw" | tr -cd '0-9.,-' | tr ',' ' ')
            fi
            vlog "bbox from pbf header: $BBOX"
        fi
        # Polygon clip: physically drop the source's data wherever a deeper map
        # covers (the clip .poly's holes are the deeper maps' boundaries), so the
        # coarse vectors never co-draw inside a detailed map. osmium honours the
        # osmosis hole rings. Done before planetiler so the vector tiles are born
        # with the holes — no post-hoc tile surgery.
        if [ -n "${CLIP_POLY:-}" ] && [ -f "$CLIP_POLY" ]; then
            log "clipping source to $CLIP_POLY"
            # osmium has no --quiet; verbosity is -v and the progress meter is
            # --no-progress. Default to the quiet path (suppress the meter), only
            # going verbose when asked.
            osmium extract --overwrite --strategy complete_ways \
                -p "$CLIP_POLY" "$IN_PBF" -o "$WORK/clipped.osm.pbf" \
                $([ "$VERBOSE" = 1 ] && echo --verbose || echo --no-progress)
            src="$WORK/clipped.osm.pbf"
        fi
    fi

    # Take exactly the first four bbox fields, so a stray token can't shift them.
    set -- $BBOX
    local MINLON=${1:-} MINLAT=${2:-} MAXLON=${3:-} MAXLAT=${4:-}
    [ -n "$MAXLAT" ] || { echo "could not determine a bbox; pass --bbox explicitly" >&2; exit 1; }

    # pbf -> OpenMapTiles vector .mbtiles, rendered natively to the requested
    # ceiling (--maxzoom). We deliberately set NO --minzoom: a vector source with
    # a non-zero minzoom renders BLANK at exactly that bottom level in MapLibre
    # (no parent to compose from), so starting at z0 keeps every baked zoom above
    # the source floor. planetiler/OMT cap vector zoom (~z15) and OSM carries no
    # real detail above ~z14, so a higher ceiling may error or only yield larger
    # tiles, not more detail.
    #
    # planetiler HARD-caps the stored maxzoom at 16 (it errors above that). A
    # higher folder ceiling is still honoured for the DEVICE tiles: clamp only what
    # planetiler stores, then let the combine bake overzoom those z16 vectors up to
    # $ZMAX (tileserver-gl serves above a vector source's maxzoom natively), same as
    # the cross-border overzoom. So the device tree still reaches $ZMAX.
    local PZMAX="$ZMAX"
    if [ "$PZMAX" -gt 16 ]; then
        PZMAX=16
        vlog "planetiler maxzoom clamped $ZMAX -> 16 (tool hard cap); device tiles still bake to z$ZMAX via overzoom"
    fi
    log "building vector tiles  z0-$PZMAX  ->  $OUT/tiles.mbtiles"
    vlog "planetiler: $src -> $OUT/tiles.mbtiles  bounds $MINLON,$MINLAT,$MAXLON,$MAXLAT"
    cd "$CACHE"   # planetiler caches downloaded sources under ./data here
    local pt=( java ${JAVA_MEM:+-Xmx"$JAVA_MEM"} -jar /opt/planetiler.jar
               --osm-path="$src" --output="$OUT/tiles.mbtiles"
               --bounds="$MINLON,$MINLAT,$MAXLON,$MAXLAT"
               --maxzoom="$PZMAX" --tmpdir="$WORK/planetiler-tmp" --download --force )
    if [ "$VERBOSE" = 1 ]; then
        "${pt[@]}"
    else
        set +e
        "${pt[@]}" 2>&1 | grep -F ' ERR ' >&2
        local rc=${PIPESTATUS[0]}; set -e
        [ "$rc" -eq 0 ] || { echo "planetiler failed — re-run tilebake with -v for the full log" >&2; exit 1; }
    fi
    log "done -> $OUT/tiles.mbtiles"
}

# ---------------------------------------------------------------------------
# MODE=combine — all maps' vector .mbtiles -> one seamless device tile tree.
# ---------------------------------------------------------------------------
do_combine() {
    : "${MANIFEST:?set MANIFEST}"; : "${OUT:?set OUT}"
    local FMT="${FMT:-jpg}" QUALITY="${QUALITY:-80}"
    [ -f "$MANIFEST" ] || { echo "manifest not found: $MANIFEST" >&2; exit 1; }
    mkdir -p "$RUN" "$OUT"

    # Lay the style bundle (style + fonts) into $RUN and symlink every source's
    # mbtiles in beside it as s<index>.mbtiles — the tileserver 'data' entries, the
    # style's mbtiles://{s<index>} source urls and these symlinks all key off the
    # source's POSITION in the manifest (coarsest-first), so the keys are always
    # valid ids no matter what the maps' folder names are.
    cp -a /opt/tsdata/. "$RUN/"
    python3 - "$MANIFEST" "$RUN" <<'PY'
import json, os, sys
manifest, run = sys.argv[1], sys.argv[2]
for i, s in enumerate(json.load(open(manifest))["sources"]):
    link = os.path.join(run, "s%d.mbtiles" % i)
    if os.path.lexists(link):
        os.remove(link)
    os.symlink(s["mbtiles"], link)
PY

    local STYLE_REL
    STYLE_REL=$(cd "$RUN/styles" && find . -maxdepth 3 -name style.json | sed 's#^\./##' | head -n1)
    [ -n "$STYLE_REL" ] || { echo "no bundled style.json found under $RUN/styles" >&2; exit 1; }
    local STYLE_ID="${STYLE:-$(dirname "$STYLE_REL")}"

    # Rewrite the bundled style IN PLACE into a multi-source composite: its OMT
    # layer set is cloned once per source (coarsest->deepest, so a deeper map
    # draws on top in the rare residual overlap), each clone bound to its source.
    # Editing in place preserves glyphs/sprite/version and keeps every relative
    # asset path valid.
    python3 /opt/style_compose.py "$MANIFEST" "$RUN/styles/$STYLE_REL"

    # tileserver config: one data entry per source (mbtiles resolved under root),
    # the composite style.
    python3 - "$MANIFEST" "$RUN" "$STYLE_ID" "$STYLE_REL" > "$RUN/config.json" <<'PY'
import json, sys
manifest, run, style_id, style_rel = sys.argv[1:5]
srcs = json.load(open(manifest))["sources"]
cfg = {
    "options": {"paths": {"root": run, "fonts": "fonts", "styles": "styles", "mbtiles": ""}},
    "styles": {style_id: {"style": style_rel}},
    "data":   {"s%d" % i: {"mbtiles": "s%d.mbtiles" % i} for i in range(len(srcs))},
}
print(json.dumps(cfg, indent=2))
PY

    # Serve the composite on loopback. GL render needs an X display.
    vlog "starting tileserver-gl on 127.0.0.1:$PORT"
    cd "$RUN"
    rm -f /tmp/.X99-lock
    export DISPLAY=:99
    Xvfb "$DISPLAY" -nolisten unix >/tmp/xvfb.log 2>&1 &
    node /usr/src/app --config "$RUN/config.json" -p "$PORT" -b 127.0.0.1 \
        >/tmp/tileserver.log 2>&1 &
    SERVER_PID=$!
    trap 'kill "$SERVER_PID" 2>/dev/null || true' EXIT

    local TILE_URL="http://127.0.0.1:$PORT/styles/$STYLE_ID/{z}/{x}/{y}.png"
    local PROBE="http://127.0.0.1:$PORT/styles/$STYLE_ID/0/0/0.png"
    local i
    for i in $(seq 1 120); do
        if curl -sf -o /dev/null "$PROBE"; then break; fi
        if ! kill -0 "$SERVER_PID" 2>/dev/null; then
            echo "tileserver-gl exited early; log:" >&2; cat /tmp/tileserver.log >&2; exit 1
        fi
        [ "$i" = 120 ] && { echo "tileserver-gl not ready after 120s; log:" >&2; \
            cat /tmp/tileserver.log >&2; exit 1; }
        sleep 1
    done
    vlog "serving $TILE_URL"

    # Rebuild OUT fresh — it's a derived artifact.
    find "$OUT" -mindepth 1 -delete 2>/dev/null || true

    # Bake each source's region over its zoom band. Every fetched tile is the full
    # composite (all sources, coarse overzoomed under the deep ones), so which
    # source 'drives' the fetch only decides WHICH tiles get written, not their
    # content. Low zooms are re-fetched by several sources (identical bytes, cheap
    # at z<=7); deep zooms are fetched only by the maps that reach them, over their
    # own bbox, so there's no giant-union blow-up.
    local FMT_FLAG=--jpg; [ "$FMT" = bin ] && FMT_FLAG=--bin
    log "rasterizing + tiling the composite  ->  $OUT"
    local n; n=$(python3 -c 'import json,sys; print(len(json.load(open(sys.argv[1]))["sources"]))' "$MANIFEST")
    local k
    for k in $(seq 0 $((n - 1))); do
        # id  zmin  zmax  bbox   for source k
        local fields; fields=$(python3 - "$MANIFEST" "$k" <<'PY'
import json, sys
s = json.load(open(sys.argv[1]))["sources"][int(sys.argv[2])]
print("%s\t%d\t%d\t%s" % (s["id"], s["zmin"], s["maxzoom"], s["bbox"]))
PY
)
        local id zmin zmax bbox
        IFS=$'\t' read -r id zmin zmax bbox <<<"$fields"
        printf '\n\033[1;32m>>> baking %s  z%s-%s\033[0m\n' "$id" "$zmin" "$zmax" >&2
        local mt=( python3 /opt/maketiles.py --src "$TILE_URL" --out "$OUT"
                   --bbox $bbox --zoom "$zmin" "$zmax" $FMT_FLAG --quality "$QUALITY" --delay 0 )
        if [ "$VERBOSE" = 1 ]; then
            "${mt[@]}"
        else
            "${mt[@]}" >/dev/null || { echo "tiling failed — re-run tilebake with -v for the full log" >&2; exit 1; }
        fi
    done
    log "done -> $OUT  (copy its contents to the SD card under /sdcard/maps)"
}

case "$MODE" in
    render)  do_render ;;
    combine) do_combine ;;
    *) echo "unknown MODE='$MODE' (want render|combine)" >&2; exit 1 ;;
esac
