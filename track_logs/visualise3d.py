"""
An offline visualiser for a .raemot3d file.
Reads the binary file (header + 64-byte records, as documented in the README and raemot_file.hpp),
reconstructs each global track's 3D trajectory, and renders them in a single 3D plot, one colour
per global_id.

Usage:
    python3 visualise3d.py <file_name>
    python3 visualise3d.py <file_name> --colour-by speed --arrows
    python3 visualise3d.py <file_name> --save <image_name> --no-show
    python3 visualise3d.py <file_name> --elev 25 --azim -60
"""
import argparse
import sys
import bisect
from pathlib import Path

import numpy as np
import random
import struct
from datetime import datetime
from zoneinfo import ZoneInfo
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import hsv_to_rgb
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d.art3d import Line3DCollection

MAGIC = b"RAEMOT3D"
HEADER_SIZE = 64
REC_POINT, REC_OPEN, REC_CLOSE = 1, 2, 3

# set your time zone info here
CANBERRA = ZoneInfo("Australia/Canberra")

# data type for one 64-byte record.
# The payload (48 bytes) is generically read as 6 doubles (POINT uses all 6 as
# x, y, z, vx, vy, vz; OPEN's first 8 bytes are reinterpreted as two int32
# camera track ids via .view(); CLOSE's payload is unused since its empty)
RECORD_DTYPE = np.dtype([
    ("type", "u1"),
    ("pad", "V3"),
    ("id", "<u4"),
    ("tg", "<f8"),
    ("payload", "<f8", 6),
])

# one track class, used to hold all the information for a track
class Track:
    __slots__ = ("global_id", "a_id", "b_id", "tg", "xyz", "vxyz", "closed")

    def __init__(self, global_id, tg, xyz, vxyz):
        self.global_id = global_id
        self.a_id = None
        self.b_id = None
        self.tg = tg
        self.xyz = xyz
        self.vxyz = vxyz
        self.closed = False

## -------------------------------------------------------------------- ##
## File accessors
## -------------------------------------------------------------------- ##

def open_records(path: Path) -> np.memmap:
    """
    Parses a .raemot3d file to validate the header and returns a simple read-only memmap of the included records, as well as the time the file was created (pulled from the header)
    """
    with open(path, "rb") as f:
        header = f.read(HEADER_SIZE)
        if len(header) < HEADER_SIZE or header[:8] != MAGIC:
            raise ValueError(f"{path} is not a .raemot3d file (bad or missing magic bytes in header - should be 'RAEMOT3D')")

    # extract the file creation timestamp
    created_unix_ns = struct.unpack_from("<Q", header, 16)[0]

    rec_size = RECORD_DTYPE.itemsize
    contents = path.stat().st_size - HEADER_SIZE
    n_rec = contents // rec_size
    if n_rec == 0:
        raise ValueError(f"{path} file has a valid header but no records? Where are they at huh?")
    if contents % rec_size:
        print(f"Warning: {contents % rec_size} trailing bytes ignored. Perhaps the file is correupted?", file=sys.stderr)
    mm = np.memmap(path, dtype=RECORD_DTYPE, mode="r", offset=HEADER_SIZE, shape=(n_rec,))
    return mm, created_unix_ns

def find_predecessors(mm, low, ids, cutoff_tg, chunk):
    """
    For each id in 'ids', the latest POINT record before record index 'low' with tg >= cutoff_tg.
    Scans backwards in chunks and stops early once every id is found or the cutoff is passed.
    """
    needed = set(ids)
    rows = []
    end = low
    while end > 0 and needed:
        s = max(0, end - chunk)
        ch = mm[s:end]
        need_arr = np.fromiter(needed, dtype=np.uint32, count=len(needed))
        m = (ch["type"] == REC_POINT) & (ch["tg"] >= cutoff_tg) & np.isin(ch["id"], need_arr)
        cand = ch[m]
        if cand.size:
            cand = cand[np.argsort(cand["id"], kind="stable")]
            u, st, c = np.unique(cand["id"], return_index=True, return_counts=True)
            rows.append(cand[st + c - 1])
            needed.difference_update(u.tolist())
        if ch["tg"][0] < cutoff_tg:
            break
        end = s
    return np.concatenate(rows) if rows else np.empty(0, dtype=RECORD_DTYPE)

def load_window(mm, low, high, window_start, lookback, max_points, chunk):
    """
    Load records [low, high) and returns ({global_id: Track}, n_points_in_window)
    """
    parts = []
    for s in range(low, high, chunk):
        ch = mm[s:min(s + chunk, high)]
        p = ch[ch["type"] == REC_POINT]
        if p.size:
            parts.append(p.copy()) # detach from the memmap
    if not parts:
        return {}, 0

    pts = np.concatenate(parts)
    n_in = int(pts.size)

    pre = find_predecessors(mm, low, np.unique(pts["id"]).tolist(), window_start - lookback, chunk)

    if pre.size:
        pts = np.concatenate([pre, pts])

    pts = pts[np.lexsort((pts["tg"], pts["id"]))]

    tracks = {}
    u, start, c = np.unique(pts["id"], return_index=True, return_counts=True)
    for gid, st, n in zip(u, start, c):
        sub = pts[st:st + n]
        if max_points > 0 and n > max_points:
            # linspace evenly strides, including both first and last point
            sub = sub[np.unique(np.linspace(0, n - 1, max_points).astype(np.int64))]
        tracks[int(gid)] = Track(int(gid), sub["tg"].copy(), sub["payload"][:, 0:3].copy(), sub["payload"][:, 3:6].copy())

    return tracks, n_in

def sample_global(mm, n_samples=50_000):
    """
    Cheap whole-filled sample -> fixed axis limits and speed colour range.
    """
    idx = np.unique(np.linspace(0, len(mm) - 1, min(len(mm), n_samples)).astype(np.int64))
    s = mm[idx]
    pts = s[s["type"] == REC_POINT]
    if pts.size == 0:
        raise ValueError("uhh... no POINT records found in the file. Nothing to plot ._.")
    xyz = pts["payload"][:, 0:3]
    speed = np.linalg.norm(pts["payload"][:, 3:6], axis=1)
    vmin, vmax = np.percentile(speed, [1, 99])
    return equal_limits(xyz), (float(vmin), float(vmax))

## -------------------------------------------------------------------- ##
## Plotting helpers
## -------------------------------------------------------------------- ##

def equal_limits(xyz):
    """
    Returns (centre, radius) of a cube that contains all xyz, so axes are equally scaled and
    trajectories aren't distorted by whichever axis has the largest range.
    """
    low, high = xyz.min(axis=0), xyz.max(axis=0)
    radius = 0.5 * float((high - low).max()) * 1.05
    return (low + high) / 2, (radius if radius > 0 else 1.0)

def colour_for(gid: int):
    """
    Assigns a unique, stable, well-spread colour per global_id using golden-ratio hue spacing,
    so a track keeps its colour in every time window regardless of what else is visible.
    Neighbouring IDs should never have a visually similar colour.
    """
    return tuple(hsv_to_rgb(((gid * 0.6180339887) % 1.0, 0.65, 0.95))) + (1.0,)

def style_axes(ax):
    ax.set_facecolor("#0e0e12")
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_facecolor((0.08, 0.08, 0.11, 1.0))
        axis.pane.set_edgecolor((1, 1, 1, 0.15))
        axis._axinfo["grid"]["color"] = (1, 1, 1, 0.12)
    ax.tick_params(colors="#aaaaaa", labelsize=8)
    ax.set_xlabel("X  (m)")
    ax.set_ylabel("Y  (m)")
    ax.set_zlabel("Z  (m)")
    for label in (ax.xaxis.label, ax.yaxis.label, ax.zaxis.label):
        label.set_color("#cccccc")

def ordinal(n: int) -> str:
    if 11 <= n % 100 <= 13:
        return "th"
    return {1: "st", 2: "nd", 3: "rd"}.get(n % 10, "th")


class Viewer:
    def __init__(self, mm, created_unix_ns, args):
        self.mm, self.args = mm, args
        self.created_s = created_unix_ns / 1e9 # unix seconds at tg = 0 (roughly, anyways)
        self.tg = mm["tg"]
        self.n = len(mm)
        self.t0, self.t1 = float(self.tg[0]), float(self.tg[self.n - 1])
        self.step = args.step
        span = self.t1 - self.t0
        self.n_windows = 1 if span <= self.step else int(np.ceil((span - self.step) / self.step)) + 1
        self.k = min(max(args.start_step, 0), self.n_windows - 1)
        self.elev, self.azim, self.roll = args.elev, args.azim, 0.0
        self._drawn = False

        self.fixed_lim, self.speed_rng = sample_global(mm)

        # left / right / home are matplotlib's default back/forward/home keys
        plt.rcParams["keymap.back"] = []
        plt.rcParams["keymap.forward"] = []
        plt.rcParams["keymap.home"] = ["h", "r"]

        self.fig = plt.figure(figsize=(12, 9), facecolor="#0e0e12")
        self.ax = self.fig.add_subplot(111, projection="3d")

        if args.colour_by == "speed":
            mappable = cm.ScalarMappable(norm=plt.Normalize(*self.speed_rng), cmap="plasma")
            cbar = self.fig.colorbar(mappable, ax=self.ax, shrink=0.6, pad=0.08)
            cbar.set_label("speed (m/s)", color="#cccccc")
            cbar.ax.yaxis.set_tick_params(color="#cccccc")
            plt.setp(plt.getp(cbar.ax, "yticklabels"), color="#cccccc")
        if not args.no_show:
            self.fig.text(0.5, 0.01, "arrow keys ←/→ : step forward/backward    Home/End : go to first/last", color="#777777", fontsize=8, ha="center")

        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

    # timestamp helpers
    def fmt_window(self, start_tg: float, end_tg: float) -> str:
        """
        Creates local Canberra timestamp: e.g. Monday 12th October 3:19pm - 3:20pm
        """
        a = datetime.fromtimestamp(self.created_s + start_tg, CANBERRA)
        b = datetime.fromtimestamp(self.created_s + end_tg, CANBERRA)

        def day(d):
            return f"{d:%A} {d.day}{ordinal(d.day)} {d:%B}"

        def clock(d):
            return f"{d.hour % 12 or 12}:{d.minute:02d}{'am' if d.hour < 12 else 'pm'}"

        if a.date() == b.date():
            return f"{day(a)} {clock(a)} - {clock(b)}" # standard, expected output
        return f"{day(a)} {clock(a)} - {day(b)} {clock(b)}" # in the off-chance that the time step crosses midnight


    # ---- Navigation ----
    def on_key(self, event):
        k = self.k
        if event.key == "right":
            k += 1
        elif event.key == "left":
            k -= 1
        elif event.key == "home":
            k = 0
        elif event.key == "end":
            k = self.n_windows - 1
        else:
            return
        k = min(max(k, 0), self.n_windows - 1)
        if k != self.k:
            self.draw(k)
            self.fig.canvas.draw_idle()

    # ---- Create one window ----
    def window_bounds(self, k):
        start = self.t0 + k * self.step
        end = start + self.step
        low = bisect.bisect_left(self.tg, start) if k > 0 else 0
        high = self.n if k == self.n_windows - 1 else bisect.bisect_left(self.tg, end)
        return start, end, low, high

    def draw(self, k):
        a = self.args
        self.k = k
        start, end, low, high = self.window_bounds(k)
        tracks, n_pts = load_window(self.mm, low, high, start, a.lookback if a.lookback is not None else self.step, a.max_points, a.chunk_records)

        ax = self.ax
        if self._drawn:
            # keep the last rotation across redraws
            self.elev, self.azim = ax.elev, ax.azim
            self.roll = getattr(ax, "roll", 0.0)
        ax.clear()
        self._drawn = True
        style_axes(ax)

        for gid in sorted(tracks):
            t = tracks[gid]
            colour = colour_for(gid)
            if a.colour_by == "speed" and t.xyz.shape[0] > 1:
                # Segment-coloured line: each small segment tinted by local speed,
                # so that fast/slow sections of the SAME track are visible
                pts = t.xyz.reshape(-1, 1, 3)
                segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
                speed = np.linalg.norm(t.vxyz, axis=1)[:-1]
                lc = Line3DCollection(segs, cmap="plasma", norm=plt.Normalize(*self.speed_rng))
                lc.set_array(speed)
                lc.set_linewidth(2.2)
                ax.add_collection3d(lc)
            else:
                ax.plot(t.xyz[:, 0], t.xyz[:, 1], t.xyz[:, 2], color=colour, linewidth=2.0, alpha=0.9, label=f"id {gid}")

            # start (circle) and end (x) markers show direction of travel
            ax.scatter(*t.xyz[0], color=colour, marker="o", s=45, edgecolor="white", linewidth=0.6, zorder=5)
            ax.scatter(*t.xyz[-1], color=colour, marker="x", s=45, linewidth=1.5, zorder=5)

            if a.arrows and t.xyz.shape[0] > 1:
                step = max(1, t.xyz.shape[0] // a.arrow_count)
                idx = np.arange(0, t.xyz.shape[0], step)
                speed = np.linalg.norm(t.vxyz[idx], axis=1)
                scale = a.arrow_scale / max(speed.max(), 1e-9)
                ax.quiver(t.xyz[idx, 0], t.xyz[idx, 1], t.xyz[idx, 2],
                          t.vxyz[idx, 0] * scale, t.vxyz[idx, 1] * scale, t.vxyz[idx, 2] * scale,
                          color=colour, alpha=0.6, linewidth=1.0, arrow_length_ratio=0.3)

        # Axis limits: fixed for the whole file unless --autoscale is passed
        center, radius = self.fixed_lim
        if a.autoscale and tracks:
            center, radius = equal_limits(np.concatenate([t.xyz for t in tracks.values()]))
        ax.set_xlim3d(center[0] - radius, center[0] + radius)
        ax.set_ylim3d(center[1] - radius, center[1] + radius)
        ax.set_zlim3d(center[2] - radius, center[2] + radius)
        if hasattr(ax, "roll"):
            ax.view_init(elev=self.elev, azim=self.azim, roll=self.roll)
        else:
            ax.view_init(elev=self.elev, azim=self.azim)

        shown = sum(t.xyz.shape[0] for t in tracks.values())
        n_tr = len(tracks)
        pts_str = f"{n_pts} points" if shown >= n_pts else f"{n_pts} points ({shown} plotted)"
        ax.set_title(
            f"{self.fmt_window(start, min(end, self.t1))}\n"
            f"Time window {k + 1}/{self.n_windows} • system time [{start:.2f}, {min(end, self.t1):.2f}] s • "
            f"{n_tr} track{'s' if n_tr != 1 else ''} • {pts_str}"
            + ("" if tracks else " • (no data)"),
            color="#eeeeee", fontsize=11, pad=14,
        )

        if a.colour_by != "speed" and 0 < n_tr <= 25:
            legend = ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0),
                               fontsize=8, facecolor="#1a1a20", edgecolor="#333333",
                               labelcolor="#dddddd", framealpha=0.9)
            legend.set_title("global track", prop={"size": 9})
            legend.get_title().set_color("#dddddd")

        if not a.quiet:
            print(f"window {k + 1}/{self.n_windows}: tg [{start:.2f}, {end:.2f}]  "
                  f"records {low}-{high}  tracks={n_tr}  points={n_pts}", file=sys.stderr)


## -------------------------------------------------------------------- ##
## Entry point
## -------------------------------------------------------------------- ##
def main():

    bee_facts = ["Did you know that bees have tiny magnetic cells near their stomach that act as in-built compasses. That means they can tell which way is north!",
                 "Did you know that bees partake in democracy? Not to elect a queen, but they have their own form of 'voting' on where to move their colony to if they need to relocate!",
                 "Did you know that bees have five eyes? Two compound eyes at the front of their head and three simple eyes on top! These simple eyes don't see things but instead measure light levels to help the bee stay stable while flying.",
                 ]

    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)

    p.add_argument("input", help="name of a .raemot3d file")
    p.add_argument("--step", type=float, default=60.0, help="time step length in global time seconds (default 60 = 1 minute)")
    p.add_argument("--lookback", type=float, default=None, help="how far beyond a window start (tg seconds) to look for a point that connects each track across the step boundary (default = --step)")
    p.add_argument("--start-step", type=int, default=0, help="Time step index to start on (defaults to 0 - the start)")
    p.add_argument("--max-points", type=int, default=0, help="max points plotted per track per time step (evenly strided). 0 = plot all points (default)")
    p.add_argument("--chunk-records", type=int, default=1_000_000, help="records read per chunk, 64 bytes each (default 1,000,000)")
    p.add_argument("--autoscale", action="store_true", help="fit axis limits to each time step rather than fixed whole-life limits")
    p.add_argument("--colour-by", choices=["id", "speed"], default="id", help="colour lines by global track id (default), or by local speed")
    p.add_argument("--arrows", action="store_true", help="overlay velocity direction arrows")
    p.add_argument("--arrow-count", type=int, default=12, help="arrows per track (default 12)")
    p.add_argument("--arrow-scale", type=float, default=0.3, help="arrow length as a fraction of the plot's radius (default 0.3)")
    p.add_argument("--elev", type=float, default=22.0, help="camera elevation angle")
    p.add_argument("--azim", type=float, default=-60.0, help="camera azimuth angle")
    p.add_argument("--save", default=None, help="save the starting window as an image to this name (as a png in same folder)")
    p.add_argument("--save-frames", default=None, help="save EVERY time step as frame_0000.png, ... into this directory")
    p.add_argument("--quiet", action="store_true", help="suppress progress output")
    p.add_argument("--no-show", action="store_true", help="don't open an interactive window")
    p.add_argument("--bee-facts", action="store_true", help="print out a super fun and interesting bee fact while you wait for the file to be read")
    args = p.parse_args()

    if args.step <= 0:
        p.error("--step must be > 0")
    if args.chunk_records < 1:
        p.error("--chunk-records must be >= 1")

    name = args.input if args.input.endswith(".raemot3d") else args.input + ".raemot3d"
    input_path = Path(__file__).parent / name

    if args.save is not None:
        output_path = Path(__file__).parent / Path(args.save).with_suffix(".png")

    print("Preparing to ride abroad in ostentation...")

    try:
        mm, created_unix_ns = open_records(input_path)

        if args.bee_facts:
            print(f"[Bee fact] {random.choice(bee_facts)}")

        print("Subsisting on Fuzz...")

        viewer = Viewer(mm, created_unix_ns, args)
    except (FileNotFoundError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"{args.input}: {viewer.n} records, tg [{viewer.t0:.2f}, {viewer.t1:.2f}] s -> "
          f"{viewer.n_windows} steps(s) of {viewer.step:g} s")
 
    if args.save_frames:
        out_dir = Path(__file__).parent / args.save_frames
        out_dir.mkdir(parents=True, exist_ok=True)
        for k in range(viewer.n_windows):
            viewer.draw(k)
            viewer.fig.savefig(out_dir / f"frame_{k:04d}.png", dpi=160,
                               facecolor=viewer.fig.get_facecolor())
        print(f"saved {viewer.n_windows} frame(s) to {out_dir}")
 
    viewer.draw(min(max(args.start_step, 0), viewer.n_windows - 1))
 
    if args.save:
        viewer.fig.savefig(output_path, dpi=160, facecolor=viewer.fig.get_facecolor())
        print(f"saved image to {output_path}")
 
    if not args.no_show:
        plt.show()

if __name__ == "__main__":
    main()
            
        