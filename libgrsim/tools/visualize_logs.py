#!/usr/bin/env python3
"""Render grSim headless CSV logs into a top-down 2D animation (GIF/MP4).

Robots are drawn as colored circles with fading position trails.
Command vectors are drawn as arrows from the robot center.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from collections import defaultdict, deque
from pathlib import Path

import numpy as np

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    print("Pillow is required: pip install pillow", file=sys.stderr)
    sys.exit(1)


def parse_meta(path: Path) -> dict:
    meta = {}
    if not path.exists():
        return meta
    for line in path.read_text().splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            meta[k.strip()] = v.strip()
    return meta


def load_vision(path: Path):
    """Return ordered frames: list of dicts with t, ball, robots."""
    frames = {}
    with path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame = int(float(row["frame"]))
            t = float(row["t"])
            ball = (float(row["ball_x"]), float(row["ball_y"]), float(row["ball_z"]))
            if frame not in frames:
                frames[frame] = {"t": t, "ball": ball, "robots": []}
            team = row["team"]
            rid = int(float(row["id"]))
            if team in ("blue", "yellow") and rid >= 0:
                frames[frame]["robots"].append(
                    {
                        "team": team,
                        "id": rid,
                        "x": float(row["x"]),
                        "y": float(row["y"]),
                        "ori": float(row["orientation"]),
                        "vx": float(row.get("vx", 0) or 0),
                        "vy": float(row.get("vy", 0) or 0),
                    }
                )
    return [frames[k] for k in sorted(frames.keys())]


def load_commands(path: Path):
    """Map frame -> list of commands."""
    cmds = defaultdict(list)
    if not path.exists():
        return cmds
    with path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame = int(float(row["frame"]))
            cmds[frame].append(
                {
                    "team": row["team"],
                    "id": int(float(row["id"])),
                    "vt": float(row["vel_tangent"]),
                    "vn": float(row["vel_normal"]),
                    "vw": float(row["vel_angular"]),
                }
            )
    return cmds


def world_to_pix(x, y, field_l, field_w, width, height, margin=40):
    # field coords: x along length, y along width, origin center
    sx = (width - 2 * margin) / field_l
    sy = (height - 2 * margin) / field_w
    s = min(sx, sy)
    px = width / 2 + x * s
    py = height / 2 - y * s  # invert y for image coords
    return px, py, s


def draw_field(draw, field_l, field_w, width, height, margin=40):
    # grass
    draw.rectangle([0, 0, width, height], fill=(28, 90, 42))
    _, _, s = world_to_pix(0, 0, field_l, field_w, width, height, margin)
    # field rectangle
    x0, y0, _ = world_to_pix(-field_l / 2, field_w / 2, field_l, field_w, width, height, margin)
    x1, y1, _ = world_to_pix(field_l / 2, -field_w / 2, field_l, field_w, width, height, margin)
    draw.rectangle([x0, y0, x1, y1], outline=(240, 240, 240), width=2)
    # center line
    cx0, cy0, _ = world_to_pix(0, field_w / 2, field_l, field_w, width, height, margin)
    cx1, cy1, _ = world_to_pix(0, -field_w / 2, field_l, field_w, width, height, margin)
    draw.line([cx0, cy0, cx1, cy1], fill=(220, 220, 220), width=1)
    # center circle
    r = 0.5 * s
    draw.ellipse([width / 2 - r, height / 2 - r, width / 2 + r, height / 2 + r],
                 outline=(220, 220, 220), width=1)
    # goals
    gw = 1.8
    gd = 0.18
    for sign in (-1, 1):
        gx0, gy0, _ = world_to_pix(sign * field_l / 2, gw / 2, field_l, field_w, width, height, margin)
        gx1, gy1, _ = world_to_pix(sign * (field_l / 2 + gd), -gw / 2, field_l, field_w, width, height, margin)
        draw.rectangle([min(gx0, gx1), min(gy0, gy1), max(gx0, gx1), max(gy0, gy1)],
                       outline=(255, 255, 255), width=2)


def render_frame(frame, cmds_for_frame, trails, field_l, field_w, width, height,
                 robot_radius=0.09, trail_len=40):
    img = Image.new("RGB", (width, height), (20, 20, 20))
    draw = ImageDraw.Draw(img, "RGBA")
    draw_field(draw, field_l, field_w, width, height)

    # update trails
    for rob in frame["robots"]:
        key = (rob["team"], rob["id"])
        trails[key].append((rob["x"], rob["y"]))
        while len(trails[key]) > trail_len:
            trails[key].popleft()

    # trails
    for key, pts in trails.items():
        team = key[0]
        base = (70, 140, 255) if team == "blue" else (255, 220, 60)
        n = len(pts)
        for i, (x, y) in enumerate(pts):
            alpha = int(40 + 180 * (i + 1) / max(n, 1))
            px, py, s = world_to_pix(x, y, field_l, field_w, width, height)
            rr = max(2, int(0.04 * s))
            color = base + (min(alpha, 255),)
            draw.ellipse([px - rr, py - rr, px + rr, py + rr], fill=color)

    # robots + heading
    cmd_map = {(c["team"], c["id"]): c for c in cmds_for_frame}
    for rob in frame["robots"]:
        px, py, s = world_to_pix(rob["x"], rob["y"], field_l, field_w, width, height)
        rr = max(6, int(robot_radius * s))
        if rob["team"] == "blue":
            fill = (50, 110, 255, 255)
            outline = (200, 220, 255, 255)
        else:
            fill = (240, 210, 40, 255)
            outline = (255, 255, 200, 255)
        draw.ellipse([px - rr, py - rr, px + rr, py + rr], fill=fill, outline=outline, width=2)
        # orientation tick
        hx = px + rr * math.cos(rob["ori"])
        hy = py - rr * math.sin(rob["ori"])
        draw.line([px, py, hx, hy], fill=(255, 255, 255, 255), width=2)
        # id
        draw.text((px - 4, py - 5), str(rob["id"]), fill=(0, 0, 0, 255))

        # command arrow (local vel -> global)
        c = cmd_map.get((rob["team"], rob["id"]))
        if c:
            cang = rob["ori"]
            # local tangent=forward, normal=left
            vgx = c["vt"] * math.cos(cang) - c["vn"] * math.sin(cang)
            vgy = c["vt"] * math.sin(cang) + c["vn"] * math.cos(cang)
            scale = 0.35 * s  # m/s to pixels-ish
            ex = px + vgx * scale
            ey = py - vgy * scale
            draw.line([px, py, ex, ey], fill=(255, 80, 80, 230), width=3)
            # arrow head
            ang = math.atan2(-(ey - py), ex - px)
            ah = 8
            draw.polygon(
                [
                    (ex, ey),
                    (ex - ah * math.cos(ang - 0.4), ey - ah * math.sin(ang - 0.4)),
                    (ex - ah * math.cos(ang + 0.4), ey - ah * math.sin(ang + 0.4)),
                ],
                fill=(255, 80, 80, 230),
            )

    # ball
    bx, by, _ = frame["ball"]
    bpx, bpy, s = world_to_pix(bx, by, field_l, field_w, width, height)
    br = max(4, int(0.0215 * s * 2.5))
    draw.ellipse([bpx - br, bpy - br, bpx + br, bpy + br], fill=(255, 140, 0, 255),
                 outline=(255, 220, 180, 255), width=1)

    # HUD
    hud = f"t={frame['t']:.2f}s  robots={len(frame['robots'])}"
    draw.rectangle([8, 8, 280, 32], fill=(0, 0, 0, 140))
    draw.text((14, 12), hud, fill=(255, 255, 255, 255))
    return img.convert("RGB")


def main():
    ap = argparse.ArgumentParser(description="Visualize grSim headless logs")
    ap.add_argument("--vision", required=True, help="Vision CSV path")
    ap.add_argument("--commands", default=None, help="Commands CSV path")
    ap.add_argument("--meta", default=None, help="Optional meta.txt")
    ap.add_argument("--out", default="output/videos/run.gif", help="Output GIF or MP4 path")
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--stride", type=int, default=2, help="Use every Nth frame")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--field-length", type=float, default=12.0)
    ap.add_argument("--field-width", type=float, default=9.0)
    ap.add_argument("--max-frames", type=int, default=0, help="0 = all")
    args = ap.parse_args()

    vision_path = Path(args.vision)
    cmd_path = Path(args.commands) if args.commands else Path(str(vision_path).replace("_vision.csv", "_commands.csv"))
    meta = parse_meta(Path(args.meta)) if args.meta else {}
    field_l = float(meta.get("field_length", args.field_length))
    field_w = float(meta.get("field_width", args.field_width))

    frames = load_vision(vision_path)
    cmds = load_commands(cmd_path)
    if not frames:
        print("No frames in vision log", file=sys.stderr)
        return 1

    if args.max_frames > 0:
        frames = frames[: args.max_frames]

    trails = defaultdict(lambda: deque(maxlen=50))
    images = []
    for i, fr in enumerate(frames):
        if i % args.stride != 0:
            continue
        # nearest command frame
        # vision frames are dense; commands sparse — use last known
        frame_id = None
        # re-parse not available; use index-based approx via t
        # Better: rebuild frame ids
        images.append(
            render_frame(fr, [], trails, field_l, field_w, args.width, args.height)
        )

    # second pass with commands: reload frames with ids
    frames_raw = {}
    with vision_path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame = int(float(row["frame"]))
            t = float(row["t"])
            ball = (float(row["ball_x"]), float(row["ball_y"]), float(row["ball_z"]))
            if frame not in frames_raw:
                frames_raw[frame] = {"frame": frame, "t": t, "ball": ball, "robots": []}
            team = row["team"]
            rid = int(float(row["id"]))
            if team in ("blue", "yellow") and rid >= 0:
                frames_raw[frame]["robots"].append(
                    {
                        "team": team,
                        "id": rid,
                        "x": float(row["x"]),
                        "y": float(row["y"]),
                        "ori": float(row["orientation"]),
                    }
                )
    ordered = [frames_raw[k] for k in sorted(frames_raw.keys())]
    if args.max_frames > 0:
        ordered = ordered[: args.max_frames]

    trails = defaultdict(lambda: deque(maxlen=50))
    images = []
    last_cmds = []
    for i, fr in enumerate(ordered):
        if i % args.stride != 0:
            continue
        fid = fr["frame"]
        if fid in cmds:
            last_cmds = cmds[fid]
        images.append(
            render_frame(fr, last_cmds, trails, field_l, field_w, args.width, args.height)
        )

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    if not images:
        print("No images rendered", file=sys.stderr)
        return 1

    duration_ms = int(1000 / max(args.fps, 1))
    if out.suffix.lower() == ".gif":
        images[0].save(
            out,
            save_all=True,
            append_images=images[1:],
            duration=duration_ms,
            loop=0,
            optimize=False,
        )
        print(f"Wrote GIF {out} ({len(images)} frames)")
    else:
        # write PNG sequence and use ffmpeg if available
        seq_dir = out.with_suffix("").as_posix() + "_frames"
        os.makedirs(seq_dir, exist_ok=True)
        for i, im in enumerate(images):
            im.save(os.path.join(seq_dir, f"frame_{i:05d}.png"))
        import subprocess

        cmd = [
            "ffmpeg", "-y", "-framerate", str(args.fps),
            "-i", os.path.join(seq_dir, "frame_%05d.png"),
            "-pix_fmt", "yuv420p",
            str(out),
        ]
        try:
            subprocess.check_call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"Wrote video {out} ({len(images)} frames)")
        except Exception as e:
            gif_fallback = out.with_suffix(".gif")
            images[0].save(
                gif_fallback,
                save_all=True,
                append_images=images[1:],
                duration=duration_ms,
                loop=0,
            )
            print(f"ffmpeg failed ({e}); wrote GIF {gif_fallback}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
