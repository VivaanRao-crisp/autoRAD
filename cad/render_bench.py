"""cad/render_bench.py - PNG pictures of the printed parts and of the assembly (closed / open) into docs/img/.

Run from the repo root:   python cad/render_bench.py
Simple shaded pictures made with matplotlib (no CAD viewer needed). For checking shapes, not for the report.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np                                            # noqa: E402
import matplotlib                                             # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                               # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection       # noqa: E402
import params as p                                            # noqa: E402
import bench_parts as bp                                      # noqa: E402
import bench_assembly as ba                                   # noqa: E402

IMG = os.path.join(ROOT, "docs", "img")
LIGHT = np.array([0.35, -0.45, 0.82])
COLOURS = {"handle_R": "#d9822b", "handle_L": "#2b7bd9", "link_R": "#e0b020", "link_L": "#30a890", "carriage": "#c0392b",
           "board": "#d8c8a8", "dowel_R": "#9a7b4f", "dowel_L": "#9a7b4f"}


def triangles(shape, tol=0.2):
    """Triangles (n, 3, 3) of a solid."""
    verts, tris = shape.tessellate(tol)
    v = np.array([(q.x, q.y, q.z) for q in verts])
    return v[np.array(tris)]


def add(ax, shape, colour, alpha=1.0):
    """Draw one solid with flat shading."""
    t = triangles(shape)
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    shade = 0.45 + 0.55 * np.abs(n @ (LIGHT / np.linalg.norm(LIGHT)))
    rgb = np.array(matplotlib.colors.to_rgb(colour))
    cols = np.clip(shade[:, None] * rgb[None, :], 0, 1)
    ax.add_collection3d(Poly3DCollection(t, facecolors=np.c_[cols, np.full(len(cols), alpha)], edgecolors="none"))
    return t.reshape(-1, 3)


def frame(ax, pts, elev, azim, title):
    """Equal-scale axes around the points."""
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    c, r = 0.5 * (lo + hi), 0.5 * (hi - lo).max() * 1.05
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1)); ax.set_proj_type("ortho"); ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z"); ax.set_title(title, fontsize=9)


def render_parts(parts):
    """One picture per printed model, design frame: view along the plate normal and an oblique view."""
    for name in ("P1_handle", "P2_link", "P3_carriage"):
        fig = plt.figure(figsize=(11, 5.5))
        for k, (elev, azim, label) in enumerate(((90, -90, "front (+z toward you)"), (25, -60, "oblique"))):
            ax = fig.add_subplot(1, 2, k + 1, projection="3d")
            pts = add(ax, parts[name][0], "#c9792b")
            frame(ax, pts, elev, azim, "%s - %s" % (name, label))
        fig.savefig(os.path.join(IMG, "bench_%s.png" % name), dpi=110)
        plt.close(fig)
        print("written docs/img/bench_%s.png" % name)


def render_assembly(parts):
    """Assembly at closed and open: seen from the front, from behind, and obliquely from behind."""
    for label, tr in (("closed", p.BENCH_CHECK_TRAVELS_MM[0]), ("open", p.BENCH_CHECK_TRAVELS_MM[-1])):
        bodies, _fx, _cr = ba.assembly(tr, parts)
        fig = plt.figure(figsize=(16.5, 6.5))
        views = ((90, -90, "front", lambda n: True), (-90, 90, "from behind (board hidden)", lambda n: n != "board"),
                 (-25, 60, "oblique from behind (board hidden)", lambda n: n != "board"))
        for k, (elev, azim, vlabel, keep) in enumerate(views):
            ax = fig.add_subplot(1, 3, k + 1, projection="3d")
            pts = [add(ax, s, COLOURS.get(n, "#8a8f98"), 0.35 if n == "board" else 1.0) for n, s in bodies.items() if keep(n)]
            frame(ax, np.vstack(pts), elev, azim, "%s, carriage travel %.0f mm - %s" % (label, tr, vlabel))
        fig.savefig(os.path.join(IMG, "bench_assembly_%s.png" % label), dpi=100)
        plt.close(fig)
        print("written docs/img/bench_assembly_%s.png" % label)


if __name__ == "__main__":
    os.makedirs(IMG, exist_ok=True)
    all_parts = bp.build_all()
    render_parts(all_parts)
    render_assembly(all_parts)
