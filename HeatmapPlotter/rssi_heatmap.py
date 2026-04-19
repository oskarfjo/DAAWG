"""
RSSI Heatmap Generator
----------------------
Reads logg.csv (columns: rssi, lat, lon) and produces a heatmap where:
  - RSSI <= 200  -> warm (strong signal / hotspot)
  - RSSI >= 800  -> cold (weak signal)
  - Uncovered cells stay white

The script bins irregular (lat, lon) samples onto a regular grid and
auto-adjusts grid bounds and resolution to the data.

Usage:
    python rssi_heatmap.py                     # reads ./logg.csv
    python rssi_heatmap.py path/to/data.csv    # custom input
    python rssi_heatmap.py -o out.png -g 150   # custom output / grid size
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, Normalize


# ---- Configuration ----------------------------------------------------------

WARM_THRESHOLD = 200   # rssi <= this is "warm" (hotspot)
COLD_THRESHOLD = 800   # rssi >= this is "cold"
DEFAULT_GRID   = 100   # target cells along the longer axis; auto-tuned below


def build_colormap():
    """Diverging colormap: deep red (warm) -> yellow -> light -> blue (cold)."""
    stops = [
        (0.00, "#7a0403"),  # deep red  (very warm / strongest signal)
        (0.25, "#e63946"),  # red
        (0.45, "#f4a261"),  # orange
        (0.55, "#f1faee"),  # near-white transition
        (0.75, "#457b9d"),  # blue
        (1.00, "#1d3557"),  # deep blue (very cold / weakest signal)
    ]
    return LinearSegmentedColormap.from_list("rssi_warm_cold", stops)


def choose_grid_shape(lats, lons, target_cells=DEFAULT_GRID):
    """
    Pick grid dimensions (ny, nx) so each cell is roughly square in the
    coordinate space, and the longer axis has ~target_cells bins.
    Avoids producing a grid far finer than the sample density.
    """
    lat_span = lats.max() - lats.min()
    lon_span = lons.max() - lons.min()
    if lat_span == 0: lat_span = 1e-9
    if lon_span == 0: lon_span = 1e-9

    if lon_span >= lat_span:
        nx = target_cells
        ny = max(10, int(round(target_cells * (lat_span / lon_span))))
    else:
        ny = target_cells
        nx = max(10, int(round(target_cells * (lon_span / lat_span))))

    # Don't oversample: cap total cells at ~4x the number of samples
    n_samples = len(lats)
    max_cells = max(400, n_samples * 4)
    if nx * ny > max_cells:
        scale = (max_cells / (nx * ny)) ** 0.5
        nx = max(10, int(nx * scale))
        ny = max(10, int(ny * scale))

    return ny, nx


def bin_to_grid(df, ny, nx):
    """
    Assign each (lat, lon) sample to its grid cell and average the RSSI
    values in each cell. Cells with no samples stay as NaN (rendered white).
    """
    lat_min, lat_max = df["lat"].min(), df["lat"].max()
    lon_min, lon_max = df["lon"].min(), df["lon"].max()

    # Pad slightly so max values land inside the last bin
    lat_pad = (lat_max - lat_min) * 1e-6 or 1e-9
    lon_pad = (lon_max - lon_min) * 1e-6 or 1e-9
    lat_edges = np.linspace(lat_min, lat_max + lat_pad, ny + 1)
    lon_edges = np.linspace(lon_min, lon_max + lon_pad, nx + 1)

    iy = np.clip(np.digitize(df["lat"].values, lat_edges) - 1, 0, ny - 1)
    ix = np.clip(np.digitize(df["lon"].values, lon_edges) - 1, 0, nx - 1)

    sums   = np.zeros((ny, nx), dtype=np.float64)
    counts = np.zeros((ny, nx), dtype=np.int64)
    np.add.at(sums,   (iy, ix), df["rssi"].values)
    np.add.at(counts, (iy, ix), 1)

    grid = np.full((ny, nx), np.nan, dtype=np.float64)
    mask = counts > 0
    grid[mask] = sums[mask] / counts[mask]

    extent = (lon_min, lon_max, lat_min, lat_max)  # (left, right, bottom, top)
    coverage = mask.sum() / mask.size
    return grid, extent, coverage


def plot_heatmap(grid, extent, coverage, out_path, title="RSSI Heatmap"):
    cmap = build_colormap()
    cmap.set_bad(color="white")  # NaN cells render white (no coverage)

    # Anchor the color scale at the warm/cold thresholds; clamp extremes.
    norm = Normalize(vmin=WARM_THRESHOLD, vmax=COLD_THRESHOLD, clip=True)
    masked = np.ma.masked_invalid(grid)

    fig, ax = plt.subplots(figsize=(10, 8), dpi=120)
    im = ax.imshow(
        masked,
        origin="lower",
        extent=extent,
        cmap=cmap,
        norm=norm,
        interpolation="nearest",
        aspect="auto",
    )

    cbar = fig.colorbar(im, ax=ax, shrink=0.85)
    cbar.set_label("RSSI  (low = warm / strong, high = cold / weak)")

    ax.set_title(f"{title}\nGrid {grid.shape[1]}x{grid.shape[0]}  •  "
                 f"coverage {coverage*100:.1f}%")
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.grid(False)

    fig.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)


def load_csv(path):
    df = pd.read_csv(path)
    df.columns = [c.strip().lower() for c in df.columns]
    required = {"rssi", "lat", "lon"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"CSV is missing required columns: {missing}. "
                         f"Found: {list(df.columns)}")
    df = df[["rssi", "lat", "lon"]].apply(pd.to_numeric, errors="coerce").dropna()
    if df.empty:
        raise ValueError("No valid numeric rows found in CSV.")
    return df


def main():
    ap = argparse.ArgumentParser(description="Generate an RSSI heatmap from logg.csv")
    ap.add_argument("csv", nargs="?", default="logg.csv",
                    help="Path to the input CSV (default: logg.csv)")
    ap.add_argument("-o", "--output", default="rssi_heatmap.png",
                    help="Output image path (default: rssi_heatmap.png)")
    ap.add_argument("-g", "--grid", type=int, default=DEFAULT_GRID,
                    help=f"Target cells on longer axis (default: {DEFAULT_GRID})")
    args = ap.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"Error: {csv_path} not found", file=sys.stderr)
        sys.exit(1)

    df = load_csv(csv_path)
    print(f"Loaded {len(df)} samples.")
    print(f"  RSSI range: {df['rssi'].min():.1f} to {df['rssi'].max():.1f}")
    print(f"  Lat  range: {df['lat'].min():.6f} to {df['lat'].max():.6f}")
    print(f"  Lon  range: {df['lon'].min():.6f} to {df['lon'].max():.6f}")

    ny, nx = choose_grid_shape(df["lat"].values, df["lon"].values, args.grid)
    grid, extent, coverage = bin_to_grid(df, ny, nx)
    print(f"Grid: {nx} x {ny}  |  coverage: {coverage*100:.1f}%")

    plot_heatmap(grid, extent, coverage, args.output)
    print(f"Saved heatmap to {args.output}")


if __name__ == "__main__":
    main()
