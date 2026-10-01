#!/usr/bin/env python3
"""Merge a map bundle's corner/surf PCDs into one voxel-downsampled PCD for NDT.

  prepare_map.py <bundle_dir> <out.pcd> [--voxel 0.4]

The bundle PCDs are binary PCD v0.7, fields x y z intensity (float32). Each voxel keeps the
centroid of its points.
"""

import argparse

import numpy as np

LAYERS = ("full_corner.pcd", "full_surf.pcd")


def read_pcd(path):
    with open(path, "rb") as f:
        while True:
            line = f.readline().decode("ascii")
            if line.startswith("POINTS"):
                count = int(line.split()[1])
            elif line.startswith("DATA"):
                break
        data = np.fromfile(f, dtype=np.float32, count=count * 4)
    return data.reshape(count, 4)


def voxel_downsample(points, leaf):
    keys = np.floor(points[:, :3] / leaf).astype(np.int64)
    keys -= keys.min(axis=0)
    dims = keys.max(axis=0) + 1
    flat = (keys[:, 0] * dims[1] + keys[:, 1]) * dims[2] + keys[:, 2]
    _, inverse, counts = np.unique(flat, return_inverse=True, return_counts=True)
    sums = np.zeros((counts.size, 4), dtype=np.float64)
    np.add.at(sums, inverse, points)
    return (sums / counts[:, None]).astype(np.float32)


def write_pcd(path, points):
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(points.tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle_dir")
    parser.add_argument("out_pcd")
    parser.add_argument("--voxel", type=float, default=0.4, help="leaf size [m]")
    args = parser.parse_args()

    merged = np.concatenate([read_pcd(f"{args.bundle_dir}/{name}") for name in LAYERS])
    out = voxel_downsample(merged, args.voxel)
    print(f"{len(merged):,} -> {len(out):,} points (voxel {args.voxel} m)")
    write_pcd(args.out_pcd, out)


if __name__ == "__main__":
    main()
