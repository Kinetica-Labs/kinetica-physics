"""Measure the 1833 x 1009 bucket recording from 21:51:36, in image pixels."""
import argparse
import csv

import cv2
import numpy as np


REGIONS = {
    "upper": [(897, 309), (1000, 242), (1107, 207), (1189, 330), (1241, 455),
              (1036, 545), (961, 474)],
    "middle": [(926, 651), (1030, 675), (1154, 674), (1276, 631), (1265, 681),
               (1203, 703), (1022, 712), (957, 692)],
    "lower": [(955, 756), (1178, 759), (1161, 939), (1013, 951), (981, 898)],
    "floor": [(409, 405), (740, 405), (740, 900), (409, 900)],
}


def features(first, polygon):
    mask = np.zeros(first.shape, np.uint8)
    cv2.fillPoly(mask, [np.int32(polygon)], 255)
    return cv2.goodFeaturesToTrack(first, 100, 0.003, 7, mask=mask)


def silhouette_edges(frame):
    # The upper part of the yellow bucket against the dark floor and sky.
    b, _, r = cv2.split(frame[180:480, 765:1350].astype(np.int16))
    mask = (r - b > 38) & (r > 95)
    occupied = mask.any(axis=1)
    return np.where(occupied, mask.argmax(axis=1), -1), np.where(
        occupied, mask.shape[1] - 1 - mask[:, ::-1].argmax(axis=1), -1)


def normal_pixels(frame):
    # Only the green/blue buckets and their vicinity, below the yellow body.
    b, g, r = cv2.split(frame[675:1009, 842:1345])
    return np.uint8((r > 200) & (g > 152) & (b < 95))


def contour_distance(edges):
    # Fill between the extremes so checkerboard seams do not enter the outer contour.
    x = np.arange(585)
    mask = np.uint8((x >= edges[0][:, None]) & (x <= edges[1][:, None]))
    contour = mask - cv2.erode(mask, np.ones((3, 3), np.uint8))
    return contour.astype(bool), cv2.distanceTransform(1 - contour, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)


def measure(video, csv_path, image):
    cap = cv2.VideoCapture(video)
    ok, frame = cap.read()
    if not ok or frame.shape[:2] != (1009, 1833):
        raise ValueError("Expected the original 1833 x 1009 recording")
    first = frame.copy()
    gray0 = cv2.cvtColor(first, cv2.COLOR_BGR2GRAY)
    regions = [features(gray0, polygon) for polygon in REGIONS.values()]
    start = np.concatenate(regions)
    low = high = first[:, 755:1350].copy()
    edges0 = silhouette_edges(first)
    contour0, distance0 = contour_distance(edges0)
    options = dict(winSize=(21, 21), maxLevel=3)
    previous = None
    kernel = np.ones((5, 5), np.uint8)
    with open(csv_path, "w", newline="") as out:
        writer = csv.writer(out)
        writer.writerow(["frame", "line_pixels_changed", "scanline_extent_max_px", "contour_p99_px", "contour_max_px"] +
                        [name + suffix for name in REGIONS for suffix in
                         ("_features", "_dx_px", "_dy_px", "_p90_px")])
        index = 0
        while ok:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            now, st, _ = cv2.calcOpticalFlowPyrLK(gray0, gray, start, None, **options)
            back, valid, _ = cv2.calcOpticalFlowPyrLK(gray, gray0, now, None, **options)
            keep = (st.ravel() > 0) & (valid.ravel() > 0) & (
                np.linalg.norm((back - start).reshape(-1, 2), axis=1) < 0.65)
            row, offset = [], 0
            for region in regions:
                part = slice(offset, offset + len(region))
                delta = (now[part] - start[part]).reshape(-1, 2)[keep[part]]
                if len(delta) < 4:
                    raise ValueError(f"Too few tracked features at frame {index}")
                row.extend([len(delta), *np.median(delta, axis=0),
                            np.quantile(np.linalg.norm(delta, axis=1), 0.90)])
                offset += len(region)
            normals = normal_pixels(frame)
            changed = 0 if previous is None else max(
                np.count_nonzero(normals & ~cv2.dilate(previous, kernel)),
                np.count_nonzero(previous & ~cv2.dilate(normals, kernel)))
            edges = silhouette_edges(frame)
            # Use scanlines occupied in both frames; test vertical extent separately.
            common = (edges0[0] >= 0) & (edges[0] >= 0)
            top = abs(int(np.flatnonzero(edges0[0] >= 0)[0]) - int(np.flatnonzero(edges[0] >= 0)[0]))
            span = max(top, *(abs(a[common] - b[common]).max() for a, b in zip(edges, edges0)))
            contour, distance = contour_distance(edges)
            a, b = distance0[contour], distance[contour0]
            writer.writerow([index, changed, span, max(np.quantile(a, .99), np.quantile(b, .99)),
                             max(a.max(), b.max()), *row])
            low = np.minimum(low, frame[:, 755:1350])
            high = np.maximum(high, frame[:, 755:1350])
            index += 1
            previous = normals
            ok, frame = cap.read()
    cap.release()
    if image:
        result = first[:, 755:1350].copy()
        result[(high.astype(int) - low.astype(int)).max(axis=2) > 86] = (30, 35, 255)
        if not cv2.imwrite(image, result[145:]):
            raise OSError(image)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("video")
    parser.add_argument("csv")
    parser.add_argument("--image")
    args = parser.parse_args()
    cv2.setNumThreads(3)
    measure(args.video, args.csv, args.image)
