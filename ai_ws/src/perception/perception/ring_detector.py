#!/usr/bin/env python3
"""
Offline prototype for the A4 two-ring target.

Target geometry (all diameters in mm, concentric):
    outer black ring:  outer 140, inner 110
    white separation:  outer 110, inner 80
    inner black ring:  outer 80,  inner 50
    centre white disk: diameter 50

The four radii are therefore 70, 55, 40, 25 mm (15 mm per band).
The detector finds the white A4 sheet, rectifies it to a canonical
420x594 image (2 px/mm), then validates the two concentric black rings.
Pose/distance is obtained with solvePnP on the four A4 corners.

Run:
    python3 -m perception.ring_detector
    python3 -m perception.ring_detector --image /path/to/photo.jpg
"""

import argparse
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class RingSpec:
    a4_w_mm: float = 210.0
    a4_h_mm: float = 297.0
    # Measured print (2026-11-02):
    #   outer black ring: outer 167 mm, inner 127 mm, width 20 mm
    #   white gap:        outer 127 mm, inner 108 mm, width 9.5 mm
    #   inner black ring: outer 108 mm, inner 66 mm, width 21 mm
    # Radii: 83.5 / 63.5 / 54 / 33 mm
    outer_outer_d_mm: float = 167.0
    outer_inner_d_mm: float = 127.0
    white_inner_d_mm: float = 108.0
    inner_inner_d_mm: float = 66.0
    pixels_per_mm: float = 2.0

    @property
    def radii_mm(self) -> List[float]:
        return [
            self.outer_outer_d_mm / 2.0,
            self.outer_inner_d_mm / 2.0,
            self.white_inner_d_mm / 2.0,
            self.inner_inner_d_mm / 2.0,
        ]

    def validate(self) -> None:
        r = self.radii_mm
        if not all(r[i] > r[i + 1] for i in range(len(r) - 1)):
            raise ValueError(f"Ring radii must decrease outward->inward: {r}")
        widths = [r[0] - r[1], r[1] - r[2], r[2] - r[3]]
        if any(w <= 0 for w in widths):
            raise ValueError(f"Invalid ring band widths: {widths}")
        print("Target radii (mm): {}; band widths (mm): {}".format(
            [round(v, 1) for v in r], [round(v, 1) for v in widths]))


@dataclass
class Detection:
    valid: bool
    message: str = ""
    method: str = ""
    a4_corners: Optional[np.ndarray] = None
    center_px: Optional[Tuple[float, float]] = None
    center_warp: Optional[Tuple[float, float]] = None
    radii_px: Optional[List[float]] = None
    bbox_px: Optional[Tuple[int, int, int, int]] = None
    center_camera_mm: Optional[np.ndarray] = None
    distance_m: Optional[float] = None
    rvec: Optional[np.ndarray] = None
    tvec: Optional[np.ndarray] = None
    a4_warp: Optional[np.ndarray] = None
    annotated: Optional[np.ndarray] = None


def order_corners(points: np.ndarray) -> np.ndarray:
    """Order 4 points as top-left, top-right, bottom-right, bottom-left."""
    pts = np.asarray(points, dtype=np.float32).reshape(4, 2)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).ravel()
    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmin(d)]
    bl = pts[np.argmax(d)]
    return np.array([tl, tr, br, bl], dtype=np.float32)


def find_a4(image: np.ndarray, spec: RingSpec) -> Optional[np.ndarray]:
    """Find the largest white four-corner sheet (the A4 target)."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, th = cv2.threshold(blur, 0, 255,
                          cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

    contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    image_area = image.shape[0] * image.shape[1]
    best = None
    best_area = 0.0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < image_area * 0.01:
            continue
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
        if len(approx) != 4 or not cv2.isContourConvex(approx):
            continue

        rect = cv2.minAreaRect(approx)
        w, h = rect[1]
        if min(w, h) <= 0:
            continue
        ratio = max(w, h) / min(w, h)
        # A4 is 297/210 = 1.414; allow perspective and print margins.
        if not (1.20 <= ratio <= 1.75):
            continue

        # Reject the image border / entire frame: on a white desk the
        # Otsu mask can otherwise select the whole image as the "sheet".
        bx, by, bw, bh = cv2.boundingRect(approx)
        if (bx <= 2 or by <= 2
                or bx + bw >= image.shape[1] - 2
                or by + bh >= image.shape[0] - 2):
            continue
        if area > image_area * 0.90:
            continue

        if area > best_area:
            best_area = area
            best = approx

    if best is None:
        return None
    return order_corners(best.reshape(4, 2))


def warp_a4(image: np.ndarray, corners: np.ndarray,
            spec: RingSpec) -> Tuple[np.ndarray, np.ndarray]:
    """Rectify the A4 sheet to a canonical fronto-parallel image."""
    w = int(round(spec.a4_w_mm * spec.pixels_per_mm))
    h = int(round(spec.a4_h_mm * spec.pixels_per_mm))
    dst = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]],
                   dtype=np.float32)
    M = cv2.getPerspectiveTransform(corners, dst)
    return cv2.warpPerspective(image, M, (w, h)), M


def _ring_candidates(warp: np.ndarray,
                     spec: RingSpec) -> List[dict]:
    """Find black annuli in the rectified A4 image."""
    gray = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 0, 255,
                            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            np.ones((3, 3), np.uint8))

    result = cv2.findContours(mask, cv2.RETR_TREE,
                              cv2.CHAIN_APPROX_SIMPLE)
    contours, hierarchy = result[-2], result[-1]
    if hierarchy is None:
        return []
    hierarchy = hierarchy[0]

    expected = [r * spec.pixels_per_mm for r in spec.radii_mm]
    outer_width_mm = spec.radii_mm[0] - spec.radii_mm[1]
    inner_width_mm = spec.radii_mm[2] - spec.radii_mm[3]
    expected_widths = {
        expected[0]: outer_width_mm * spec.pixels_per_mm,
        expected[2]: inner_width_mm * spec.pixels_per_mm,
    }
    candidates = []

    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area < 50:
            continue

        (cx, cy), r_out = cv2.minEnclosingCircle(contour)
        if r_out <= 0:
            continue
        peri = cv2.arcLength(contour, True)
        circularity = 4.0 * math.pi * area / (peri * peri) if peri > 0 else 0.0

        child = hierarchy[i][2]
        if child == -1:
            continue
        (icx, icy), r_in = cv2.minEnclosingCircle(contours[child])
        if math.hypot(icx - cx, icy - cy) > 0.15 * r_out:
            continue

        width = r_out - r_in
        for target in (expected[0], expected[2]):
            if abs(r_out - target) / target <= 0.15:
                want_width = expected_widths[target]
                if abs(width - want_width) > 0.30 * want_width:
                    break
                candidates.append({
                    "center": (float(cx), float(cy)),
                    "r_out": float(r_out),
                    "r_in": float(r_in),
                    "circularity": float(circularity),
                    "target": float(target),
                    "area": float(area),
                })
                break

    return candidates


def detect_rings(warp: np.ndarray,
                 spec: RingSpec) -> Optional[Tuple[Tuple[float, float],
                                                   List[float], dict, dict]]:
    """Validate the two concentric black rings in the rectified image."""
    candidates = _ring_candidates(warp, spec)
    if not candidates:
        return None

    expected = [r * spec.pixels_per_mm for r in spec.radii_mm]
    outer = min(candidates, key=lambda c: abs(c["r_out"] - expected[0]))
    inner = min(candidates, key=lambda c: abs(c["r_out"] - expected[2]))

    if outer is inner:
        return None

    center_dist = math.hypot(
        outer["center"][0] - inner["center"][0],
        outer["center"][1] - inner["center"][1])
    if center_dist > 0.10 * expected[0]:
        return None

    center = (
        (outer["center"][0] + inner["center"][0]) / 2.0,
        (outer["center"][1] + inner["center"][1]) / 2.0,
    )
    radii = [outer["r_out"], outer["r_in"], inner["r_out"], inner["r_in"]]

    # Validate the four radii against the expected pattern.
    for got, want in zip(radii, expected):
        if abs(got - want) / want > 0.15:
            return None

    return center, radii, outer, inner


def detect_rings_direct(image: np.ndarray,
                        spec: RingSpec) -> Optional[Tuple[Tuple[float, float],
                                                          List[float], dict, dict]]:
    """
    Detect the two concentric rings directly in the original image.

    This is the fallback when the white A4 sheet cannot be separated from a
    white desk/keyboard.  It uses scale-invariant ratios derived from the
    measured print:

        outer ring width / outer radius  = 20 / 83.5   = 0.240
        inner ring width / outer radius  = 21 / 54     = 0.389
        outer_outer / inner_outer        = 83.5 / 54   = 1.546
        outer_inner / inner_outer        = 63.5 / 54   = 1.176
        inner_outer / inner_inner        = 54 / 33     = 1.636
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, mask = cv2.threshold(blur, 0, 255,
                            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            np.ones((3, 3), np.uint8))

    result = cv2.findContours(mask, cv2.RETR_TREE,
                              cv2.CHAIN_APPROX_SIMPLE)
    contours, hierarchy = result[-2], result[-1]
    if hierarchy is None:
        return None
    hierarchy = hierarchy[0]

    candidates = []
    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area < 200:
            continue
        (cx, cy), r_out = cv2.minEnclosingCircle(contour)
        if r_out < 15:
            continue

        child = hierarchy[i][2]
        if child == -1:
            continue
        (icx, icy), r_in = cv2.minEnclosingCircle(contours[child])
        if r_in <= 0 or r_in >= r_out:
            continue
        if math.hypot(icx - cx, icy - cy) > 0.12 * r_out:
            continue

        width_ratio = (r_out - r_in) / r_out
        if not (0.12 <= width_ratio <= 0.55):
            continue

        peri = cv2.arcLength(contour, True)
        circularity = 4.0 * math.pi * area / (peri * peri) if peri > 0 else 0.0
        candidates.append({
            "center": (float(cx), float(cy)),
            "r_out": float(r_out),
            "r_in": float(r_in),
            "width_ratio": float(width_ratio),
            "circularity": float(circularity),
            "area": float(area),
        })

    if len(candidates) < 2:
        return None

    r = spec.radii_mm
    expected_outer_ratio = r[0] / r[2]
    expected_gap_ratio = r[1] / r[2]
    expected_inner_ratio = r[2] / r[3]
    expected_outer_width_ratio = (r[0] - r[1]) / r[0]
    expected_inner_width_ratio = (r[2] - r[3]) / r[2]

    best = None
    for a in candidates:
        for b in candidates:
            if a is b:
                continue
            if a["r_out"] >= b["r_out"]:
                outer, inner = a, b
            else:
                outer, inner = b, a

            ratio_outer_inner = outer["r_out"] / inner["r_out"]
            if abs(ratio_outer_inner - expected_outer_ratio) > 0.15:
                continue
            if abs(outer["width_ratio"] - expected_outer_width_ratio) > 0.08:
                continue
            if abs(inner["width_ratio"] - expected_inner_width_ratio) > 0.10:
                continue
            if abs(outer["r_in"] / inner["r_out"] - expected_gap_ratio) > 0.12:
                continue
            if abs(inner["r_out"] / inner["r_in"] - expected_inner_ratio) > 0.15:
                continue

            center_dist = math.hypot(
                outer["center"][0] - inner["center"][0],
                outer["center"][1] - inner["center"][1])
            if center_dist > 0.12 * outer["r_out"]:
                continue

            score = (abs(ratio_outer_inner - expected_outer_ratio)
                     + abs(outer["width_ratio"] - expected_outer_width_ratio)
                     + abs(inner["width_ratio"] - expected_inner_width_ratio)
                     + center_dist / outer["r_out"])
            if best is None or score < best["score"]:
                best = {"outer": outer, "inner": inner, "score": score}

    if best is None:
        return None

    outer = best["outer"]
    inner = best["inner"]
    center = (
        (outer["center"][0] + inner["center"][0]) / 2.0,
        (outer["center"][1] + inner["center"][1]) / 2.0,
    )
    radii = [outer["r_out"], outer["r_in"], inner["r_out"], inner["r_in"]]
    return center, radii, outer, inner


def estimate_pose(corners: np.ndarray, ring_center_warp: Tuple[float, float],
                  spec: RingSpec,
                  camera_matrix: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Estimate the A4 plane pose and the ring centre in camera coordinates."""
    object_points = np.array([
        [0.0, 0.0, 0.0],
        [spec.a4_w_mm, 0.0, 0.0],
        [spec.a4_w_mm, spec.a4_h_mm, 0.0],
        [0.0, spec.a4_h_mm, 0.0],
    ], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1), dtype=np.float32)

    ok, rvec, tvec = cv2.solvePnP(
        object_points, corners.astype(np.float32),
        camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        raise RuntimeError("solvePnP failed")

    R, _ = cv2.Rodrigues(rvec)
    ring_mm = np.array([
        ring_center_warp[0] / spec.pixels_per_mm,
        ring_center_warp[1] / spec.pixels_per_mm,
        0.0,
    ], dtype=np.float32)
    center_camera_mm = R @ ring_mm + tvec.ravel()
    return rvec, tvec, center_camera_mm


def _default_camera_matrix(image: np.ndarray) -> np.ndarray:
    h, w = image.shape[:2]
    f = max(w, h) * 1.1
    return np.array([
        [f, 0, w / 2.0],
        [0, f, h / 2.0],
        [0, 0, 1.0],
    ], dtype=np.float32)


def detect(image: np.ndarray, spec: RingSpec,
           camera_matrix: Optional[np.ndarray] = None) -> Detection:
    # 1) Preferred: A4 sheet + solvePnP (full 6-DoF pose).
    corners = find_a4(image, spec)
    if corners is not None:
        warp, M = warp_a4(image, corners, spec)
        rings = detect_rings(warp, spec)
        if rings is not None:
            center, radii, _, _ = rings
            if camera_matrix is None:
                camera_matrix = _default_camera_matrix(image)

            try:
                rvec, tvec, center_camera_mm = estimate_pose(
                    corners, center, spec, camera_matrix)
            except Exception as exc:
                return Detection(False, f"pose_error: {exc}",
                                 method="a4_pnp",
                                 a4_corners=corners, center_warp=center,
                                 radii_px=radii, a4_warp=warp)

            annotated = image.copy()
            cv2.polylines(annotated, [corners.astype(np.int32)], True,
                          (0, 255, 0), 3)
            center_original = cv2.perspectiveTransform(
                np.array([[center]], dtype=np.float32),
                np.linalg.inv(M))[0][0]
            cv2.circle(
                annotated,
                (int(round(center_original[0])), int(round(center_original[1]))),
                8, (0, 0, 255), -1)

            bx, by, bw, bh = cv2.boundingRect(corners.astype(np.int32))
            bbox = (int(bx), int(by), int(bw), int(bh))
            cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh),
                          (0, 255, 0), 2)
            cv2.putText(annotated, "TARGET", (bx, max(by - 8, 18)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            distance_m = float(np.linalg.norm(center_camera_mm) / 1000.0)
            text = (f"A4+PnP  r=[{radii[0]:.1f},{radii[1]:.1f},"
                    f"{radii[2]:.1f},{radii[3]:.1f}]px  d={distance_m:.2f}m")
            cv2.putText(annotated, text, (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            return Detection(
                True, "ok", method="a4_pnp",
                a4_corners=corners,
                center_px=(float(center_original[0]), float(center_original[1])),
                center_warp=center,
                radii_px=radii,
                bbox_px=bbox,
                center_camera_mm=center_camera_mm,
                distance_m=distance_m,
                rvec=rvec,
                tvec=tvec,
                a4_warp=warp,
                annotated=annotated,
            )

    # 2) Fallback: detect the two concentric rings directly.  This works even
    #    when the white A4 sheet merges with a white desk/keyboard.
    direct = detect_rings_direct(image, spec)
    if direct is None:
        return Detection(False, "target_not_found", a4_corners=corners)

    center, radii, outer, _ = direct
    if camera_matrix is None:
        camera_matrix = _default_camera_matrix(image)

    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])
    if fx <= 0 or fy <= 0:
        return Detection(False, "invalid_camera_matrix",
                         method="direct", center_px=center, radii_px=radii)

    # For a tilted planar circle the major axis (minEnclosingCircle radius)
    # is the true radius; the minor axis is foreshortened.
    z_mm = fx * (spec.outer_outer_d_mm / 2.0) / outer["r_out"]
    x_mm = (center[0] - cx) * z_mm / fx
    y_mm = (center[1] - cy) * z_mm / fy
    center_camera_mm = np.array([x_mm, y_mm, z_mm], dtype=np.float32)
    distance_m = float(z_mm / 1000.0)

    annotated = image.copy()
    c_pt = (int(round(center[0])), int(round(center[1])))
    for r, color in ((radii[0], (0, 255, 0)), (radii[1], (0, 255, 0)),
                     (radii[2], (0, 255, 255)), (radii[3], (0, 255, 255))):
        cv2.circle(annotated, c_pt, int(round(r)), color, 2)
    cv2.circle(annotated, c_pt, 7, (0, 0, 255), -1)

    r_outer = int(round(radii[0]))
    x1 = max(0, c_pt[0] - r_outer)
    y1 = max(0, c_pt[1] - r_outer)
    x2 = min(image.shape[1] - 1, c_pt[0] + r_outer)
    y2 = min(image.shape[0] - 1, c_pt[1] + r_outer)
    bbox = (x1, y1, x2 - x1, y2 - y1)
    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(annotated, "TARGET", (x1, max(y1 - 8, 18)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    text = (f"direct  r=[{radii[0]:.1f},{radii[1]:.1f},"
            f"{radii[2]:.1f},{radii[3]:.1f}]px  d={distance_m:.2f}m")
    cv2.putText(annotated, text, (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    return Detection(
        True, "ok", method="direct",
        center_px=center,
        radii_px=radii,
        bbox_px=bbox,
        center_camera_mm=center_camera_mm,
        distance_m=distance_m,
        annotated=annotated,
    )


def draw_target_texture(spec: RingSpec) -> np.ndarray:
    """Create a fronto-parallel A4 texture with the two black rings."""
    w = int(round(spec.a4_w_mm * spec.pixels_per_mm))
    h = int(round(spec.a4_h_mm * spec.pixels_per_mm))
    texture = np.full((h, w, 3), 255, dtype=np.uint8)
    center = (w // 2, h // 2)
    r = [int(round(v * spec.pixels_per_mm)) for v in spec.radii_mm]
    cv2.circle(texture, center, r[0], (0, 0, 0), -1)
    cv2.circle(texture, center, r[1], (255, 255, 255), -1)
    cv2.circle(texture, center, r[2], (0, 0, 0), -1)
    cv2.circle(texture, center, r[3], (255, 255, 255), -1)
    return texture


def make_synthetic_scene(spec: RingSpec, distance_m: float = 1.0,
                         yaw_deg: float = 8.0,
                         pitch_deg: float = 4.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Render the A4 target at a known camera distance for offline testing."""
    scene_w, scene_h = 1280, 720
    camera_matrix = np.array([
        [800.0, 0.0, scene_w / 2.0],
        [0.0, 800.0, scene_h / 2.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float32)

    object_points = np.array([
        [-spec.a4_w_mm / 2.0, -spec.a4_h_mm / 2.0, 0.0],
        [spec.a4_w_mm / 2.0, -spec.a4_h_mm / 2.0, 0.0],
        [spec.a4_w_mm / 2.0, spec.a4_h_mm / 2.0, 0.0],
        [-spec.a4_w_mm / 2.0, spec.a4_h_mm / 2.0, 0.0],
    ], dtype=np.float32)
    rvec = np.array([
        math.radians(pitch_deg),
        math.radians(yaw_deg),
        0.0,
    ], dtype=np.float32)
    tvec = np.array([0.0, 0.0, distance_m * 1000.0], dtype=np.float32)

    image_points, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, None)
    image_points = order_corners(image_points.reshape(4, 2))

    texture = draw_target_texture(spec)
    th, tw = texture.shape[:2]
    src = np.array([[0, 0], [tw - 1, 0], [tw - 1, th - 1], [0, th - 1]],
                   dtype=np.float32)
    M = cv2.getPerspectiveTransform(src, image_points)
    warped = cv2.warpPerspective(texture, M, (scene_w, scene_h))
    mask = cv2.warpPerspective(
        np.full((th, tw), 255, dtype=np.uint8), M, (scene_w, scene_h))

    scene = np.full((scene_h, scene_w, 3), 120, dtype=np.uint8)
    scene[mask > 0] = warped[mask > 0]
    scene = cv2.GaussianBlur(scene, (3, 3), 0)
    noise = np.random.normal(0, 2.0, scene.shape).astype(np.int16)
    scene = np.clip(scene.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    return scene, image_points, camera_matrix


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", help="Input image; omit to use a synthetic scene")
    parser.add_argument("--output", default="/tmp/ring_detector_annotated.png")
    parser.add_argument("--warp-output", default="/tmp/ring_detector_warp.png")
    args = parser.parse_args()

    spec = RingSpec()
    spec.validate()

    if args.image:
        image = cv2.imread(args.image)
        if image is None:
            print(f"ERROR: cannot read {args.image}")
            return 2
        camera_matrix = None
        truth = None
    else:
        image, truth, camera_matrix = make_synthetic_scene(spec)
        print("Synthetic scene: true A4 corners =",
              truth.astype(int).tolist())
        print("Synthetic scene: true distance = 1.00 m")

    result = detect(image, spec, camera_matrix)
    print("valid:", result.valid)
    print("message:", result.message)
    print("method:", result.method)
    if result.a4_corners is not None:
        print("A4 corners:", result.a4_corners.astype(int).tolist())
    if result.center_px is not None:
        print("ring center (image px):",
              tuple(round(v, 2) for v in result.center_px))
    if result.center_warp is not None:
        print("ring center (warp px):", tuple(round(v, 2) for v in result.center_warp))
    if result.radii_px is not None:
        print("radii (px):", [round(v, 2) for v in result.radii_px])
    if result.center_camera_mm is not None:
        print("ring center (camera mm):", np.round(result.center_camera_mm, 1).tolist())
    if result.distance_m is not None:
        print("distance (m): %.3f" % result.distance_m)

    if result.annotated is not None:
        cv2.imwrite(args.output, result.annotated)
        print("annotated:", args.output)
    if result.a4_warp is not None:
        cv2.imwrite(args.warp_output, result.a4_warp)
        print("warp:", args.warp_output)

    return 0 if result.valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
