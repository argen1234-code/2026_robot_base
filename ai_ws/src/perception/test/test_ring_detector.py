#!/usr/bin/env python3
"""Synthetic regression test for the A4 two-ring detector."""

import math

from perception.ring_detector import RingSpec, detect, make_synthetic_scene


def main():
    spec = RingSpec()
    spec.validate()
    image, true_corners, camera_matrix = make_synthetic_scene(
        spec, distance_m=1.0, yaw_deg=8.0, pitch_deg=4.0)

    result = detect(image, spec, camera_matrix)
    print("valid:", result.valid, "message:", result.message)
    assert result.valid, result.message

    # Canonical A4 warp is 420x594 px at 2 px/mm; target is centred.
    assert result.center_warp is not None
    cx, cy = result.center_warp
    assert abs(cx - 210.0) < 6.0, (cx, cy)
    assert abs(cy - 297.0) < 6.0, (cx, cy)

    # 167/127/108/66 mm at 2 px/mm in the rectified A4 image.
    expected_radii = [167.0, 127.0, 108.0, 66.0]
    assert result.radii_px is not None
    for got, want in zip(result.radii_px, expected_radii):
        assert abs(got - want) / want < 0.10, (got, want)

    # The synthetic camera is 1.0 m away; PnP should recover it.
    assert result.distance_m is not None
    assert abs(result.distance_m - 1.0) < 0.20, result.distance_m

    print("center:", result.center_warp)
    print("radii:", result.radii_px)
    print("distance_m:", result.distance_m)
    print("SYNTHETIC TEST PASSED")


if __name__ == "__main__":
    main()
