"""Target ROI depth sampling shared by the detector and regression checks."""
import numpy as np


def bbox_depth(depth, bbox, color_size, offset=(0., 0.), minimum_m=.30, maximum_m=8.):
    height, width = depth.shape[:2]
    color_width, color_height = color_size
    x, y, bw, bh = bbox
    sx, sy = width/float(color_width), height/float(color_height)
    ox, oy = offset[0]*width, offset[1]*height
    x0, x1 = max(0, int((x+.2*bw)*sx+ox)), min(width, int((x+.8*bw)*sx+ox))
    y0, y1 = max(0, int((y+.2*bh)*sy+oy)), min(height, int((y+.8*bh)*sy+oy))
    if x1 <= x0 or y1 <= y0:
        return None, 0
    roi = depth[y0:y1, x0:x1].astype(np.float32)
    valid = roi[(roi > minimum_m*1000.) & (roi < maximum_m*1000.)]
    if valid.size < 10:
        return None, int(valid.size)
    return float(np.percentile(valid, 30.))/1000., int(valid.size)
