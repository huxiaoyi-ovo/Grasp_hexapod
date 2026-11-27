"""CPU-only numerical helpers, independent of ROS and torch."""
import cv2
import numpy as np


class Letterbox:
    def __init__(self, size):
        self.size = int(size)
        self.canvas = np.empty((size, size, 3), np.uint8)
        self.tensor = np.empty((1, 3, size, size), np.float32)

    def prepare(self, rgb):
        """Letterbox an RGB image into the reusable CHW tensor; input must be RGB."""
        h, w = rgb.shape[:2]
        ratio = min(self.size / w, self.size / h)
        nw, nh = round(w * ratio), round(h * ratio)
        left, top = (self.size - nw) // 2, (self.size - nh) // 2
        self.canvas.fill(114)
        cv2.resize(rgb, (nw, nh), dst=self.canvas[top:top+nh, left:left+nw])
        # RGB / CHW into a reusable input tensor; channels are already RGB.
        np.multiply(self.canvas.transpose(2, 0, 1), 1.0/255,
                    out=self.tensor[0], casting='unsafe')
        return self.tensor, (nw / w, nh / h, left, top, w, h)


def best_box(output, mapping, confidence):
    """Single-class YOLOv8/11 raw head [1,5,N] or [1,N,5], xywh pixels.

    With max_det=1, the highest score is exactly the first greedy NMS result;
    computing the rest of NMS is unnecessary. Reject COCO/end-to-end exports.
    """
    raw = np.asarray(output)
    if raw.ndim != 3 or raw.shape[0] != 1:
        raise ValueError('Expected raw YOLO output [1,5,N] / [1,N,5]')
    rows = raw[0].T if raw.shape[1] == 5 else raw[0]
    if rows.ndim != 2 or rows.shape[1] != 5:
        raise ValueError('Model must have ONE class (xiaolan), raw xywh+score, nms=False')
    valid = np.isfinite(rows).all(axis=1) & (rows[:, 4] >= confidence)
    valid &= (rows[:, 4] <= 1) & (rows[:, 2] > 0) & (rows[:, 3] > 0)
    ids = np.flatnonzero(valid)
    sx, sy, left, top, width, height = mapping
    for idx in ids[np.argsort(-rows[ids, 4])]:
        cx, cy, bw, bh, score = rows[idx]
        x1 = np.clip((cx-bw/2-left)/sx, 0, width)
        y1 = np.clip((cy-bh/2-top)/sy, 0, height)
        x2 = np.clip((cx+bw/2-left)/sx, 0, width)
        y2 = np.clip((cy+bh/2-top)/sy, 0, height)
        x1, y1 = int(np.floor(x1)), int(np.floor(y1))
        x2, y2 = int(np.ceil(x2)), int(np.ceil(y2))
        if x2 > x1 and y2 > y1:
            return x1, y1, x2-x1, y2-y1, float(score)
    return None


def depth_median_sparse(depth, encoding, k_depth, k_color, r_cd, t_cd, box, center,
                        roi_size=9, min_valid=5, min_z=.2, max_z=1., diagnostics=None):
    """Depth at a colour-image ROI from an UNALIGNED depth image (sparse D2C).

    Same result as depth_median() on a driver-aligned image, without aligning
    the whole frame: only the depth pixels that can land in the colour ROI
    (a window bounded by the depth-colour parallax at min_z) are back-projected
    with the depth intrinsics, moved into the colour frame (p_c = r_cd @ p_d +
    t_cd) and projected with the colour intrinsics.  Returns the median colour-
    frame z of the points inside the ROI (clipped to the bbox) or None.
    k_depth / k_color: 3x3 intrinsics; box/center in colour pixels.
    """
    if encoding not in ('16UC1', '32FC1') or depth.ndim != 2:
        raise ValueError('Expected single-channel 16UC1 mm or 32FC1 metres')
    scale = .001 if encoding == '16UC1' else 1.
    k_depth, k_color = np.asarray(k_depth, float).reshape(3, 3), np.asarray(k_color, float).reshape(3, 3)
    r_cd, t_cd = np.asarray(r_cd, float).reshape(3, 3), np.asarray(t_cd, float).reshape(3)
    x, y, bw, bh = box
    u, v = center
    half = roi_size // 2
    ru1, rv1 = max(x, u-half), max(y, v-half)
    ru2, rv2 = min(x+bw, u+half+1), min(y+bh, v+half+1)
    # Window in the depth image: the ROI centre seen at infinity (rotation only),
    # widened by the largest parallax (at min_z) plus the ROI size.
    ray = np.linalg.solve(k_color, [u, v, 1.])
    at_inf = k_depth @ (r_cd.T @ ray)
    ud, vd = at_inf[0]/at_inf[2], at_inf[1]/at_inf[2]
    margin = int(np.ceil(k_depth[0, 0]*np.linalg.norm(t_cd)/max(min_z, 1e-3))) + roi_size
    h, w = depth.shape
    c1, c2 = max(0, int(ud)-margin), min(w, int(ud)+margin+1)
    r1, r2 = max(0, int(vd)-margin), min(h, int(vd)+margin+1)
    values = np.empty(0, np.float32)
    positive_count = roi_pixels = 0
    if c2 > c1 and r2 > r1:
        win = depth[r1:r2, c1:c2].astype(np.float32) * scale
        rows, cols = np.nonzero(np.isfinite(win) & (win > 0))
        zd = win[rows, cols]
        xd = (cols + c1 - k_depth[0, 2]) * zd / k_depth[0, 0]
        yd = (rows + r1 - k_depth[1, 2]) * zd / k_depth[1, 1]
        pc = r_cd @ np.vstack((xd, yd, zd)) + t_cd[:, None]
        front = pc[2] > 1e-6
        pc = pc[:, front]
        uc = k_color[0, 0]*pc[0]/pc[2] + k_color[0, 2]
        vc = k_color[1, 1]*pc[1]/pc[2] + k_color[1, 2]
        inside = (uc >= ru1-.5) & (uc < ru2-.5) & (vc >= rv1-.5) & (vc < rv2-.5)
        values = pc[2, inside].astype(np.float32)
        positive_count = roi_pixels = int(values.size)
    in_range = values[(values > min_z) & (values < max_z)]
    if diagnostics is not None:
        diagnostics.update(encoding=encoding, roi_pixels=roi_pixels, positive_pixels=positive_count,
            min_depth_m=min_z, max_depth_m=max_z,
            observed_median_m=float(np.median(values)) if values.size else None,
            below_range_pixels=int(np.count_nonzero(values <= min_z)),
            above_range_pixels=int(np.count_nonzero(values >= max_z)),
            valid_pixels=int(in_range.size), required_pixels=min_valid, sparse_align=True,
            reason=('MEASURED' if in_range.size >= min_valid else
                    'NO_VALID_DEPTH' if not values.size else
                    'DEPTH_OUT_OF_RANGE' if not in_range.size else 'INSUFFICIENT_DEPTH_PIXELS'))
    return float(np.median(in_range)) if in_range.size >= min_valid else None


def depth_median(depth, encoding, box, center, roi_size=9,
                 min_valid=5, min_z=.2, max_z=1., diagnostics=None):
    """Use only the central ROI clipped to bbox; ignore zero/NaN/out-of-range."""
    if encoding not in ('16UC1', '32FC1') or depth.ndim != 2:
        raise ValueError('Expected single-channel 16UC1 mm or 32FC1 metres')
    scale = .001 if encoding == '16UC1' else 1.
    h, w = depth.shape
    x, y, bw, bh = box
    u, v = center
    if not (0 <= x < x+bw <= w and 0 <= y < y+bh <= h and
            x <= u < x+bw and y <= v < y+bh):
        raise ValueError('RGB bbox outside aligned depth')
    half = roi_size // 2
    x1, y1, x2, y2 = u-half, v-half, u+half+1, v+half+1
    crop = depth[max(y,y1):min(y+bh,y2), max(x,x1):min(x+bw,x2)]
    values = crop.astype(np.float32).ravel() * scale
    positive = values[np.isfinite(values) & (values > 0)]
    if diagnostics is not None:
        diagnostics.update(encoding=encoding, roi_pixels=int(values.size),
            positive_pixels=int(positive.size), min_depth_m=min_z, max_depth_m=max_z,
            observed_median_m=float(np.median(positive)) if positive.size else None,
            below_range_pixels=int(np.count_nonzero(positive <= min_z)),
            above_range_pixels=int(np.count_nonzero(positive >= max_z)))
    values = values[np.isfinite(values)]
    values = values[(values > min_z) & (values < max_z)]
    if diagnostics is not None:
        diagnostics['valid_pixels'] = int(values.size)
        diagnostics['required_pixels'] = min_valid
        diagnostics['reason'] = ('MEASURED' if values.size >= min_valid else
            'NO_VALID_DEPTH' if not positive.size else
            'DEPTH_OUT_OF_RANGE' if not values.size else 'INSUFFICIENT_DEPTH_PIXELS')
    if values.size >= min_valid:
        return float(np.median(values))
    return None


class StableTarget:
    """Confirmation, jump gating, short loss hold, EMA. No ROS side effects."""
    def __init__(self, confirm=2, lost=3, max_x=.15, max_z=.15, alpha=.4):
        self.confirm, self.lost = confirm, lost
        self.max_x, self.max_z, self.alpha = max_x, max_z, alpha
        self.reset()

    def reset(self):
        self.value = self.raw = self.payload = None
        self.hits = self.misses = 0
        self.active = False

    def update(self, measurement, payload=None):
        if measurement is not None:
            x, z = measurement
            valid = np.isfinite([x, z]).all()
            if self.raw is not None:
                valid = valid and abs(x-self.raw[0]) <= self.max_x and abs(z-self.raw[1]) <= self.max_z
            if not valid:
                measurement = None
        if measurement is None:
            self.misses += 1
            if not self.active or self.misses > self.lost:
                self.reset()
                return None, None, 'NOT_FOUND'
            return self.value, self.payload, 'HELD'
        self.raw = np.array(measurement, dtype=float)
        self.value = self.raw.copy() if self.value is None else self.alpha*self.raw+(1-self.alpha)*self.value
        self.payload = payload
        self.misses = 0
        self.hits += 1
        self.active = self.hits >= self.confirm
        return (self.value, self.payload, 'VALID') if self.active else (None, None, 'CONFIRMING')
