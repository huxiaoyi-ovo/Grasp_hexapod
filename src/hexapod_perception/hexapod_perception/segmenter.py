"""Ultralytics instance-segmentation adapter."""

import cv2
import numpy as np


def _largest_connected_component(mask):
    """Discard detached class fragments before using a mask for 3-D geometry."""
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        np.asarray(mask, np.uint8), connectivity=8)
    if count <= 1:
        return np.asarray(mask, bool)
    index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return labels == index


class YoloSegmenter:
    def __init__(self, model_path, classes, confidence=0.45, image_size=640,
                 device="cpu", erosion_pixels=2):
        from ultralytics import YOLO

        self.model = YOLO(model_path, task="segment")
        self.classes = tuple(classes)
        self.confidence = confidence
        self.image_size = image_size
        self.device = device
        self.erosion_pixels = erosion_pixels

    def __call__(self, rgb):
        height, width = rgb.shape[:2]
        # Ultralytics ndarray input convention is BGR.
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        result = self.model.predict(
            bgr, conf=self.confidence, imgsz=self.image_size,
            device=self.device, verbose=False)[0]
        masks = {name: np.zeros((height, width), bool) for name in self.classes}
        scores = {name: 0.0 for name in self.classes}
        if result.masks is None or result.boxes is None:
            return masks, scores
        names = result.names
        for raw_mask, class_id, score in zip(
                result.masks.data.cpu().numpy(),
                result.boxes.cls.cpu().numpy().astype(int),
                result.boxes.conf.cpu().numpy()):
            name = names[class_id]
            if name not in masks:
                continue
            resized = cv2.resize(
                raw_mask, (width, height),
                interpolation=cv2.INTER_NEAREST) > 0.5
            if name in ("front_feature", "top_green_feature"):
                masks[name] |= resized
                scores[name] = max(scores[name], float(score))
                continue
            # A frame can contain several predictions with the same class.
            # Merging them contaminates depth geometry with detached false
            # positives.  Each physical support class is unique in this task,
            # so retain only its highest-confidence instance.
            if float(score) <= scores[name]:
                continue
            masks[name] = resized
            scores[name] = float(score)
        # Remove mixed-depth boundary pixels and make support classes exclusive.
        if self.erosion_pixels > 0:
            size = 2 * self.erosion_pixels + 1
            kernel = np.ones((size, size), np.uint8)
            for name in masks:
                masks[name] = cv2.erode(masks[name].astype(np.uint8), kernel).astype(bool)
        for name in ("board", "platform_robot"):
            if name in masks:
                masks[name] = _largest_connected_component(masks[name])
        if "board" in masks and "platform_robot" in masks:
            masks["board"] &= ~masks["platform_robot"]
        return masks, scores


class FrontKeypointPredictor:
    """Return the midpoint of reliable front endpoints, or either endpoint."""

    def __init__(self, model_path, confidence=0.35, image_size=640, device="cpu"):
        from ultralytics import YOLO

        self.model = YOLO(model_path, task="pose")
        self.confidence = confidence
        self.image_size = image_size
        self.device = device
        self.last_valid_count = 0

    def __call__(self, rgb, region_mask=None):
        self.last_valid_count = 0
        height, width = rgb.shape[:2]
        x1, y1, x2, y2 = 0, 0, width, height
        if region_mask is not None and np.any(region_mask):
            ys, xs = np.nonzero(region_mask)
            margin_x = max(8, int((xs.max() - xs.min() + 1) * 0.30))
            margin_y = max(8, int((ys.max() - ys.min() + 1) * 0.30))
            x1, x2 = max(0, int(xs.min()) - margin_x), min(width, int(xs.max()) + 1 + margin_x)
            y1, y2 = max(0, int(ys.min()) - margin_y), min(height, int(ys.max()) + 1 + margin_y)
        crop = rgb[y1:y2, x1:x2]
        bgr = cv2.cvtColor(crop, cv2.COLOR_RGB2BGR)
        result = self.model.predict(
            bgr, conf=self.confidence, imgsz=self.image_size,
            device=self.device, verbose=False)[0]
        if result.boxes is None or result.keypoints is None or len(result.boxes) == 0:
            return None, 0.0
        box_scores = result.boxes.conf.cpu().numpy()
        index = int(np.argmax(box_scores))
        data = result.keypoints.data.cpu().numpy()[index, :2]
        xy = data[:, :2].astype(np.float64)
        if data.shape[1] >= 3:
            confidence = data[:, 2]
        else:
            confidence = np.full(2, float(box_scores[index]), np.float64)
        valid = (np.all(np.isfinite(xy), axis=1)
                 & np.isfinite(confidence)
                 & (confidence >= self.confidence)
                 & np.any(xy != 0.0, axis=1))
        self.last_valid_count = int(np.count_nonzero(valid))
        if self.last_valid_count == 0:
            return None, float(np.max(confidence)) if len(confidence) else 0.0
        score = min(float(box_scores[index]), float(np.min(confidence[valid])))
        xy = xy[valid]
        xy += np.array([x1, y1], np.float64)
        # Two points give the front-edge midpoint.  One point is still enough
        # to choose the sign of the independently estimated geometric axis.
        return np.mean(xy, axis=0), score


class SemanticFeaturePredictor:
    """Detect one or more small segmentation features and return their centers.

    The green-feature model is trained on a crop around Xiaolan, while the
    front-rectangle model is trained on the full image.  Keeping crop handling
    here guarantees that inference uses the same geometry as training.
    """

    def __init__(self, model_path, confidence=0.15, image_size=640,
                 device="cpu", crop_to_region=False, crop_margin=0.28):
        from ultralytics import YOLO

        self.model = YOLO(model_path, task="segment")
        self.confidence = confidence
        self.image_size = image_size
        self.device = device
        self.crop_to_region = crop_to_region
        self.crop_margin = crop_margin

    def __call__(self, rgb, region_mask=None, maximum=2):
        height, width = rgb.shape[:2]
        x1, y1, x2, y2 = 0, 0, width, height
        if (self.crop_to_region and region_mask is not None
                and np.any(region_mask)):
            ys, xs = np.nonzero(region_mask)
            scale = max(xs.max() - xs.min() + 1, ys.max() - ys.min() + 1)
            margin = max(18, int(self.crop_margin * scale))
            x1, x2 = max(0, int(xs.min()) - margin), min(width, int(xs.max()) + margin + 1)
            y1, y2 = max(0, int(ys.min()) - margin), min(height, int(ys.max()) + margin + 1)
        crop = rgb[y1:y2, x1:x2]
        if crop.size == 0:
            return []
        bgr = cv2.cvtColor(crop, cv2.COLOR_RGB2BGR)
        result = self.model.predict(
            bgr, conf=self.confidence, imgsz=self.image_size,
            device=self.device, verbose=False)[0]
        if result.masks is None or result.boxes is None:
            return []
        scores = result.boxes.conf.cpu().numpy()
        order = np.argsort(scores)[::-1][:maximum]
        detections = []
        crop_height, crop_width = crop.shape[:2]
        for index in order:
            raw_mask = result.masks.data.cpu().numpy()[index]
            mask = cv2.resize(
                raw_mask, (crop_width, crop_height),
                interpolation=cv2.INTER_NEAREST) > 0.5
            ys, xs = np.nonzero(mask)
            if len(xs) < 8:
                continue
            center = np.array(
                [np.mean(xs) + x1, np.mean(ys) + y1], np.float64)
            detections.append((center, float(scores[index])))
        return detections
