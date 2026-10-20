"""Appearance signature for re-acquisition: an HSV colour histogram of the
torso region of a person box. Needs OpenCV and NumPy; nothing is trained.

Why the torso: it is the largest clothing area visible from behind, from the
side and from above, and it changes less than legs or heads as people walk.
Saturation-weighted hue bins describe clothing colour while staying fairly
stable under lighting changes; a few value bins separate black, grey and white.
"""

import cv2
import numpy as np

H_BINS, S_BINS, V_BINS = 12, 3, 3


def torso_region(box, frame_shape, top=0.2, bottom=0.6, side=0.2):
    """Return (x1, y1, x2, y2) of the torso band inside a person box."""
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    fx1 = int(max(0, x1 + side * w))
    fx2 = int(min(frame_shape[1], x2 - side * w))
    fy1 = int(max(0, y1 + top * h))
    fy2 = int(min(frame_shape[0], y1 + bottom * h))
    return fx1, fy1, fx2, fy2


def torso_histogram(frame_bgr, box):
    """Normalised feature vector (tuple of floats), or None if the region is empty.

    Layout: H_BINS x S_BINS hue/saturation histogram of coloured pixels,
    then V_BINS value histogram of low-saturation (grey/black/white) pixels.
    """
    x1, y1, x2, y2 = torso_region(box, frame_bgr.shape)
    if x2 - x1 < 4 or y2 - y1 < 4:
        return None
    hsv = cv2.cvtColor(frame_bgr[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    coloured = s >= 50
    hs = cv2.calcHist([hsv], [0, 1], coloured.astype(np.uint8),
                      [H_BINS, S_BINS], [0, 180, 50, 256]).ravel()
    grey_v = np.histogram(v[~coloured], bins=V_BINS, range=(0, 256))[0].astype(np.float32)
    feat = np.concatenate([hs, grey_v])
    total = feat.sum()
    if total <= 0:
        return None
    return tuple(float(x) for x in feat / total)
