"""Serit tespit algoritmasi.

>>> ENTEGRASYON NOKTASI <<<
Obur projeden gelen mevcut serit takibi kodu bu sinifin icine tasinacak.
Sozlesme sabit kalmali:

    detect(frame_bgr) -> LaneResult(found, lateral_offset, angle_deg, confidence,
                                     end_of_line_hint, debug_mask)

  lateral_offset : -1..1  (seridin goruntu merkezine gore yatay konumu; + = sagda)
  angle_deg      : seridin dikeyden sapma acisi (+ = saga yatik) -90..90
  confidence     : 0..1
  end_of_line_hint: seridin ust ucu goruntunun icinde bitiyor (tahta sonu olabilir)

Asagidaki gerceklestirme calisan bir varsayilan: HSV esikleme + en buyuk
kontur + fitLine. Mevcut kod tasininca yalnizca detect() icini degistirin.
"""
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass
class LaneResult:
    found: bool
    lateral_offset: float = 0.0
    angle_deg: float = 0.0
    confidence: float = 0.0
    end_of_line_hint: bool = False
    debug_mask: Optional[np.ndarray] = None


class LaneDetector:
    def __init__(self,
                 hsv_lower=(0, 80, 80), hsv_upper=(15, 255, 255),
                 hsv_lower2=(165, 80, 80), hsv_upper2=(180, 255, 255),
                 roi_top=0.2, min_area_ratio=0.002):
        """Varsayilan esikler kirmizi serit icindir (H sarmasi icin cift aralik).

        Su altinda renkler sonar/derinlikle degisir: havuz gununde
        `config/auv_params.yaml` uzerinden yeniden ayarlanacak.
        """
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)
        self.hsv_lower2 = np.array(hsv_lower2, dtype=np.uint8)
        self.hsv_upper2 = np.array(hsv_upper2, dtype=np.uint8)
        self.roi_top = roi_top
        self.min_area_ratio = min_area_ratio

    def set_thresholds(self, lower, upper, lower2=None, upper2=None):
        self.hsv_lower = np.array(lower, dtype=np.uint8)
        self.hsv_upper = np.array(upper, dtype=np.uint8)
        if lower2 is not None:
            self.hsv_lower2 = np.array(lower2, dtype=np.uint8)
        if upper2 is not None:
            self.hsv_upper2 = np.array(upper2, dtype=np.uint8)

    def detect(self, frame_bgr: np.ndarray) -> LaneResult:
        h, w = frame_bgr.shape[:2]
        roi_y0 = int(h * self.roi_top)
        roi = frame_bgr[roi_y0:, :]

        blur = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask |= cv2.inRange(hsv, self.hsv_lower2, self.hsv_upper2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return LaneResult(found=False, debug_mask=mask)

        biggest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest)
        if area < self.min_area_ratio * (roi.shape[0] * roi.shape[1]):
            return LaneResult(found=False, debug_mask=mask)

        # serit ekseni: fitLine
        vx, vy, x0, y0 = cv2.fitLine(biggest, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
        angle = float(np.degrees(np.arctan2(vx, -vy)))  # dikeyden sapma, + = saga
        if angle > 90:
            angle -= 180
        elif angle < -90:
            angle += 180

        m = cv2.moments(biggest)
        cx = m['m10'] / m['m00']
        lateral = float((cx - w / 2.0) / (w / 2.0))

        # tahta sonu ipucu: konturun ust ucu goruntunun ust kenarina degmiyor
        top_y = biggest[:, :, 1].min()
        end_hint = bool(top_y > roi.shape[0] * 0.25)

        conf = float(min(1.0, area / (0.05 * roi.shape[0] * roi.shape[1])))
        return LaneResult(found=True, lateral_offset=lateral, angle_deg=angle,
                          confidence=conf, end_of_line_hint=end_hint, debug_mask=mask)
