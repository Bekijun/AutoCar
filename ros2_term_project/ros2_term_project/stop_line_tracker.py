import cv2
import numpy as np


class StopLineTracker:
    def __init__(self):
        self.stop_detected = False

    def process(self, img: np.ndarray) -> None:
        h, w, _ = img.shape

        # HSV 변환 및 흰색 정지선 마스크 생성
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])  # 흰색 범위
        upper_white = np.array([180, 30, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # 관심 영역 (ROI) 설정 - 이미지 하단 영역만 사용
        roi_top = int(h * 2.0 / 3)
        mask[:roi_top, :] = 0

        # 열림 연산을 통해 작은 노이즈 제거
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Hough Line Transform을 사용하여 가로선 감지 (정지선)
        lines = cv2.HoughLinesP(mask, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=10)

        self.stop_detected = False
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1 + 1e-6)
                # 거의 수평에 가까운 선 (기울기 절대값이 작은 경우)
                if abs(slope) < 0.1:
                    self.stop_detected = True
                    break

        # 시각화
        cv2.imshow("Stop Mask View", mask)
        cv2.waitKey(1)

    def is_stop_detected(self) -> bool:
        return self.stop_detected

