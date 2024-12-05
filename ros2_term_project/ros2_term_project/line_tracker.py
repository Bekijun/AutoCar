import cv2
import numpy as np


class LineTracker:
    def __init__(self):
        self._delta = 0.0
        self.left_to_center_dist = None  # 왼쪽 차선과 중앙값 간의 거리
        self.right_to_center_dist = None  # 오른쪽 차선과 중앙값 간의 거리

    def process(self, img: np.ndarray) -> None:
        h, w, _ = img.shape

        # HSV 변환 및 흰색 차선 마스크 생성 (HSV 범위 확대)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 130])  # 흰색 감지 범위 - 하한값 조정 (기존 150에서 130으로)
        upper_white = np.array([180, 70, 255])  # 상한값 조정 (기존 60에서 70으로)
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # 관심 영역 (ROI) 설정 - 이미지 하단 1.5/3 영역만 사용
        roi_top = int(h * 1.3 / 3)
        mask[:roi_top, :] = 0

        # 양쪽을 각각 5%씩 잘라내고 중앙 90%만 사용
        left_crop = int(w * 0.05)  # 왼쪽 5% 잘라내기 (기존 10%에서 감소)
        right_crop = int(w * 0.95)  # 오른쪽 5% 잘라내기 (기존 10%에서 감소)
        mask[:, :left_crop] = 0  # 왼쪽 영역 자르기
        mask[:, right_crop:] = 0  # 오른쪽 영역 자르기

        # 열림 연산을 통해 작은 노이즈 제거
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Hough Line Transform을 사용하여 선 감지
        lines = cv2.HoughLinesP(mask, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=150)

        left_lines = []
        right_lines = []

        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    slope = (y2 - y1) / (x2 - x1 + 1e-6)  # 기울기 계산
                    if slope < -0.5:  # 왼쪽 차선
                        left_lines.append((x1, y1, x2, y2))
                    elif slope > 0.5:  # 오른쪽 차선
                        right_lines.append((x1, y1, x2, y2))

        # 차선 중심 계산
        left_center = np.mean([line[0] for line in left_lines] + [line[2] for line in left_lines]) if left_lines else None
        right_center = np.mean([line[0] for line in right_lines] + [line[2] for line in right_lines]) if right_lines else None

        lane_center = None

        if left_center is not None and right_center is not None:
            # 두 차선을 모두 인식할 때 중앙값 계산 및 거리 저장
            lane_center = (left_center + right_center) / 2
            self.left_to_center_dist = lane_center - left_center
            self.right_to_center_dist = right_center - lane_center
        elif left_center is not None and self.right_to_center_dist is not None:
            # 오른쪽 차선이 없을 때 왼쪽 차선과의 거리로 중앙값 예측
            lane_center = left_center + self.left_to_center_dist
        elif right_center is not None and self.left_to_center_dist is not None:
            # 왼쪽 차선이 없을 때 오른쪽 차선과의 거리로 중앙값 예측
            lane_center = right_center - self.right_to_center_dist
        else:
            # 모든 차선이 인식되지 않을 때 중앙 유지
            lane_center = w / 2

        # 에러 계산 (차선 중심과 이미지의 수평 중앙과의 차이)
        self._delta = lane_center - (w / 2)

        # 시각화
        if left_center is not None:
            cv2.line(img, (int(left_center), h), (int(left_center), 0), (255, 0, 0), 2)
        if right_center is not None:
            cv2.line(img, (int(right_center), h), (int(right_center), 0), (0, 255, 0), 2)
        if lane_center is not None:
            cv2.circle(img, (int(lane_center), h // 2), 10, (0, 0, 255), -1)

        # 출력
        cv2.imshow("Camera View", img)
        cv2.imshow("Mask View", mask)
        cv2.waitKey(1)

    @property
    def delta(self):
        return self._delta

