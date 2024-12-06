import cv2
import numpy as np


class LineTracker:
    def __init__(self):
        self._delta = 0.0
        self.prev_center = None  # 이전 중앙값

    def process(self, img: np.ndarray) -> None:
        h, w, _ = img.shape

        # 흰색 차선 마스크 생성
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])
        msk = cv2.inRange(hsv, lower_white, upper_white)

        # 관심 영역 설정
        roi_top = int(h * 2.0 / 3)
        msk[:roi_top, :] = 0

        # 열림 연산으로 작은 노이즈 제거
        kernel = np.ones((5, 5), np.uint8)
        msk = cv2.morphologyEx(msk, cv2.MORPH_OPEN, kernel)

        # Hough Transform으로 차선 감지
        lines = cv2.HoughLinesP(msk, 1, np.pi / 180, threshold=80, minLineLength=50, maxLineGap=30)

        left_line_xs = []
        right_line_xs = []

        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    if x1 < w / 2 and x2 < w / 2:  # 왼쪽 차선
                        left_line_xs.extend([x1, x2])
                    elif x1 > w / 2 and x2 > w / 2:  # 오른쪽 차선
                        right_line_xs.extend([x1, x2])

        # 차선 중앙 계산
        left_cx = np.mean(left_line_xs) if left_line_xs else None
        right_cx = np.mean(right_line_xs) if right_line_xs else None
        lane_center = None

        if left_cx and right_cx:
            # 두 차선 모두 감지된 경우
            lane_center = (left_cx + right_cx) / 2
            self.prev_center = lane_center
        elif right_cx:
            # 오른쪽 차선만 감지된 경우 -> 왼쪽 차선을 대칭으로 생성
            left_cx = w - right_cx
            lane_center = (left_cx + right_cx) / 2
            self.prev_center = lane_center
        elif left_cx:
            # 왼쪽 차선만 감지된 경우 -> 오른쪽 차선을 대칭으로 생성
            right_cx = w - left_cx
            lane_center = (left_cx + right_cx) / 2
            self.prev_center = lane_center
        else:
            # 차선이 전혀 감지되지 않을 경우 이전 중앙값 유지
            lane_center = self.prev_center if self.prev_center else w / 2

        # 에러 계산
        self._delta = lane_center - w / 2

        # 시각화
        if left_cx:
            cv2.line(img, (int(left_cx), h), (int(left_cx), 0), (255, 0, 0), 5)  # 파란선 (왼쪽 차선)
        if right_cx:
            cv2.line(img, (int(right_cx), h), (int(right_cx), 0), (0, 255, 0), 5)  # 초록선 (오른쪽 차선)
        if lane_center:
            cv2.circle(img, (int(lane_center), h // 2), 20, (0, 0, 255), -1)  # 중앙값

        # 화면 출력
        cv2.imshow("Camera View", img)
        cv2.imshow("Mask View", msk)
        cv2.waitKey(1)

    @property
    def delta(self):
        return self._delta
