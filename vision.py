import cv2
import numpy as np
import utils


class Vision:
    def __init__(self, capture_channel=0, frame_width=640, frame_height=480):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.web_cam = cv2.VideoCapture(capture_channel)
        self.web_cam.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.web_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

        self.threshold = 70

        self.roi_height_offset = 0

        self.roi_width = 500
        self.roi_height = 200

    def get_current_frame(self):
        _, frame = self.web_cam.read()
        return frame

    def get_roi(self, image):
        center_height = self.frame_height // 2
        center_width = self.frame_width // 2

        height_roi_start = center_height + self.roi_height_offset - self.roi_height // 2
        height_roi_end = center_height + self.roi_height_offset + self.roi_height // 2

        width_roi_start = center_width - self.roi_width // 2
        width_roi_end = center_width + self.roi_width // 2

        return image[height_roi_start: height_roi_end, width_roi_start: width_roi_end, :]

    def draw_contour(self, contours, roi, cx, cy):
        cv2.line(roi, (cx, 0), (cx, self.frame_height), (255, 0, 0), 1)
        cv2.line(roi, (0, cy), (self.frame_width, cy), (255, 0, 0), 1)
        cv2.drawContours(roi, contours, -1, (0, 255, 0), 4)

    def get_main_contour_params(self):
        # Capture and gray conversion
        frame = self.get_current_frame()
        roi = self.get_roi(frame)
        gray_image = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(gray_image, (5, 5), 0)
        ret, bw_img = cv2.threshold(blur, self.threshold, 255, cv2.THRESH_BINARY_INV)

        mask = cv2.erode(bw_img, None, iterations=3)
        mask = cv2.dilate(mask, None, iterations=3)

        _, contours, _ = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)
        cx = cy = sides_approx = None
        if len(contours) > 0:
            main_contour = max(contours, key=cv2.contourArea)
            sides_approx = cv2.approxPolyDP(main_contour, 0.01 * cv2.arcLength(main_contour, True), True)

            cx = int(cv2.moments(main_contour)['m10'] / cv2.moments(main_contour)['m00'])
            cy = int(cv2.moments(main_contour)['m01'] / cv2.moments(main_contour)['m00'])

            self.draw_contour(contours, roi, cx, cy)

        cv2.imshow('roi_frame', roi)
        return cx, cy, sides_approx

    def start_seeing(self):
        while True:
            cx, cy, s = self.get_main_contour_params()
            utils.show("cx :" + str(cx) + " cy: " + str(cy) + " sides : " + str(s))
            utils.show(chr(13))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.web_cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    vision = Vision()
    vision.start_seeing()
