import cv2
import numpy as np
import time
import utils


class Vision:
    def __init__(self, capture_channel=0, frame_width=300, frame_height=200):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.web_cam = cv2.VideoCapture(capture_channel)
        self.web_cam.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.web_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        self.frame = None

        self.threshold = 70

        self.roi_height_offset = 0

        self.roi_width = 300
        self.roi_height = 200
        time.sleep(0.5)

    def get_current_frame(self):
        _, self.frame = self.web_cam.read()
        return self.frame

    def get_roi(self, image):
        center_height = self.frame_height // 2
        center_width = self.frame_width // 2

        height_roi_start = center_height + self.roi_height_offset - self.roi_height // 2
        height_roi_end = center_height + self.roi_height_offset + self.roi_height // 2

        width_roi_start = center_width - self.roi_width // 2
        width_roi_end = center_width + self.roi_width // 2

        return image[height_roi_start: height_roi_end, width_roi_start: width_roi_end, :]

    @staticmethod
    def __find_angle_of_min_area_rectangle(min_area_rect):
        raw_angle = int(min_area_rect[2])
        if raw_angle < -45:
            raw_angle = 90 + raw_angle
        if min_area_rect[1][0] < min_area_rect[1][1] and raw_angle > 0:
            raw_angle = (90 - raw_angle) * -1
        if min_area_rect[1][0] > min_area_rect[1][1] and raw_angle < 0:
            raw_angle = 90 + raw_angle
        return raw_angle

    @staticmethod
    def __filter_image(image):
        kernel = np.ones((3, 3), np.uint8)
        image = cv2.erode(image, kernel, iterations=4)
        # image = cv2.dilate(image, kernel, iterations=7)
        return image

    @staticmethod
    def __convert_to_binary_image(image, threshold_range):
        # return cv2.inRange(image, threshold_range[0], threshold_range[1])
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        return thresh

    @staticmethod
    def __get_contour_centroid(contour):
        moments = cv2.moments(contour)
        return int(moments ['m10'] / moments['m00']), int(moments['m01'] / moments['m00'])

    @staticmethod
    def __get_contour_sides(contour):
        return len(cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True))

    @staticmethod
    def __draw_min_area_rect_box(image, min_area_rect):
        box = cv2.boxPoints(min_area_rect)
        box = np.int0(box)
        cv2.drawContours(image, [box], 0, (0, 0, 255), 2)
        cv2.circle(image, (int(min_area_rect[0][0]), int(min_area_rect[0][1])), 5, (0, 255, 255), 2)

    @staticmethod
    def __construct_text(self, *args):
        string = ""
        for arg in args:
            string += (str(arg) + " ")
        # print(string)
        return string 

    def __is_obstacle_visible(self, frame):
        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        return False
        params = self.__get_contour_params(frame, [(0, 0, 0), (20, 255, 255)])
        if len(params) > 0 and params[1] > 50:
            return True
        return False

    def process_frame(self, frame):
        path_params = self.__get_contour_params(frame, [(0, 0, 0), (50, 50, 50)])
        if len(path_params) > 0: 
            string = self.__construct_text("", path_params[0], path_params[1][0]*path_params[1][1], path_params[2])
            frame = cv2.putText(frame, string, (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
            path_params.append(self.__is_obstacle_visible(frame))
        cv2.imshow("ROI - VISIBLE - CONTOURS", frame)
        return path_params

    def what_do_i_see(self):
        i_saw = self.process_frame(self.get_current_frame())
        self.__check_breaking_condition()
        return i_saw

    def __get_contour_params(self, frame, threshold_range):
        image = self.get_roi(frame)
        path = self.__convert_to_binary_image(image, threshold_range)
        path = self.__filter_image(path)
        cv2.imshow("bw_img", path)
        contours, hierarchy = cv2.findContours(path.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        params = []
        if len(contours) > 0:
            main_contour = max(contours, key=cv2.contourArea)
            min_area_rect = cv2.minAreaRect(main_contour)
            rectangle_center = int(min_area_rect[0][0]), int(min_area_rect[0][1])
            rectangle_dimension = int(min_area_rect[1][0]), int(min_area_rect[1][1])
            params = [rectangle_center, rectangle_dimension, cv2.contourArea(main_contour)]
            self.__draw_min_area_rect_box(image, min_area_rect)

        # cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
        return params

    def mouse_rgb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # checks mouse left button down condition
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            colors_b = self.frame[y, x, 0]
            colors_g = self.frame[y, x, 1]
            colors_r = self.frame[y, x, 2]
            colors = self.frame[y, x]
            print("BRG Format: ", colors)
            print("HSV Format: ", hsv[y, x])
            print("GRAYSCALE: ", gray[y, x])
            print("Coordinates of pixel: X: ", x, "Y: ", y)

    def __check_breaking_condition(self):
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.web_cam.release()
            cv2.destroyAllWindows()
            exit()

    def start_seeing(self):
        frame = 0
        v = time.time()
        cv2.namedWindow('ROI - VISIBLE - CONTOURS')
        cv2.setMouseCallback('ROI - VISIBLE - CONTOURS', self.mouse_rgb)
        while True:
            frame += 1
            self.what_do_i_see()
            # cx, cy, s, area = self.get_main_contour_params()
            # utils.show("cx :" + str(cx) + " cy: " + str(cy) + " sides : " + str(s))
            # utils.show(chr(13))
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #    break

        utils.show("frame Rate: ", frame / (time.time() - v))
        self.web_cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    vision = Vision()
    vision.start_seeing()
