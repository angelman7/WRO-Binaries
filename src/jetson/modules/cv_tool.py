import cv2
import numpy as np
import time
from pathlib import Path
import os
from json import loads
from datetime import datetime


class CVTool:
    def __init__(self, use_cam=True, width=640, height=480, framerate=120, flip_method=0, rsp_cam=True, cnf_file="cnf.txt"):
        self.width = width
        self.height = height
        self.framerate = framerate
        self.flip_method = flip_method
        self.microbit = None
        self.cnf_file = os.path.dirname(__file__) + "/../" + cnf_file
        print(cnf_file)
        self.cropping = False
        self.get_rect = False
        self.x_start = 0
        self.y_start = 0
        self.x_end = 0
        self.y_end = 0
        self.focal_distance = 0.304
        self.px_to_cm = 0.0264583333
        self.rsp_cam = rsp_cam
        self.camera = None
        if use_cam:
            self.open_cam()

    def open_cam(self):
        if self.camera_is_open():
            return None
        if self.rsp_cam:
            streamer = ("nvarguscamerasrc ! "
                        "video/x-raw(memory:NVMM), "
                        "width=(int)%d, height=(int)%d, "
                        "format=(string)NV12, framerate=(fraction)%d/1 ! "
                        "nvvidconv flip-method=%d ! "
                        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                        "videoconvert ! "
                        "video/x-raw, format=(string)BGR ! appsink"
                        % (
                            self.width,
                            self.height,
                            self.framerate,
                            self.flip_method,
                            self.width,
                            self.height,
                        ))
            self.camera = cv2.VideoCapture(streamer, cv2.CAP_GSTREAMER)
        else:
            self.camera = cv2.VideoCapture(0)

    def camera_is_open(self):
        if self.camera is None:
            return False
        return self.camera.isOpened()

    def try_camera(self):
        self.open_cam()
        fps = 0
        frame_counter = 0
        start = time.time()
        while True:
            if self.camera_is_open():
                start_time = time.time()
                _, frame = self.camera.read()
                cv2.imshow("Try Camera", frame)
                frame_counter += 1
                end_time = time.time()
                fps = 1//(end_time - start_time)
                print("FPS :", fps)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    end = time.time()
                    break

        print("Average FPS: ", frame_counter / (end - start))

    def find_colors(self, img, hsv_lowers, hsv_uppers):
        '''
        Exctract object in image with hsv values
        :params
        -img: the original image
        -hsv_lower: list with the lower HSV values (e.g. [5, 120, 130])
        -hsv_upper: list with the upper HSV values (e.g. [15, 170, 255])
        Returns: the object mask, is any
        '''
        if len(hsv_lowers) != len(hsv_uppers):
            print("Different hsv list sizes lowers/uppers")
            return None
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        masks = []
        for i, hsv_lower in enumerate(hsv_lowers):
            lower = np.array(hsv_lower)
            upper = np.array(hsv_uppers[i])
            mask = cv2.inRange(imgHSV, lower, upper)
            masks.append(mask)
        final_mask = np.zeros((img.shape[0], img.shape[1], 1), np.uint8)
        for mask in masks:
            final_mask = cv2.bitwise_or(final_mask, mask)
        return mask

    def find_contours(self, img, mask, min_area=1000, sort=True, filter=0, draw=False):
        '''
        Exctract contours in image using a mask
        :params
        -img: the original image
        -mask: the image mask with the contours
        -min_area: the minimum contours' area, in pixels
        -sort: True (sorted list)/False (insorted list)
        -draw: True/False - if True, draws detected contours in returned image
        Returns: tuple (cont_img, countours), where:
                cont_img: the image with the drawn contours (if draw==True)
                contours: list with all detected contours (area, [x,y,w,h])
        '''
        conFound = []
        imgContours = img.copy()
        # if mask == None:
        #     mask = self.pre_processing(img)
        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > min_area:
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02*peri, True)
                # print(len(approx))
                if len(approx) == filter or filter == 0:
                    if draw:
                        cv2.drawContours(imgContours, cnt, -
                                         1, (255, 0, 255), 3)
                    x, y, w, h = cv2.boundingRect(approx)
                    cv2.rectangle(imgContours, (x, y),
                                  (x+w, y+h), (0, 255, 0), 2)
                    cv2.circle(imgContours, (x+(w//2), y+(h//2)),
                               5, (0, 255, 0), cv2.FILLED)
                    conFound.append([area, [x, y, w, h]])

        if sort:
            conFound = sorted(conFound, key=lambda x: x[0], reverse=True)

        return imgContours, conFound

    def calibrate_color(self, folder_path, color_name="demo"):
        '''
        Updates the cnf file for the lower and upper HSV values of the 
        color 'color_name', using the images from the folder 'folder_path' 
        :params
        -images: the list with the original images
        -color_name: the color name ['red', 'green', 'blue', 'black'] (use 'blue' as wild)
        Returns: True if cnf updated successfully, False otherwise
        '''
        images = []
        for filename in os.listdir(folder_path):
            img = cv2.imread(os.path.join(folder_path, filename))
            if img is not None:
                images.append(img)
        total_lower = [179, 255, 255]
        total_upper = [0, 0, 0]

        for i, image in enumerate(images):

            print("'c' to crop after mouse detection, 'esq' to pass image")
            crop = self.crop_color(image)
            if crop is None:
                continue
            lower = crop[0]
            upper = crop[1]
            self.initTrackbars(lower, upper)
            cancel = False
            print("Use the trackbars to improve values")
            print("'s' to keep values 'esq' to cancel values")
            while True:
                lower, upper = self.getTrackbarValues()
                mask = self.find_colors(image, [lower], [upper])
                img_contours, contours = self.find_contours(
                    image, mask, min_area=1000, sort=True, filter=0, draw=True)
                cv2.imshow("img_contours_" + str(i), img_contours)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('s'):
                    break
                if key == 27:
                    cancel = True
                    break
            cv2.destroyAllWindows()
            if cancel:
                continue
            total_lower[0] = min(total_lower[0], lower[0])
            total_lower[1] = min(total_lower[1], lower[1])
            total_lower[2] = min(total_lower[2], lower[2])
            total_upper[0] = max(total_upper[0], upper[0])
            total_upper[1] = max(total_upper[1], upper[1])
            total_upper[2] = max(total_upper[2], upper[2])
        print("Lower values of color " + color_name + ": " + str(total_lower))
        print("Upper values of color " + color_name + ": " + str(total_upper))
        if color_name != "demo":
            message = "if you want to update cnf.txt press 's', otherwise press 'p'"
            user_input = input(message)
            while user_input != 'p' and user_input != 'P' and user_input != 's' and user_input != 'S':
                user_input = input(message)
            if user_input == 's' or user_input == 'S':
                # save values to cnf file
                self.write_cnf_file(color_name, total_lower, total_upper)
                print("The cnf.txt file was modified successfully")
            else:
                print("The cnf.txt file has not been modified")

    def cnf_validation(self):
        data_names = ['red1', 'red2', 'green',
                      'blue_line', 'orange_line', 'wall']
        data = {}
        # Read all data from cnf
        for d in data_names:
            data[d] = self.read_cnf_file(d)
        for color_name1 in data:
            for color_name2 in data:
                if color_name1 == color_name2 or (color_name1 == "red1" and color_name2 == "red2") or (color_name1 == "red2" and color_name2 == "red1"):
                    continue
                conflicts = 0
                for i in range(3):
                    # if lower1 < lower2 and upper1 > lower2 we have conflict!
                    if data[color_name1][0][i] < data[color_name2][0][i] and data[color_name1][1][i] >= data[color_name2][0][i]:
                        # print("Conflict in colors " + color_name1 + " and " + color_name2 + " in hsv[" + str(i) + "]")
                        conflicts += 1
                    # if lower1 > lower2 and upper2 > lower1 we have conflict!
                    elif data[color_name1][0][i] > data[color_name2][0][i] and data[color_name2][1][i] >= data[color_name1][0][i]:
                        # print("Conflict in colors " + color_name1 + " and " + color_name2 + " in hsv[" + str(i) + "]")
                        conflicts += 1
                    # if lower1 == lower2 we have conflict!
                    elif data[color_name1][0][i] == data[color_name2][0][i]:
                        # print("Conflict in colors " + color_name1 + " and " + color_name2 + " same lower values!")
                        conflicts += 1
                if conflicts == 3:
                    print("Three conflicts in colors",
                          color_name1, "and", color_name2, "!!!")
                    return False
        print("No conflicts in cnf file!!!")
        return True

    def initTrackbars(self, hsv_lower, hsv_upper):
        """
        To intialize Trackbars . Need to run only once
        """
        cv2.namedWindow("TrackBars")
        cv2.resizeWindow("TrackBars", 640, 240)
        cv2.createTrackbar("Hue Min", "TrackBars",
                           hsv_lower[0], 179, self.empty)
        cv2.createTrackbar("Hue Max", "TrackBars",
                           hsv_upper[0], 179, self.empty)
        cv2.createTrackbar("Sat Min", "TrackBars",
                           hsv_lower[1], 255, self.empty)
        cv2.createTrackbar("Sat Max", "TrackBars",
                           hsv_upper[1], 255, self.empty)
        cv2.createTrackbar("Val Min", "TrackBars",
                           hsv_lower[2], 255, self.empty)
        cv2.createTrackbar("Val Max", "TrackBars",
                           hsv_upper[2], 255, self.empty)

    def getTrackbarValues(self):
        """
        Gets the trackbar values in runtime
        :return: hsv values from the trackbar window
        """
        hmin = cv2.getTrackbarPos("Hue Min", "TrackBars")
        smin = cv2.getTrackbarPos("Sat Min", "TrackBars")
        vmin = cv2.getTrackbarPos("Val Min", "TrackBars")
        hmax = cv2.getTrackbarPos("Hue Max", "TrackBars")
        smax = cv2.getTrackbarPos("Sat Max", "TrackBars")
        vmax = cv2.getTrackbarPos("Val Max", "TrackBars")

        return [[hmin, smin, vmin], [hmax, smax, vmax]]

    def empty(self, a):
        pass

    def click_and_crop(self, event, x, y, flags, param):
        # EVENT_LBUTTONDOWN = 1           Left click
        # EVENT_LBUTTONUP = 4             Left key release
        # EVENT_MOUSEMOVE = 0             slide
        if event == cv2.EVENT_LBUTTONDOWN:
            self.cropping = True
            self.x_start, self.y_start, self.x_end, self.y_end = x, y, x, y
        elif event == cv2.EVENT_MOUSEMOVE and self.cropping:
            self.x_end, self.y_end = x, y
        elif event == cv2.EVENT_LBUTTONUP:
            self.x_end, self.y_end = x, y
            self.cropping = False
            self.get_rect = True
        # print(event)

    def crop_color(self, image):

        #image = cv2.imread("resources/shapes.png")
        cv2.namedWindow("Image")
        cv2.setMouseCallback("Image", self.click_and_crop)
        while True:

            clone_image = image.copy()
            if self.cropping or self.get_rect:
                cv2.rectangle(clone_image, (self.x_start, self.y_start),
                              (self.x_end, self.y_end), (0, 255, 0), 2)

            cv2.imshow("Image", clone_image)
            key = cv2.waitKey(1)
            if key == 27 & 0xFF:
                return None
            if key == ord('c') & 0xFF:
                self.get_rect = False
                break

        if (self.x_start, self.y_start) != (self.x_end, self.y_end):
            rect = clone_image[self.y_start:self.y_end,
                               self.x_start:self.x_end]
            hsv_rect = cv2.cvtColor(rect, cv2.COLOR_BGR2HSV)
            lower = (hsv_rect[:, :, 0].min(), hsv_rect[:,
                     :, 1].min(), hsv_rect[:, :, 2].min())
            upper = (hsv_rect[:, :, 0].max(), hsv_rect[:,
                     :, 1].max(), hsv_rect[:, :, 2].max())

        return lower, upper

    def read_cnf_file(self, color_name):
        lower = [0, 0, 0]
        upper = [0, 0, 0]
        with open(self.cnf_file, 'r') as f:
            for line in f:
                # print(line)
                dict_line = loads(line)
                if dict_line["data"] == color_name:
                    lower[0] = dict_line['hmin']
                    lower[1] = dict_line['smin']
                    lower[2] = dict_line['vmin']
                    upper[0] = dict_line['hmax']
                    upper[1] = dict_line['smax']
                    upper[2] = dict_line['vmax']
                    return lower, upper
        return None

    def write_cnf_file(self, color_name, lower, upper):
        replacement = ""
        with open(self.cnf_file, 'r') as f:
            # using the for loop
            for line in f:
                dict_line = loads(line)
                if dict_line["data"] == color_name:
                    dict_line["hmin"] = lower[0]
                    dict_line["smin"] = lower[1]
                    dict_line["vmin"] = lower[2]
                    dict_line["hmax"] = upper[0]
                    dict_line["smax"] = upper[1]
                    dict_line["vmax"] = upper[2]
                replacement = replacement + str(dict_line) + "\n"
        time.sleep(0.1)
        # re-write the file with the new values
        with open(self.cnf_file, 'w') as f:
            f.write(replacement.replace("\'", "\""))

    def calibrate_menu(self):
        color_name = ""
        while color_name != "cancel":
            print("Available colors:")
            print("\tFor pillars: red1, red2, green")
            print("\tFor wall: wall")
            print("\tFor lines: blue_line, orange_line")
            print("\tFor demo purposes: demo")
            color_name = input("Type color name (or 'cancel'): ")
            if color_name == 'cancel':
                break
            cnf_hsv_range = self.read_cnf_file(color_name)
            if cnf_hsv_range is None and color_name != "demo":
                print("Color", color_name, "does not exists in 'cnf.txt' file.")
                resp = input(
                    "Type 'y' to contunue with other color, 'n' to exit: ")
                while resp not in ['n', 'y']:
                    resp = input("Wrong input (y/n): ")
                if resp == 'n':
                    break
                else:
                    continue
            all_directories = os.listdir()
            folder_name = input("Type folder name for color " + color_name + ": ")
            while folder_name not in all_directories:
                print("Directory " + folder_name + " does not exist")
                print("Available directories: " + str(all_directories))
                folder_name = input("Type folder name for color " + color_name + ": ")
            self.calibrate_color(folder_path=folder_name, color_name=color_name)

    def warp_image(self, image, top_crop=100, side_crop=10):
        width = image.shape[1]
        height = image.shape[0]
        points = np.float32([(side_crop, top_crop), (width - side_crop, top_crop),
                             (0, height-side_crop), (width - 0, height-side_crop)])
        pts1 = np.float32(points)
        pts2 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        image_warp = cv2.warpPerspective(image, matrix, (width, height))
        return image_warp

    def get_photos(self, folder_path):
        self.open_cam()
        # Create directory to save the images
        Path(folder_path).mkdir(parents=True, exist_ok=True)
        time.sleep(1)
        print("Press 's' to save image, 'q' to stop camera")
        counter = 0
        while True:
            if self.camera_is_open():
                _, img = self.camera.read()
                cv2.imshow("Image", img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                if key == ord('s'):
                    filename = folder_path + "/img_" + str(counter) + ".png"
                    cv2.imwrite(filename, img)
                    print("Image saved with name ", filename)
                    time.sleep(0.5)
                    counter += 1
        cv2.destroyAllWindows()

    def create_steering_folders(self, basic_folder="images"):
        # fix ranges to your model's needs ...
        steer_ranges = [(-100, -71), (-70, -51), (-50, -31), (-30, -11),
                        (-10, 9), (10, 29), (30, 49), (50, 69), (70, 100)]
        # Create basic dir
        # e.g. "images"
        Path(basic_folder).mkdir(parents=True, exist_ok=True)
        # Create txt file to write classes
        with open(basic_folder + '/labels.txt', 'w') as f:
            labels = []
            for r in steer_ranges:
                labels.append(str(r[0]) + "_" + str(r[1]))
            # Labels must be sorted, so the directories
            labels.sort()
            for label in labels:
                f.write(label + "\n")
        # Sub images names
        # e.g. "images/train", "images/val", "images/test"
        model_folders_dir = {"train": basic_folder + "/train",
                             "val": basic_folder + "/val",
                             "test": basic_folder + "/test"}
        # The full directory structure, returns from method
        # e.g. {"train/-50_-31":(-50, -31), ...}
        dir_structure = {}
        train_structure = {}
        val_structure = {}
        test_structure = {}
        # Create folders for every class
        for n, dir in model_folders_dir.items():
            # Create train, test or val sub directory
            Path(dir).mkdir(parents=True, exist_ok=True)
            for r in steer_ranges:
                # Build class directory name
                # e.g. "-50_-31" or "70_100"
                sub_dir_name = str(r[0]) + "_" + str(r[1])
                full_dir_name = dir + "/" + sub_dir_name
                Path(full_dir_name).mkdir(parents=True, exist_ok=True)
                if n == "train":
                    train_structure[full_dir_name] = r
                elif n == "val":
                    val_structure[full_dir_name] = r
                elif n == "test":
                    test_structure[full_dir_name] = r
        dir_structure["train"] = train_structure
        dir_structure["val"] = val_structure
        dir_structure["test"] = test_structure
        return dir_structure

    def get_steering_folder_name(self, steering, step=10):
        return step + steering // step

    def get_photos_remotely(self, folder_path=None):
        self.open_cam()

        if folder_path == None:
            now = datetime.now()
            dt_string = now.strftime("%d_%m_%Y_%H_%M")
            folder_path = "images_" + dt_string
        dir_structure = self.create_steering_folders(folder_path)
        print(dir_structure)
        time.sleep(1)
        counter = 0
        dir_number = 0
        while True:
            in_data = self.microbit.get_data()
            if self.camera_is_open():
                _, img = self.camera.read()
                # cv2.imshow("Image",img)
                key = cv2.waitKey(1) & 0xFF
                if in_data is not None and len(in_data) == 2 and in_data[0] == 7:
                    steering = in_data[1]
                    structure = dir_structure["train"]
                    if dir_number == 9 or dir_number == 8:
                        # 20% of images in val directory
                        structure = dir_structure["val"]
                    elif dir_number == 10:
                        # 10% of images in test directory
                        structure = dir_structure["test"]
                        dir_number = 0
                    dir_number += 1
                    for dir, steer_range in structure.items():
                        if steer_range[0] <= steering <= steer_range[1]:
                            filename = dir + "/img_" + str(counter) + ".png"
                            cv2.imwrite(filename, img)
                            print("Image saved with name ", filename)
                            counter += 1
                            break

                elif in_data is not None and len(in_data) > 0 and in_data[0] == 0:
                    break
        cv2.destroyAllWindows()

    def wrap_all_images(self, new_dir, img_dir):
        Path(new_dir).mkdir(parents=True, exist_ok=True)
        sub_dirs = ["train", "val", "test"]
        class_names = []
        with open(img_dir + '/labels.txt', 'r') as f:
            name = f.readline()
            name = name[:-1]  # remove \n
            if name is not None and name != "":
                class_names.append(name)
        with open(new_dir + '/labels.txt', 'w') as f:
            for name in class_names:
                f.write(name + '\n')
        for d in sub_dirs:
            # read original image
            or_sub_dir = img_dir + "\\" + d
            new_sub_dir = new_dir + "\\" + d
            Path(new_sub_dir).mkdir(parents=True, exist_ok=True)
            for c in class_names:
                or_class_dir = or_sub_dir + "\\" + c
                new_class_dir = new_sub_dir + "\\" + c
                Path(new_class_dir).mkdir(parents=True, exist_ok=True)
                for or_file in os.listdir(or_class_dir):
                    if or_file.endswith(".png"):
                        image = cv2.imread(or_class_dir + "\\" + or_file)
                        new_image = self.warp_image(
                            image, top_crop=100, side_crop=10)
                        cv2.imwrite(new_class_dir + "\\" + or_file, new_image)


def main():
    cvtool = CVTool(use_cam=False, rsp_cam=True)
    # cvtool.get_photos_remotely()

    cvtool.try_camera()
    # cvtool.calibrate_menu()
    # cvtool.get_photos("images_4_7_22")
    # cvtool.cnf_validation()


if __name__ == "__main__":
    main()

