from cv2 import createTrackbar, getTrackbarPos, imshow, waitKey, destroyAllWindows, namedWindow, resizeWindow, setTrackbarMin, setTrackbarMax, putText, FONT_HERSHEY_PLAIN, LINE_AA
from numpy import concatenate, zeros, uint8
from camera import Camera


class Program:
    def __init__(self):
        self.cam = Camera()

        self.running = True

        self.file_name = "camera_calibration.txt"

        self.height = None

        self.window_name = "Image Calibration"
        self.trackbar_name = "Height"

        namedWindow(self.window_name)

        self.button_is_pressed = False

        self.button_height = 50
        
        self.frame_size = [0,0]

        self.empty_image_height = 15

        self.arrow_space = 100
        self.arrow_tip_size = 11
        self.arrow_thickness = 7

        self.old_h = 0

        self.frame = None
    
    def read_file(self):
        with open(self.file_name, 'r') as file:
            contents = file.read()
        try:
            return int(contents)
        except Exception:
            return 1

    def write_file(self):
        with open(self.file_name, 'w') as file:
            file.write(str(self.height))

    def main(self):
        while self.running:
            try:
                reading = self.cam.read()
            except Exception as exc:
                print(exc)
                continue
                

            # Use old frame when there is no new frame for greater performance
            if reading is None:
                pass
            else:
                if self.frame is None:
                    self.frame_size = [reading.shape[1], reading.shape[0]]

                    # Set trackbar max to image height when the first frame is read
                    self.starting_trackbar_value = self.read_file()
                    createTrackbar(self.trackbar_name, self.window_name, self.starting_trackbar_value, self.frame_size[1]-1, lambda _: None)
                    setTrackbarMin(self.trackbar_name, self.window_name, 1)
                    setTrackbarMax(self.trackbar_name, self.window_name, self.frame_size[1]-1)

                    # Create the empty image for the space between the two
                    empty_image = zeros((self.empty_image_height, self.frame_size[0], 3), uint8)
                    empty_image[:, :] = (255, 255, 255)

                    raw_button_image = zeros((self.button_height, self.arrow_space + self.frame_size[0], 3), uint8)

                    self.pressed_button_image, self.not_pressed_button_image = raw_button_image, raw_button_image

                    self.pressed_button_image[:, :] = (200, 200, 200)
                    self.not_pressed_button_image[:, :] = (150, 150, 150)

                    resizeWindow(self.window_name, self.frame_size[0] + self.arrow_space, 550)
                self.frame = reading
            
            # Gets the value of the trackbars
            self.height = getTrackbarPos(self.trackbar_name, self.window_name)

            if self.height != self.old_h:
                # Creates the croped image
                cropped_image = self.frame[self.height-1:self.height+1, :]

                arrow_image = zeros((2 + 3 * self.empty_image_height + self.frame_size[1], self.arrow_space, 3), uint8)
                arrow_image[:, :] = (255, 255, 255)

                h_on_images = 2 + 2 * self.empty_image_height + self.height
                arrow_image[h_on_images - self.arrow_thickness // 2:h_on_images + self.arrow_thickness // 2, :self.arrow_space - self.arrow_tip_size] = (0, 0, 0)
                arrow_image[h_on_images - self.arrow_thickness // 2 + 1:h_on_images + self.arrow_thickness // 2 - 1, :self.arrow_space - self.arrow_tip_size] = (0, 0, 255)

                for y in range(self.arrow_tip_size, -self.arrow_tip_size-1, -1):
                    try:
                        arrow_image[h_on_images - y, self.arrow_space - self.arrow_tip_size : self.arrow_space - abs(y)] = (0, 0, 255)
                    except IndexError:
                        pass
                
                sum = 0
                for pixel in self.frame[self.height, :]:
                    sum += pixel[0]
                avg = sum//self.frame_size[0]

                arrow_image = putText(arrow_image, f'Preview:', (10, 20), FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1, LINE_AA, False)

                offset = 35
                if avg > 9:
                    offset = 30
                    if avg > 99:
                        offset = 25
                
                if self.frame_size[1] - self.height < 40:
                    arrow_image = putText(arrow_image, f'Avg. Color:', (0, h_on_images-20), FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1, LINE_AA, False)
                    arrow_image = putText(arrow_image, f'({avg})', (offset, h_on_images-35), FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1, LINE_AA, False)
                else:
                    arrow_image = putText(arrow_image, f'Avg. Color:', (0, h_on_images+25), FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1, LINE_AA, False)
                    arrow_image = putText(arrow_image, f'({avg})', (offset, h_on_images+40), FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1, LINE_AA, False)

                self.old_h = self.height

            all_images = concatenate((empty_image, cropped_image, empty_image, self.frame, empty_image), axis=0)

            final_image = concatenate((arrow_image, all_images), axis=1)

            # if self.button_is_pressed:
            #     final_image = concatenate((all_images_arrow, self.pressed_button_image), axis=0)
            # else:
            #     final_image = concatenate((all_images_arrow, self.not_pressed_button_image), axis=0)
            
            # final_image = putText(final_image, f'Save', ((self.arrow_space + self.frame_size[0]) // 2 - 30, 3 * self.empty_image_height + 2 + self.frame_size[1] + self.button_height // 2+ 10), FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2, LINE_AA, False)

            # Displays the uncropped image for reference
            imshow(self.window_name, final_image)

            # Checks whether 'q' was pressed and if so closes the program
            key = chr(waitKey(1) & 255)
            if key == 'q':
                self.running = False
            elif key == 's':
                self.write_file()
                self.running = False

if __name__ == "__main__":
    program = Program()
    program.main()
    destroyAllWindows()