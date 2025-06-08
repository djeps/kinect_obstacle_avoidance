import freenect as kinect
import cv2
import pyautogui
import numpy as np

from open3d import geometry, utility, visualization


class Kinect:

    # ----------------
    # Screen constants
    # ----------------
    DEFAULT_SCREEN_WIDTH = 1804
    DEFAULT_SCREEN_HEIGHT = 976
    DEFAULT_WINDOW_WIDTH = 640
    DEFAULT_WINDOW_HEIGHT = 480

    # ----------------
    # Colors and modes
    # ----------------
    COLOR_BANDS = 6
    BAND_COLORS = 256
    RGB_ONLY = 1
    DEPTH_ONLY = 2
    RGB_AND_DEPTH = 3
    DISPLAY_MODES = {RGB_ONLY, DEPTH_ONLY, RGB_AND_DEPTH}

    # ----------------
    # Sensor constants
    # ---------------
    FOCAL_LENGTH_X = 594.21
    FOCAL_LENGTH_Y = 591.04
    OPTICAL_CENTER_X = 339.5
    OPTICAL_CENTER_Y = 242.7
    SENSOR_RESOLUTION = 2048
    MIN_VISIBLE_SENSOR_RANGE = 500
    MAX_VISIBLE_SENSOR_RANGE = SENSOR_RESOLUTION - 1


    def __init__(self, window_name: str, display_mode: int = 1):
        if display_mode not in Kinect.DISPLAY_MODES:
            raise ValueError("Incorrect display mode! Allowed modes: RGB_ONLY, DEPTH_ONLY, RGB_AND_DEPTH.")
    
        self.__display_mode__ = display_mode

        print("=> Press ESC to exit the application.")

        # --------------------------------
        # Setting OpenCV window properties
        # --------------------------------
        self.__window_name__ = window_name
        self.__window_width__ = Kinect.DEFAULT_WINDOW_WIDTH
        self.__window_height__ = Kinect.DEFAULT_WINDOW_HEIGHT

        if self.__display_mode__ == Kinect.RGB_AND_DEPTH:
            self.__window_width__ *= 2

        cv2.namedWindow(self.__window_name__, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.__window_name__, self.__window_width__, self.__window_height__)

        # ---------------------------------
        # Determining the screen resolution
        # ---------------------------------
        try:
            self.__screen_width__, self.__screen_height__ = pyautogui.size()
        except Exception as e:
            print(f"=> An unexpected error occurred when determining the screen resolution: {e}")
            
            self.__screen_width__ = Kinect.DEFAULT_SCREEN_WIDTH
            self.__screen_height__ = Kinect.DEFAULT_SCREEN_HEIGHT
            
            print(f"=> Going with default values for screen width = {self.__screen_width} and height = {self.__screen_height__}.")

        # --------------------------------------------------------------
        # Create the gamma color table to match 'freenect-glview' output
        # --------------------------------------------------------------
        # Gamma correction table
        self.__color_gamma__ = self.__create_gamma__()


    # Creates a lookup table and maps raw Kinect depth values (0–2047) to an RGB color
    # to produce a color gradient similar to what the 'freenect-glview' tool produces
    def __create_gamma__(self):
        
        # Create a matrix of 3x2048 to represent all raw data points from each color in RGB
        color_gamma = np.zeros((Kinect.SENSOR_RESOLUTION, 3), dtype=np.uint8)
        
        for i in range(Kinect.SENSOR_RESOLUTION):
            # Normalize and apply non-linear mapping
            v = (i / float(Kinect.SENSOR_RESOLUTION)) ** 3 * Kinect.COLOR_BANDS

            # Convert to a color band index (6 bands, each from 0 to 255)
            pval = int(v * Kinect.COLOR_BANDS * Kinect.BAND_COLORS)

            # Get low byte for smooth transitions
            lb = pval & 0xff

            # Get band index
            pval >>= 8
            
            # Assign colors based on band

            # White to magenta
            white = Kinect.BAND_COLORS - 1

            if pval == 0: color_gamma[i] = [white, white - lb, white - lb]
            # Magenta to red
            elif pval == 1: color_gamma[i] = [white, lb, 0]
            # Red to yellow
            elif pval == 2: color_gamma[i] = [white - lb, lb, 0]
            # Yellow to green
            elif pval == 3: color_gamma[i] = [white - lb, white, 0]
            # Green to cyan
            elif pval == 4: color_gamma[i] = [0, white - lb, white]
            # Cyan to blue
            elif pval == 5: color_gamma[i] = [0, 0, white - lb]
        
        return color_gamma


    def __get_depth_data__(self):
        depth_data_mm = None

        try:
            depth_data_mm = kinect.sync_get_depth()[0]
        except Exception as e:
            print(f"=> An unexpected error occurred when returning a Kinect depth frame: {e}")
        
        return depth_data_mm


    def __get_rgb_data__(self):
        rgb_data = None

        try:
            rgb_data = kinect.sync_get_video()[0]
        except Exception as e:
            print(f"=> An unexpected error occurred when returning a Kinect RGB frame: {e}")

        return rgb_data
    

    def __get_rgb_frame__(self):
        return self.__get_rgb_data__()


    def __get_depth_frame__(self):
        kinect_frame_color = None

        kinect_frame_raw = self.__get_depth_data__()

        if kinect_frame_raw is not None:
            # Clip to a valid range
            # Basically points within the visible range of the Kinect
            kinect_frame_clipped = np.clip(kinect_frame_raw, Kinect.MIN_VISIBLE_SENSOR_RANGE, Kinect.MAX_VISIBLE_SENSOR_RANGE)
            
            # Apply gamma correction to depth data
            kinect_frame_color = self.__color_gamma__[kinect_frame_clipped]

        return kinect_frame_color
    

    def __show_at__(self, cv2_frame, x_pos: int, y_pos: int):
        cv2.moveWindow(self.__window_name__, x_pos, y_pos)
        cv2.imshow(self.__window_name__, cv2_frame)


    def __show_centered__(self, cv2_frame):
        x_pos_center = (self.__screen_width__ - self.__window_width__) // 2
        y_pos_center = (self.__screen_height__ - self.__window_height__) // 2
        self.__show_at__(cv2_frame, x_pos_center, y_pos_center)


    def __get_cv2_frame__(self):
        cv2_frame = None

        if self.__display_mode__ == Kinect.RGB_ONLY:
            kinect_frame_plain = self.__get_rgb_frame__()

            if kinect_frame_plain is not None:
                # Convert from RGB to BGR for OpenCV
                cv2_frame = cv2.cvtColor(kinect_frame_plain, cv2.COLOR_RGB2BGR)

        elif self.__display_mode__ == Kinect.DEPTH_ONLY:
            kinect_frame_depth = self.__get_depth_frame__()

            if kinect_frame_depth is not None:
                # Convert from RGB to BGR for OpenCV
                cv2_frame = cv2.cvtColor(kinect_frame_depth, cv2.COLOR_RGB2BGR)

        elif self.__display_mode__ == Kinect.RGB_AND_DEPTH:
            kinect_frame_plain = self.__get_rgb_frame__()
            kinect_frame_depth = self.__get_depth_frame__()

            if (kinect_frame_plain is not None) and (kinect_frame_depth is not None):
                # Convert from RGB to BGR for OpenCV
                cv2_frame_plain = cv2.cvtColor(kinect_frame_plain, cv2.COLOR_RGB2BGR)
                cv2_frame_depth = cv2.cvtColor(kinect_frame_depth, cv2.COLOR_RGB2BGR)

                # Combine streams horizontally
                cv2_frame = np.hstack((cv2_frame_depth, cv2_frame_plain))
        
        return cv2_frame
    

    def run(self, x_pos: int = -1, y_pos: int = -1):
        while True:
            cv2_frame = self.__get_cv2_frame__()

            # Abort if there's no frame
            if cv2_frame is None:
                break

            if (x_pos >= 0) and (y_pos >= 0):
                self.__show_at__(cv2_frame, x_pos, y_pos)
            else:
                self.__show_centered__(cv2_frame)

            if cv2.waitKey(1) & 0xFF == 27:  # Esc key to stop
                break

        cv2.destroyAllWindows()


if __name__ == '__main__':
    print("=> Kinect sensor demo...")
    sensor = Kinect("Kinect RGB & Depth (ESC to quit)", display_mode=Kinect.RGB_AND_DEPTH)

    # sensor.run() - will display the feed from the sensor centered
    # sensor.run(x_pos=0, y_pos=0) - will display the feed from the sensor at the specified position
    sensor.run() # Will display the window center
