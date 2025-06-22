import sys
import os
import freenect as kinect
import cv2
import pyautogui
import numpy as np
import math

from open3d import geometry, utility, visualization
from contextlib import contextmanager


# Suppress output on 'stdout'
@contextmanager
def suppress_stdout():
    with open(os.devnull, "w") as devnull:
        old_stdout = sys.stdout
        sys.stdout = devnull
        try:
            yield
        finally:
            sys.stdout = old_stdout


# Suppress output on 'stderr'
@contextmanager
def suppress_stderr():
    with open(os.devnull, "w") as devnull:
        old_stderr = sys.stderr
        sys.stderr = devnull
        try:
            yield
        finally:
            sys.stderr = old_stderr


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
    SENSOR_FULL_RANGE = 1
    SENSOR_OPTIMAL_RANGE = 2
    MIN_VISIBLE_SENSOR_DEPTH = 0
    MAX_VISIBLE_SENSOR_DEPTH = SENSOR_RESOLUTION - 1
    MIN_OPTIMAL_SENSOR_DEPTH = 200
    MAX_OPTIMAL_SENSOR_DEPTH = 1600
    OPTIMAL_SENSOR_DEPTH = 500
    OPTIMAL_SENSOR_DEPTH = 900

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
        raw_depth_data = None

        try:
            with suppress_stdout(), suppress_stderr():
                raw_depth_data = kinect.sync_get_depth()[0]
        except Exception as e:
            print(f"=> An unexpected error occurred when returning a Kinect depth frame: {e}")
        
        return raw_depth_data


    # Convert raw depth data to physical data
    def __depth_to_physical__(self, x, y, depth):
        if depth == 0:
            return None, None, None
        
        # Convert depth from raw units to meters!
        # Derived 'depth formula' for the Kinect v1
        depth_m = 1.0 / (depth * -0.0030711016 + 3.3309495161)
        
        # Convert to world coordinates in millimeters (mm)
        phys_x = ((x - Kinect.OPTICAL_CENTER_X) * depth_m / Kinect.FOCAL_LENGTH_X) * 1000
        phys_y = ((y - Kinect.OPTICAL_CENTER_Y) * depth_m / Kinect.FOCAL_LENGTH_Y) * 1000
        phys_z = depth_m * 1000
        
        return phys_x, phys_y, phys_z


    # Returns an extended point cloud data
    # A tuple representing 0-the actual point cloud and 1-color mappings for each point in the cloud
    def __get_point_cloud_ext__(self, sensor_range: int = 1, pixel_stepping: int = 4):
        raw_depth_data = self.__get_depth_data__()

        points = None
        pixels = None
        colors = None

        if raw_depth_data is not None:
            points = []
            pixels = []
            colors = []
            
            # Skip pixels for performance
            skip = pixel_stepping
            
            for y in range(0, raw_depth_data.shape[0], skip):
                for x in range(0, raw_depth_data.shape[1], skip):
                    depth_value = raw_depth_data[y, x]
                    
                    # Skip invalid depth readings depending on the selected sensor range 
                    if sensor_range == Kinect.SENSOR_OPTIMAL_RANGE:
                        if depth_value == Kinect.MIN_OPTIMAL_SENSOR_DEPTH or depth_value > Kinect.MAX_OPTIMAL_SENSOR_DEPTH:
                            continue # No further loop processing
                    else:
                        if depth_value == Kinect.MIN_VISIBLE_SENSOR_DEPTH or depth_value > Kinect.MAX_VISIBLE_SENSOR_DEPTH:
                            continue # No further loop processing
                    
                    # Otherwise...
                    # Convert to physical coordinates
                    world_x, world_y, world_z = self.__depth_to_physical__(x, y, depth_value)
                    
                    if world_x is not None:
                        points.append([world_x, world_y, world_z]) # A point in physical space
                        pixels.append([x, y])  # A pixel in the sensor's view representing that point
                        
                        # Add color based on depth for better visualization, and
                        # Normalize depth for coloring (closer points = red, farther points = blue)
                        normalized_depth = min(depth_value / 2047.0, 1.0)
                        color = [int(255 * (1 - normalized_depth)), 0, int(255 * normalized_depth)]
                        colors.append(color)
            
            points = np.array(points)
            pixels = np.array(pixels)
            colors = np.array(colors)
        
        return (points, pixels, colors)


    # Returns point cloud data only
    def get_point_cloud(self, sensor_range: int = 1, pixel_stepping: int = 4):
        return self.__get_point_cloud_ext__(sensor_range, pixel_stepping)[0]


    # Returns point cloud data along with pixels representing each point in the cloud
    def get_point_cloud_and_pixels(self, sensor_range: int = 1, pixel_stepping: int = 4):
        point_cloud_data = self.__get_point_cloud_ext__(sensor_range, pixel_stepping)
        return (point_cloud_data[0], point_cloud_data[1])


    # A wrapper function that returns everything (points, pixels, colors)
    def get_point_cloud_data(self, sensor_range: int = 1, pixel_stepping: int = 4):
        return self.__get_point_cloud_ext__(sensor_range, pixel_stepping)


    def __get_min_max_points__(self, points, pixels, in_xyz_space: bool = True):
        # The Kinect's origin is at (0, 0, 0)
        # When:
        #   in_xyz_space==True => 'distances' represent true Euclidian distances in 3D (XYZ space)
        # If we were to project these onto a 'ground plane' i.e. a plane in which
        # the Kinect is positioned into (a plane parallel to the ground)
        #   in_xyz_space==False => 'distances' represent 'ground plane' distances (XY-plane only!)
         
        closest = (None, None, None) # A point closest to the Kinect
        furthest = (None, None, None) # A point furthest from the Kinect

        if len(points) > 0:        
            if in_xyz_space:
                distances = np.linalg.norm(points, axis=1) # Euclidian distances in XYZ space
            else:
                distances = np.linalg.norm(points[:, :2], axis=1) # Euclidian distances in a XY-plane only
            
            # Find the indices of the closest and furthest point
            idx_min = np.argmin(distances)
            idx_max = np.argmax(distances)
            
            # Get the actual points and their pixel coordinates
            point_min = points[idx_min]
            point_max = points[idx_max]
            pixel_min = pixels[idx_min]
            pixel_max = pixels[idx_max]

            closest = point_min, pixel_min, distances[idx_min]
            furthest = point_max, pixel_max, distances[idx_max]

        return (closest, furthest)


    # Analyze point cloud
    def analyze_point_cloud_data(self, point_cloud_data, in_xyz_space: bool = True):
        # Get point cloud data
        points, pixels, _ = point_cloud_data
        
        if len(points) == 0:
            print("=> No valid points found!")
            return
        
        print("=> Analyzing point cloud data...\n")
        print("   Valid point cloud data.")
        print(f"   Available: {len(points)} 3D points.\n")
        print("   Summary:")
        print('\n')
        
        # Find closest and furthest points
        closest, furthest = self.__get_min_max_points__(points, pixels, in_xyz_space=in_xyz_space)
        
        closest_point, closest_pixel, closest_dist = closest
        furthest_point, furthest_pixel, furthest_dist = furthest
        
        # If at least we have the closest point identified, the point cloud data is valid
        # and we can proceed further with post-processing
        if closest_point is not None:
            msg = "XYZ space" if in_xyz_space else "ground plane!"
            msg = f"   CLOSEST POINT TO KINECT ({msg}):"
            print(msg)
            print('   ', end='')
            print('-' * 45)

            print(f"   3D Coordinates: ({closest_point[0]:.1f}, {closest_point[1]:.1f}, {closest_point[2]:.1f})mm")
            print(f"   Pixel Location: ({closest_pixel[0]}, {closest_pixel[1]})")
            print(f"   Distance: {closest_dist:.1f}mm ({closest_dist/1000:.2f}m)")
            
            print('\n')

            msg = "XYZ space" if in_xyz_space else "ground plane!"
            msg = f"   FURTHEST POINT FROM KINECT ({msg}):"
            print(msg)
            print('   ', end='')
            print('-' * 45)
            
            print(f"   3D Coordinates: ({furthest_point[0]:.1f}, {furthest_point[1]:.1f}, {furthest_point[2]:.1f})mm")
            print(f"   Pixel Location: ({furthest_pixel[0]}, {furthest_pixel[1]})")
            print(f"   Distance: {furthest_dist:.1f}mm ({furthest_dist/1000:.2f}m)")
            
            print("   ---")
            print(f"   Distance Range: {furthest_dist - closest_dist:.1f}mm")
            print(f"   Distance Ratio: {furthest_dist / closest_dist:.2f}x")
            
            print('\n')

    def __get_rgb_data__(self):
        rgb_data = None

        try:
            with suppress_stdout(), suppress_stderr():
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
            kinect_frame_clipped = np.clip(kinect_frame_raw, Kinect.MIN_VISIBLE_SENSOR_DEPTH, Kinect.MAX_VISIBLE_SENSOR_DEPTH)
            
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

    point_cloud_data = sensor.get_point_cloud_data(Kinect.SENSOR_OPTIMAL_RANGE)
    sensor.analyze_point_cloud_data(point_cloud_data, in_xyz_space=True)
    # sensor.analyze_point_cloud_data(point_cloud_data, in_xyz_space=False)

    print("=> Done.")
