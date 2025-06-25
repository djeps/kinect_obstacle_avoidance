import os
import sys
import freenect as kinect
import cv2
import pyautogui
import numpy as np
import time

# from open3d import geometry, utility, visualization
from typing import NamedTuple
from suppressor import SuppressNativePrints
from math import sqrt
from threading import Thread, Event
from collections import deque


DRIVING_LANE_CONTOUR = None

class DisplayMode(NamedTuple):
    RGB: int
    DEPTH: int
    RGB_AND_DEPT: int


class SensorRange(NamedTuple):
    FULL: int
    OPTIMAL: int
    OPERATIONAL: int


class Kinect:

    # ----------------
    # Screen constants
    # ----------------
    DEFAULT_SCREEN_WIDTH = 1804
    DEFAULT_SCREEN_HEIGHT = 976
    DEFAULT_WINDOW_WIDTH = 640
    DEFAULT_WINDOW_HEIGHT = 480
    KEY_WAIT_DELAY_MS = 1

    # ----------------
    # Colors and modes
    # ----------------
    COLOR_BANDS = 6
    BAND_COLORS = 256
    RGB_ONLY = 1
    DEPTH_ONLY = 2
    RGB_AND_DEPTH = 3
    DISPLAY_MODE = DisplayMode(RGB=RGB_ONLY, DEPTH=DEPTH_ONLY, RGB_AND_DEPT=RGB_AND_DEPTH)
    # The color model in OpenCV is BGR (not RGB!)
    COLOR_BLACK = (0, 0, 0)
    COLOR_WHITE = (255, 255, 255)
    COLOR_GRAY = (100, 100, 100)
    COLOR_BLUE = (255, 0, 0)
    COLOR_RED = (255, 0, 0)
    COLOR_CYAN = (255, 255, 0)
    COLOR_GREEN = (0, 255, 0)
    COLOR_YELLOW = (0, 255, 255)
    COLOR_MAGENTA = (255, 0, 255)

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
    SENSOR_OPERATIONAL_RANGE = 3
    SENSOR_RANGE = SensorRange(FULL=SENSOR_FULL_RANGE, OPTIMAL=SENSOR_OPTIMAL_RANGE, OPERATIONAL=SENSOR_OPERATIONAL_RANGE)
    MIN_VISIBLE_SENSOR_DEPTH = 0
    MAX_VISIBLE_SENSOR_DEPTH = SENSOR_RESOLUTION - 1
    MIN_OPTIMAL_SENSOR_DEPTH = 200
    MAX_OPTIMAL_SENSOR_DEPTH = 1600
    MIN_OPERATIONAL_SENSOR_DEPTH = 500
    MAX_OPERATIONAL_SENSOR_DEPTH = 900
    DISTANCE_THRESHOLD = 800

    # ----------------
    # Lane positioning
    # ----------------
    VANISHING_POINT_X_RATIO = 0.5   # How inward the vanishing point is (0.5 = 50% from the left)
    VANISHING_POINT_Y_RATIO = 0.35  # How high up the vanishing point is (0.35 = 35% from top)
    LANE_WIDTH_BOTTOM_RATIO = 0.6   # Lane width at bottom as ratio of screen width
    LANE_WIDTH_TOP_RATIO = 0.18     # Lane width at vanishing point as ratio of screen width
    MAX_CURVE_DEGREE_RIGHT = 3.0
    MAX_CURVE_DEGREE_LEFT = -3.0
    CURVE_STEP = 0.05


    def capture_thread(self):
        """Continuously capture frames and update buffer"""
        while not self._stop_event.is_set():
            try:
                with SuppressNativePrints():
                    depth = kinect.sync_get_depth()[0]
            except Exception as e:
                print(f"=> An unexpected error occurred when returning a Kinect depth frame: {e}")


            self._frame_buffer.append(depth)
            time.sleep(0.001)  # 1ms to prevent thread from hogging CPU


    def __init__(self, window_name: str, display_mode: int = DISPLAY_MODE.RGB):
        if display_mode not in Kinect.DISPLAY_MODE:
            raise ValueError("Incorrect display mode! Allowed modes: RGB_ONLY, DEPTH_ONLY, RGB_AND_DEPTH.")
    
        self.__display_mode__ = display_mode

        print("=> Press ESC to exit the application.")

        # --------------------------------
        # Setting OpenCV window properties
        # --------------------------------
        self._window_name = window_name
        self._window_width = Kinect.DEFAULT_WINDOW_WIDTH * 2
        self._window_height = Kinect.DEFAULT_WINDOW_HEIGHT * 2

        cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._window_name, self._window_width, self._window_height)

        # ---------------------------------
        # Determining the screen resolution
        # ---------------------------------
        try:
            self._screen_width, self._screen_height = pyautogui.size()
        except Exception as e:
            print(f"=> An unexpected error occurred when determining the screen resolution: {e}")
            
            self._screen_width = Kinect.DEFAULT_SCREEN_WIDTH
            self._screen_height = Kinect.DEFAULT_SCREEN_HEIGHT
            
            print(f"=> Going with default values for screen width = {self.__screen_width} and height = {self._screen_height}.")

        # --------------------------------------------------------------
        # Create the gamma color table to match 'freenect-glview' output
        # --------------------------------------------------------------
        # Gamma correction table
        self._color_gamma = self._create_gamma()

        # --------------------------------------------------------------
        # Thread-safe buffer with max capacity of 1 frame
        # This is hopefully to improve the performance of the program
        # when frequently (with each frame refresh!) fetching depth data
        # -------------------------------------------------------------- 
        self._frame_buffer = deque(maxlen=1)
        self._stop_event = Event()

        # Start capture thread
        self._thread = Thread(target=self.capture_thread, daemon=True)
        self._thread.start()

        # Wait for the first frame to arrive (safety mechanism)
        while not self._frame_buffer:
            time.sleep(0.01)


    # Creates a lookup table and maps raw Kinect depth values (0–2047) to an RGB color
    # to produce a color gradient similar to what the 'freenect-glview' tool produces
    def _create_gamma(self):
        
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


    def _depth_to_m(self, depth):
        # Convert depth from raw units to meters!
        # Derived 'depth formula' for the Kinect v1
        depth_m = 1.0 / (depth * -0.0030711016 + 3.3309495161) # Depth in 'm'
        return depth_m


    # Convert raw depth data to physical data
    def _depth_to_physical(self, x, y, depth):
        if depth == 0:
            return None, None, None
                
        depth_m = self._depth_to_m(depth)
        
        # Convert to world coordinates in millimeters (mm)
        phys_x = ((x - Kinect.OPTICAL_CENTER_X) * depth_m / Kinect.FOCAL_LENGTH_X) * 1000 # Convert to 'mm'
        phys_y = ((y - Kinect.OPTICAL_CENTER_Y) * depth_m / Kinect.FOCAL_LENGTH_Y) * 1000 # Convert to 'mm'
        phys_z = depth_m * 1000
        
        return phys_x, phys_y, phys_z # In 'mm'


    def _depth_array_to_physical(self, depth):
        if depth is None:
            return None
        
        if len(depth) == 0:
            return None

        depth_mm = 1000 * self._depth_to_m(depth)
        
        return depth_mm


    # Returns an extended point cloud data
    # A tuple representing 0-the actual point cloud and 1-color mappings for each point in the cloud
    def _get_point_cloud_ext(self, raw_depth_data, sensor_range: int = SENSOR_RANGE.FULL, pixel_stepping: int = 4):
        if raw_depth_data is None:
            return (None, None, None)

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
                    if sensor_range == Kinect.SENSOR_RANGE.FULL:
                        if not(Kinect.MIN_VISIBLE_SENSOR_DEPTH < depth_value < Kinect.MAX_VISIBLE_SENSOR_DEPTH):
                            continue
                    elif sensor_range == Kinect.SENSOR_RANGE.OPTIMAL:
                        if not(Kinect.MIN_OPTIMAL_SENSOR_DEPTH < depth_value < Kinect.MAX_OPTIMAL_SENSOR_DEPTH):
                            continue
                    elif sensor_range == Kinect.SENSOR_RANGE.OPERATIONAL:
                        if not(Kinect.MIN_OPERATIONAL_SENSOR_DEPTH < depth_value < Kinect.MAX_OPERATIONAL_SENSOR_DEPTH):
                            continue
                    
                    # Otherwise...
                    # Convert to physical coordinates
                    world_x, world_y, world_z = self._depth_to_physical(x, y, depth_value)
                    
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
    def get_point_cloud_points(self, raw_depth_data, sensor_range: int = 1, pixel_stepping: int = 4):
        return self._get_point_cloud_ext(raw_depth_data, sensor_range, pixel_stepping)[0]


    # Returns point cloud data along with pixels representing each point in the cloud
    def get_point_cloud_points_and_pixels(self, raw_depth_data, sensor_range: int = 1, pixel_stepping: int = 4):
        point_cloud_data = self._get_point_cloud_ext(raw_depth_data, sensor_range, pixel_stepping)
        return (point_cloud_data[0], point_cloud_data[1])


    # A wrapper function that returns everything (points, pixels, colors)
    def get_point_cloud_data(self, raw_depth_data, sensor_range: int = 1, pixel_stepping: int = 4):
        return self._get_point_cloud_ext(raw_depth_data, sensor_range, pixel_stepping)


    def _get_min_max_points(self, points, pixels, in_xyz_space: bool = True):
        # The Kinect's origin is at (0, 0, 0)
        # When:
        #   in_xyz_space==True => 'distances' represent true Euclidian distances in 3D (XYZ space)
        # If we were to project these onto a 'ground plane' i.e. a plane in which
        # the Kinect is positioned into (a plane parallel to the ground) (XY plane)
        #   in_xyz_space==False => 'distances' represent 'ground plane' distances (XY-plane only!)
         
        closest = (None, None, None) # A point closest to the Kinect
        furthest = (None, None, None) # A point furthest from the Kinect

        distances = np.linalg.norm(points, axis=1) # Euclidian distances in XYZ space

        # Find the indices of the closest and furthest point
        idx_min = np.argmin(distances)
        idx_max = np.argmax(distances)
        
        if len(points) > 0:        
            if not in_xyz_space:
                distances = np.linalg.norm(points[:, :2], axis=1) # Euclidian distances in a XY-plane only
            
            # Get the actual points and their pixel coordinates
            point_min = points[idx_min]
            point_max = points[idx_max]
            pixel_min = pixels[idx_min]
            pixel_max = pixels[idx_max]

            closest = point_min, pixel_min, distances[idx_min]
            furthest = point_max, pixel_max, distances[idx_max]

        return (closest, furthest)


    def analyze_point_cloud_data(self, point_cloud_data, in_xyz_space: bool = True):
        # Get point cloud data
        points, pixels, _ = point_cloud_data
        
        if len(points) == 0:
            print("=> No valid points found!")
            return
        
        print("=> Analyzing point cloud data...")
        print("=> Valid point cloud data.")
        space = "XYZ" if in_xyz_space else "Ground plane"
        print(f"=> Points = {len(points)}; Space = {space}")
        
        # Find closest and furthest points
        closest, furthest = self._get_min_max_points(points, pixels, in_xyz_space=in_xyz_space)
        
        closest_point, closest_pixel, closest_dist = closest
        furthest_point, furthest_pixel, furthest_dist = furthest
        
        # If we have identified at least the closest point, the point cloud data is valid
        # and we can proceed further with post-processing
        if closest_point is not None:
            msg = ''.join(("=> Closest: ",
                           f"Point = ({closest_point[0]:.1f}, {closest_point[1]:.1f}, {closest_point[2]:.1f}) [mm]; ",
                           f"Pixel = ({closest_pixel[0]}, {closest_pixel[1]}); ",
                           f"Distance = {closest_dist:.1f}mm"))
            print(msg)

            msg = ''.join(("=> Furthest: ",
                           f"Point = ({furthest_point[0]:.1f}, {furthest_point[1]:.1f}, {furthest_point[2]:.1f}) [mm]; ",
                           f"Pixel = ({furthest_pixel[0]}, {furthest_pixel[1]}); ",
                           f"Distance = {furthest_dist:.1f}mm"))
            print(msg)

            print(f"=> Range = {furthest_dist - closest_dist:.1f}mm; Ratio = x {furthest_dist / closest_dist:.2f}")


    def _get_rgb_data(self):
        rgb_data = None

        try:
            with SuppressNativePrints():
                rgb_data = kinect.sync_get_video()[0]
        except Exception as e:
            print(f"=> An unexpected error occurred when returning a Kinect RGB frame: {e}")

        return rgb_data
    

    def _get_rgb_frame(self):
        cv2_rgb_frame = None

        rgb_frame = self._get_rgb_data()

        if rgb_frame is not None:
            # Convert from RGB to BGR for OpenCV
            cv2_rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
            
            # Put a descriptive text into the frame
            cv2.putText(cv2_rgb_frame,
                        f"RGB camera view",
                        org=(5, 25),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        color=Kinect.COLOR_WHITE,
                        fontScale=0.65,
                        thickness=1,
                        lineType=1)
        
        return cv2_rgb_frame


    def _get_depth_data(self):
        raw_depth_data = self._frame_buffer[0] if self._frame_buffer else None

        return raw_depth_data


    def _get_depth_frame(self):
        cv2_depth_frame = None

        depth_frame_raw = self._get_depth_data()

        if depth_frame_raw is not None:
            # Clip to a valid range
            # Basically points within the visible range of the Kinect
            depth_frame_clipped = np.clip(depth_frame_raw, Kinect.MIN_VISIBLE_SENSOR_DEPTH, Kinect.MAX_VISIBLE_SENSOR_DEPTH)
            
            # Apply gamma correction to depth data
            depth_frame_color = self._color_gamma[depth_frame_clipped]

            # Convert from RGB to BGR for OpenCV
            cv2_depth_frame = cv2.cvtColor(depth_frame_color, cv2.COLOR_RGB2BGR)

            # Put a descriptive text into the frame
            cv2.putText(cv2_depth_frame,
                        f"Depth view (color map)",
                        org=(5, 25),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        color=Kinect.COLOR_BLACK,
                        fontScale=0.65,
                        thickness=1,
                        lineType=1)

        return cv2_depth_frame
    

    # Get point cloud rendered frame
    def _get_pcd_frame(self, pixel_stepping: int = 4):
        cv2_pcd_frame = np.zeros((480, 640, 3), dtype=np.uint8)

        skip = pixel_stepping
        grid_y, grid_x = np.mgrid[0:480:skip, 0:640:skip]
        grid_y_flat = grid_y.ravel()
        grid_x_flat = grid_x.ravel()

        raw_depth = self._get_depth_data()

        sampled_depth = raw_depth[grid_y, grid_x].ravel()
        mask = (sampled_depth >= Kinect.MIN_OPERATIONAL_SENSOR_DEPTH) & (sampled_depth <= Kinect.MAX_OPERATIONAL_SENSOR_DEPTH)

        if np.any(mask):
            phys_z = self._depth_array_to_physical(sampled_depth[mask])
            valid_x = grid_x_flat[mask]
            valid_y = grid_y_flat[mask]

            min_depth = np.min(phys_z)
            max_depth = np.max(phys_z)
            depth_range = max_depth - min_depth if max_depth != min_depth else 1

            intensities = (255 * (1 - (phys_z - min_depth) / depth_range)).astype(np.uint8)
            cv2_pcd_frame[valid_y, valid_x, :] = intensities[:, None]

            # Put a descriptive text into the frame
            cv2.putText(cv2_pcd_frame,
                        f"Point cloud (operational depth)",
                        org=(5, 25),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        color=Kinect.COLOR_WHITE,
                        fontScale=0.65,
                        thickness=1,
                        lineType=1)

        return cv2_pcd_frame


    def _draw_horizontal_line(self, img, y_ratio_from_top, line_type = 'solid', thickness = 1, color = COLOR_WHITE):
        height, width = img.shape[:2]
        y_from_bottom = int(height * (1 - y_ratio_from_top))
        y = height - y_from_bottom
        
        if line_type == 'solid':
            cv2.line(img, (0, y), (width, y), color, thickness, cv2.LINE_8)
        else:
            if line_type == 'dashed_sparse':
                dash_length = max(int(width * 0.03), 10)
                gap_length = max(int(width * 0.03), 10)
            elif line_type == 'dashed_dense':
                dash_length = max(int(width * 0.015), 5)
                gap_length = max(int(width * 0.008), 3)
            else:
                cv2.line(img, (0, y), (width, y), color, thickness, cv2.LINE_8)
                return img
            
            x = 0
            while x < width:
                end_x = min(x + dash_length, width)
                cv2.line(img, (x, y), (end_x, y), color, thickness, cv2.LINE_8)
                x += dash_length + gap_length
    
        return img
    

    def _draw_perspective_lane(self,
                                  img,
                                  vanishing_point_ratio = (VANISHING_POINT_X_RATIO, VANISHING_POINT_Y_RATIO),
                                  lane_width_bottom = LANE_WIDTH_BOTTOM_RATIO,
                                  lane_width_top = LANE_WIDTH_TOP_RATIO,
                                  line_color = COLOR_WHITE, thickness=1):
        height, width = img.shape[:2]
        vanishing_x = int(width * vanishing_point_ratio[0])
        vanishing_y = int(height * vanishing_point_ratio[1])
        
        bottom_center = width // 2
        bottom_half_width = int(width * lane_width_bottom / 2)
        bottom_left = bottom_center - bottom_half_width
        bottom_right = bottom_center + bottom_half_width
        
        top_half_width = int(width * lane_width_top / 2)
        top_left = vanishing_x - top_half_width
        top_right = vanishing_x + top_half_width
        
        cv2.line(img, (bottom_left, height - 1), (top_left, vanishing_y), line_color, thickness, cv2.LINE_AA)
        cv2.line(img, (bottom_right, height - 1), (top_right, vanishing_y), line_color, thickness, cv2.LINE_AA)
        
        return img


    def _draw_curved_perspective_lanes(self,
                                          img,
                                          curve_amount = 0.0,
                                          vanishing_point_ratio = (VANISHING_POINT_X_RATIO, VANISHING_POINT_Y_RATIO),
                                          lane_width_bottom=LANE_WIDTH_BOTTOM_RATIO,
                                          lane_width_top=LANE_WIDTH_TOP_RATIO,
                                          line_color=(0, 255, 0), thickness=1):
        global DRIVING_LANE_CONTOUR
        
        height, width = img.shape[:2]
        
        # Use the SAME vanishing point as the straight lanes
        vanishing_x = int(width * vanishing_point_ratio[0])
        vanishing_y = int(height * vanishing_point_ratio[1])
        
        # Use the SAME lane width parameters as the straight lanes
        bottom_center = width // 2
        bottom_half_width = int(width * lane_width_bottom / 2)
        bottom_left = bottom_center - bottom_half_width
        bottom_right = bottom_center + bottom_half_width
        
        top_half_width = int(width * lane_width_top / 2)
        top_left = vanishing_x - top_half_width
        top_right = vanishing_x + top_half_width
        
        # Generate curved points that maintain perspective
        num_points = 100
        left_points = []
        right_points = []
        
        for i in range(num_points + 1):
            t = i / num_points  # parameter from 0 (bottom) to 1 (top)
            
            # Linear interpolation for y coordinate (same as straight lines)
            y = int(height - 1 + t * (vanishing_y - (height - 1)))
            
            # Linear interpolation for x coordinates (this gives the straight perspective lines)
            left_x_straight = bottom_left + t * (top_left - bottom_left)
            right_x_straight = bottom_right + t * (top_right - bottom_right)
            
            # Apply curve offset that gets stronger as we move up
            # The curve should be subtle at the bottom and more pronounced toward the vanishing point
            curve_strength = abs(curve_amount) * width * 0.1 * (t ** 1.5)  # Exponential growth
            
            # Apply curve in the same direction for both lines
            if curve_amount > 0:  # Right turn
                curve_offset = curve_strength
            elif curve_amount < 0:  # Left turn
                curve_offset = -curve_strength
            else:  # Straight
                curve_offset = 0
            
            # BOTH lines curve in the SAME direction while maintaining perspective
            left_x_curved = left_x_straight + curve_offset
            right_x_curved = right_x_straight + curve_offset
            
            left_points.append((int(left_x_curved), y))
            right_points.append((int(right_x_curved), y))
        
        # Draw the curved lanes
        cv2.polylines(img,
                      [np.array(left_points, dtype=np.int32)],
                      isClosed=False,
                      color=line_color,
                      thickness=thickness,
                      lineType=cv2.LINE_AA)
        cv2.polylines(img,
                      [np.array(right_points, dtype=np.int32)],
                      isClosed=False,
                      color=line_color,
                      thickness=thickness,
                      lineType=cv2.LINE_AA)
        
         # Create closed contour (left points + reversed right points)
        lane_contour = np.vstack([left_points, right_points[::-1]])
        DRIVING_LANE_CONTOUR = lane_contour.astype(np.int32)

        return img


    # Creates a complete driving scene with both straight and curved lane perspectives
    def _create_driving_scene(self, img, curved_lane_color = COLOR_MAGENTA, curve_amount = 0.0):
        # ----------------
        # Draw the horizon
        # ----------------
        img = self._draw_horizontal_line(img,
                                            y_ratio_from_top=Kinect.VANISHING_POINT_Y_RATIO,
                                            line_type='dashed_dense',
                                            thickness=1,
                                            color=Kinect.COLOR_GRAY)
        
        # ----------------------
        # Draw the straight lane
        # ----------------------
        img = self._draw_perspective_lane(img,
                                             vanishing_point_ratio=(Kinect.VANISHING_POINT_X_RATIO, Kinect.VANISHING_POINT_Y_RATIO), 
                                             lane_width_bottom=Kinect.LANE_WIDTH_BOTTOM_RATIO, 
                                             lane_width_top=Kinect.LANE_WIDTH_TOP_RATIO,
                                             line_color=Kinect.COLOR_BLUE,
                                             thickness=1)
        
        # ----------------------------
        # Draw the curved/turning lane
        # ----------------------------
        img = self._draw_curved_perspective_lanes(img,
                                                     curve_amount=curve_amount,
                                                     vanishing_point_ratio=(Kinect.VANISHING_POINT_X_RATIO, Kinect.VANISHING_POINT_Y_RATIO),
                                                     lane_width_bottom=Kinect.LANE_WIDTH_BOTTOM_RATIO, 
                                                     lane_width_top=Kinect.LANE_WIDTH_TOP_RATIO,
                                                     line_color=curved_lane_color,
                                                     thickness=1)
        
        return img


    def _get_distance_from_sensor(self, point):
        if point is None:
            return -1
        
        x = point[0]
        y = point[1]
        z = point[2]

        if x is None or y is None or z is None:
            return -2
        
        # As the x, y and z point coordinates are already converted in mm
        # We don't need to apply a factor of 1000
        distance = sqrt(x**2 + y**2 + z**2) 
        gnd_distance = sqrt(x**2 + y**2)

        return (distance, gnd_distance)
    

    def _get_driving_scene_frame(self, raw_depth_data, curved_lane_color = COLOR_MAGENTA, curve_amount: float = 0.0):
        cv2_drive_frame = np.zeros((480, 640, 3), dtype=np.uint8)

        cv2_drive_frame = self._create_driving_scene(img=cv2_drive_frame, curved_lane_color=curved_lane_color, curve_amount=curve_amount)
        
        points, pixels, colors = self.get_point_cloud_data(raw_depth_data, sensor_range=Kinect.SENSOR_OPERATIONAL_RANGE, pixel_stepping=4)

        collision = ''
        i = 0

        for pixel in pixels:
            if self._pixel_in_lane(pixel):
                # Get the distance from the sensor to the point inside the 'ground'/XY place
                # If this distance (in mm!) is less than the given threshold
                # display the original pixel (from the RGB view)
                _, gnd_distance = self._get_distance_from_sensor(points[i])
                if gnd_distance < Kinect.DISTANCE_THRESHOLD:
                    cv2.circle(cv2_drive_frame, pixel, 1, Kinect.COLOR_YELLOW, thickness=-1, lineType=cv2.LINE_AA)
                    collision = " (possible collision)"
            i += 1

        # Put a descriptive text into the frame
        direction = "left" if curve_amount < 0 else "RIGHT" if curve_amount > 0 else "straight"
        msg = f"Steering{collision}: {direction} ({curve_amount:.2f})"

        cv2.putText(cv2_drive_frame,
                    msg,
                    org=(5, 25),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    color=Kinect.COLOR_WHITE,
                    fontScale=0.65,
                    thickness=1,
                    lineType=1)
        
        return cv2_drive_frame
    

    def _get_cv2_frame(self, raw_depth_data, curve_amount: float = 0.0):
        cv2_frame = None

        # -----------------------
        # RGB and Depth color map
        # -----------------------
        cv2_frame_h1 = None
        cv2_depth_frame = self._get_depth_frame()
        cv2_rgb_frame = self._get_rgb_frame()

        if cv2_depth_frame is not None and cv2_rgb_frame is not None:
            cv2_frame_h1 = np.hstack((cv2_depth_frame, cv2_rgb_frame))
        
        # -----------------------------
        # Point cloud and driving scene
        # -----------------------------
        cv2_frame_h2 = None
        cv2_pcd_frame = self._get_pcd_frame()
        cv2_drive_frame = self._get_driving_scene_frame(raw_depth_data, curve_amount=curve_amount)

        if cv2_pcd_frame is not None and cv2_drive_frame is not None:
            cv2_frame_h2 = np.hstack((cv2_pcd_frame, cv2_drive_frame))
        
        if cv2_frame_h1 is not None:
            if cv2_frame_h2 is not None:
                cv2_frame = np.vstack((cv2_frame_h1, cv2_frame_h2))
            else:
                cv2_frame = cv2_frame_h1

        return cv2_frame
    

    def _pixel_in_lane(self, pixel):
        global DRIVING_LANE_CONTOUR
        
        if DRIVING_LANE_CONTOUR is None:
            return True  # No lane has been drawn yet

        # Convert point to (x, y) tuple and check against contour
        result = cv2.pointPolygonTest(DRIVING_LANE_CONTOUR, (int(pixel[0]), int(pixel[1])), measureDist=False)
        
        # Returns
        # True: if the pixel is outside of the driving lane contour
        # False: if inside or on the edge of the driving lane contour
        return result > 0 


    def _show_at(self, cv2_frame, x_pos: int, y_pos: int):
        cv2.moveWindow(self._window_name, x_pos, y_pos)
        cv2.imshow(self._window_name, cv2_frame)


    def _show_centered(self, cv2_frame):
        x_pos_center = (self._screen_width - self._window_width) // 2
        y_pos_center = (self._screen_height - self._window_height) // 2
        self._show_at(cv2_frame, x_pos_center, y_pos_center)


    def run(self, x_pos: int = -1, y_pos: int = -1):
        curve_amount = 0.0

        try:
            while True:
                raw_depth_data = sensor._get_depth_data()

                if raw_depth_data is None:
                    print("=> Error! Failed to get the raw depth data from the sensor!")
                    break

                cv2_frame = self._get_cv2_frame(raw_depth_data, curve_amount=curve_amount)

                # Abort if there's no frame
                if cv2_frame is None:
                    print("=> Error! Nothing to display (no frames are available).")
                    break

                if (x_pos >= 0) and (y_pos >= 0):
                    self._show_at(cv2_frame, x_pos, y_pos)
                else:
                    self._show_centered(cv2_frame)

                key_pressed = cv2.waitKey(Kinect.KEY_WAIT_DELAY_MS) # Wait for a key event for 250ms

                if key_pressed == ord('1'):
                    print("=> Sensor range: optimal")
                    point_cloud_data = sensor.get_point_cloud_data(raw_depth_data, Kinect.SENSOR_RANGE.OPTIMAL)
                    sensor.analyze_point_cloud_data(point_cloud_data, in_xyz_space=True)
                
                elif key_pressed == ord ('2'):
                    print("=> Sensor range: operational")
                    point_cloud_data = sensor.get_point_cloud_data(raw_depth_data, Kinect.SENSOR_RANGE.OPERATIONAL)
                    sensor.analyze_point_cloud_data(point_cloud_data, in_xyz_space=True)
                
                elif key_pressed == ord('3'):
                    print("=> Sensor range: optimal")
                    point_cloud_data = sensor.get_point_cloud_data(raw_depth_data, Kinect.SENSOR_RANGE.OPTIMAL)
                    sensor.analyze_point_cloud_data(point_cloud_data, in_xyz_space=False)
                
                elif key_pressed == ord('4'):
                    print("=> Sensor range: operational")
                    point_cloud_data = sensor.get_point_cloud_data(raw_depth_data, Kinect.SENSOR_RANGE.OPERATIONAL)
                    sensor.analyze_point_cloud_data(point_cloud_data, in_xyz_space=False)

                elif key_pressed == 81 or key_pressed == ord('a'):  # Left arrow or 'a'
                    # print(f"=> Turning LEFT (amount: {curve_amount:.2f})")
                    curve_amount = max(curve_amount - Kinect.CURVE_STEP, Kinect.MAX_CURVE_DEGREE_LEFT)
                
                elif key_pressed == 83 or key_pressed == ord('d'):  # Right arrow or 'd'
                    # print(f"=> Turning RIGHT (amount: {curve_amount:.2f})")
                    curve_amount = min(curve_amount + Kinect.CURVE_STEP, Kinect.MAX_CURVE_DEGREE_RIGHT)
                
                elif key_pressed == ord('r'):  # Reset to a straight driving lane
                    # print(f"=> Vehicle facing straight.")
                    curve_amount = 0.0
                
                elif key_pressed == ord('0'):
                    os.system('clear')
                    print("=> Screen output cleared.")
                
                elif key_pressed == 27:
                    break # ESC was pressed

            cv2.destroyAllWindows()
        finally:
            self._stop_event.set()
            self._thread.join(timeout=1.0)
            kinect.sync_stop()



if __name__ == '__main__':
    print("=> Kinect 'obstacle avoidance' POC demo...")
    sensor = Kinect("Kinect 'obstacle avoidance' POC demo... (ESC to quit)", display_mode=Kinect.RGB_AND_DEPTH)

    # sensor.run() - will display the feed from the sensor centered
    # sensor.run(x_pos=0, y_pos=0) - will display the feed from the sensor at the specified position
    sensor.run() # Will display the window center


    print("=> Done.")
