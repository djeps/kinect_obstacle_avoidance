# pointcloud.pyx
import numpy as np
cimport numpy as np
import cython
from libc.stdlib cimport malloc, free

# Define constants
cdef int SENSOR_FULL_RANGE = 1
cdef int SENSOR_OPTIMAL_RANGE = 2
cdef int SENSOR_OPERATIONAL_RANGE = 3

cdef int MIN_VISIBLE_SENSOR_DEPTH = 0
cdef int MAX_VISIBLE_SENSOR_DEPTH = 2047
cdef int MIN_OPTIMAL_SENSOR_DEPTH = 200
cdef int MAX_OPTIMAL_SENSOR_DEPTH = 1600
cdef int MIN_OPERATIONAL_SENSOR_DEPTH = 500
cdef int MAX_OPERATIONAL_SENSOR_DEPTH = 900

cdef double FOCAL_LENGTH_X = 594.21
cdef double FOCAL_LENGTH_Y = 591.04
cdef double OPTICAL_CENTER_X = 339.5
cdef double OPTICAL_CENTER_Y = 242.7

@cython.boundscheck(False)
@cython.wraparound(False)
def get_point_cloud_ext(
    np.uint16_t[:, :] raw_depth_data, 
    int sensor_range, 
    int pixel_stepping
):
    if raw_depth_data is None:
        return (np.empty((0, 3), dtype=np.float32),
                np.empty((0, 2), dtype=np.int32),
                np.empty((0, 3), dtype=np.uint8))
    
    cdef:
        Py_ssize_t height = raw_depth_data.shape[0]
        Py_ssize_t width = raw_depth_data.shape[1]
        Py_ssize_t y = 0, x = 0, idx = 0
        np.uint16_t depth_value
        double depth_m, world_x, world_y, world_z, normalized_depth
        
        # Pre-allocate C arrays
        Py_ssize_t max_points = (height // pixel_stepping) * (width // pixel_stepping)
        double* points_data = NULL
        int* pixels_data = NULL
        unsigned char* colors_data = NULL
    
    if max_points == 0:
        return (np.empty((0, 3), dtype=np.float32),
                np.empty((0, 2), dtype=np.int32),
                np.empty((0, 3), dtype=np.uint8))
    
    points_data = <double*> malloc(3 * max_points * sizeof(double))
    pixels_data = <int*> malloc(2 * max_points * sizeof(int))
    colors_data = <unsigned char*> malloc(3 * max_points * sizeof(unsigned char))
    
    if points_data == NULL or pixels_data == NULL or colors_data == NULL:
        if points_data != NULL: free(points_data)
        if pixels_data != NULL: free(pixels_data)
        if colors_data != NULL: free(colors_data)
        raise MemoryError("Memory allocation failed")
    
    cdef double depth_a = -0.0030711016
    cdef double depth_b = 3.3309495161
    
    y = 0
    while y < height:
        x = 0
        while x < width:
            depth_value = raw_depth_data[y, x]
            
            # ALWAYS increment x first to prevent infinite loops
            x += pixel_stepping
            
            # Skip invalid depth
            skip_pixel = False
            if sensor_range == SENSOR_FULL_RANGE:
                if depth_value <= MIN_VISIBLE_SENSOR_DEPTH or depth_value >= MAX_VISIBLE_SENSOR_DEPTH:
                    skip_pixel = True
            elif sensor_range == SENSOR_OPTIMAL_RANGE:
                if depth_value <= MIN_OPTIMAL_SENSOR_DEPTH or depth_value >= MAX_OPTIMAL_SENSOR_DEPTH:
                    skip_pixel = True
            elif sensor_range == SENSOR_OPERATIONAL_RANGE:
                if depth_value <= MIN_OPERATIONAL_SENSOR_DEPTH or depth_value >= MAX_OPERATIONAL_SENSOR_DEPTH:
                    skip_pixel = True
            if skip_pixel:
                continue
            
            # Convert to physical coordinates
            depth_m = 1.0 / (depth_value * depth_a + depth_b)
            
            # Convert to world coordinates (mm)
            world_x = ((x - OPTICAL_CENTER_X) * depth_m / FOCAL_LENGTH_X) * 1000
            world_y = ((y - OPTICAL_CENTER_Y) * depth_m / FOCAL_LENGTH_Y) * 1000
            world_z = depth_m * 1000
            
            # Store in C arrays
            points_data[3*idx] = world_x
            points_data[3*idx+1] = world_y
            points_data[3*idx+2] = world_z
            
            pixels_data[2*idx] = x
            pixels_data[2*idx+1] = y
            
            normalized_depth = depth_value / 2047.0
            
            if normalized_depth > 1.0:
                normalized_depth = 1.0
            
            colors_data[3*idx] = <unsigned char>(255 * (1 - normalized_depth))
            colors_data[3*idx+1] = 0
            
            idx += 1
        y += pixel_stepping
    
    # Handle zero-points case
    if idx == 0:
        return (np.empty((0, 3), dtype=np.float32),
                np.empty((0, 2), dtype=np.int32),
                np.empty((0, 3), dtype=np.uint8))
    
    # Create arrays with explicit copy
    cdef:
        np.ndarray points_arr = np.empty((idx, 3), dtype=np.float32)
        np.ndarray pixels_arr = np.empty((idx, 2), dtype=np.int32)
        np.ndarray colors_arr = np.empty((idx, 3), dtype=np.uint8)
    
    for i in range(idx):
        points_arr[i, 0] = points_data[3*i]
        points_arr[i, 1] = points_data[3*i+1]
        points_arr[i, 2] = points_data[3*i+2]
        pixels_arr[i, 0] = pixels_data[2*i]
        pixels_arr[i, 1] = pixels_data[2*i+1]
        colors_arr[i, 0] = colors_data[3*i]
        colors_arr[i, 1] = colors_data[3*i+1]
        colors_arr[i, 2] = colors_data[3*i+2]
    
    if points_data != NULL: free(points_data)
    if pixels_data != NULL: free(pixels_data)
    if colors_data != NULL: free(colors_data)

    return (points_arr, pixels_arr, colors_arr)
