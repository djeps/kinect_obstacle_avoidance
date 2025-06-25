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
        return (None, None, None)

    cdef:
        Py_ssize_t height = raw_depth_data.shape[0]
        Py_ssize_t width = raw_depth_data.shape[1]
        Py_ssize_t y, x, idx = 0
        np.uint16_t depth_value

        # Precompute depth conversion constants
        double depth_a = -0.0030711016
        double depth_b = 3.3309495161
        double depth_m, world_x, world_y, world_z, normalized_depth
        
        bint skip_pixel = False
        
        # Pre-allocate C arrays
        Py_ssize_t max_points = (height // pixel_stepping) * (width // pixel_stepping)
        double* points_data = <double*> malloc(3 * max_points * sizeof(double))
        int* pixels_data = <int*> malloc(2 * max_points * sizeof(int))
        unsigned char* colors_data = <unsigned char*> malloc(3 * max_points * sizeof(unsigned char))
    
    if points_data == NULL or pixels_data == NULL or colors_data == NULL:
        free(points_data)
        free(pixels_data)
        free(colors_data)
        raise MemoryError("Failed to allocate memory")
    
    # Ensure pixel_stepping >= 1
    if pixel_stepping < 1:
        pixel_stepping = 1

    y = 0

    while y < height:
        x = 0
        
        while x < width:
            depth_value = raw_depth_data[y, x]
            
             # Store skip status
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

            # ALWAYS INCREMENT x FIRST
            x += pixel_stepping

            # Skip processing if needed
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
            colors_data[3*idx+2] = <unsigned char>(255 * normalized_depth)

            idx += 1

        y += pixel_stepping
    
    # CONVERSION BLOCK (requires GIL)
    points_arr = np.asarray(<double[:3*idx]> points_data).reshape(-1, 3)
    pixels_arr = np.asarray(<int[:2*idx]> pixels_data).reshape(-1, 2)
    colors_arr = np.asarray(<unsigned char[:3*idx]> colors_data).reshape(-1, 3)
     
    free(points_data)
    free(pixels_data)
    free(colors_data)

    return (points_arr, pixels_arr, colors_arr)
