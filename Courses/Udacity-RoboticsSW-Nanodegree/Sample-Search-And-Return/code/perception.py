import numpy as np
import cv2

# identify pixels as belong to particular object type based on RGB color range
def color_thresh_cv2(rgb_img, rgb_thresh_low=[0,0,0], rgb_thresh_high=[50,50,50]):
    """
    Detects object pixels in an RGB image and returns a binary mask image. User needs to provide appropriate range
    to detect the object of choice
    :param rgb_img: l_y X l_x X 3 array
    :param rgb_thresh_low: 3 element list for R,G,B low threshold in order
    :param rgb_thresh_high: 3 element list for R,G,B high threshold in order
    :return: l_y X l_x binary mask image, 1 corresponds to object pixels, 0 corresponds to non-object pixel
    """
    bgr_img = np.flip(rgb_img, axis=2)
    color_select = cv2.inRange(bgr_img, rgb_thresh_low, rgb_thresh_high)
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2).astype(np.float)
    return x_pixel, y_pixel

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
                              [Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
                              ])
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    terrain = color_thresh_cv2(warped, np.array([145, 145, 145]), np.array([255, 255, 255]))
    rocks = color_thresh_cv2(warped, np.array([0, 80, 80]), np.array([60, 255, 255]))
    obstacles = np.ones((Rover.img.shape[0], Rover.img.shape[1]))
    obstacles[np.logical_or(rocks, terrain)] = 0

    # obstacles = color_thresh_cv2(warped, np.array([0, 0, 0]), np.array([60, 60, 80]))

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obstacles*255
    Rover.vision_image[:,:,1] = rocks
    Rover.vision_image[:,:,2] = terrain

    # 5) Convert map image pixel values to rover-centric coords
    xterrain, yterrain = rover_coords(terrain)
    xrock, yrock = rover_coords(rocks)
    xobs, yobs = rover_coords(obstacles)

    # 6) Convert rover-centric pixel values to world coordinates
    world_size = np.shape(Rover.worldmap)[0]
    scale = 10
    ter_x_world, ter_y_world = pix_to_world(xterrain, yterrain, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    rock_x_world, rock_y_world = pix_to_world(xrock, yrock, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    obstacle_x_world, obstacle_y_world = pix_to_world(xobs, yobs, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    Rover.worldmap[ter_y_world, ter_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(xterrain, yterrain)
    Rover.nav_dists = rover_centric_pixel_distances
    Rover.nav_angles = rover_centric_angles

    return Rover