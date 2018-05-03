import cv2
import numpy as np


class RoverMode:
    STOP = 'stop'
    FORWARD = 'forward'
    APPROACH_SAMPLE = 'approach_sample'
    STUCK = 'stuck'


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
                   & (img[:, :, 1] > rgb_thresh[1]) \
                   & (img[:, :, 2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select


def color_between_thresh(img, rgb_thresh_low=(160, 160, 160), rgb_thresh_high=(255, 255, 255)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:, :, 0] > rgb_thresh_low[0]) \
                   & (img[:, :, 0] < rgb_thresh_high[0]) \
                   & (img[:, :, 1] > rgb_thresh_low[1]) \
                   & (img[:, :, 1] < rgb_thresh_high[1]) \
                   & (img[:, :, 2] > rgb_thresh_low[2]) \
                   & (img[:, :, 2] < rgb_thresh_high[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select


# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1] / 2).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
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
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))  # keep same size as input image

    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()

    ###########################################################################
    # 0. Extract image
    ###########################################################################
    image = Rover.img

    ###########################################################################
    # 1. Define source and destination points for perspective transform
    ###########################################################################
    # Define calibration box in source (actual) and destination (desired) coordinates
    # These source and destination points are defined to warp the image
    # to a grid where each 10x10 pixel square represents 1 square meter
    # The destination box will be 2*dst_size on each side
    dst_size = 5
    # Set a bottom offset to account for the fact that the bottom of the image
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[image.shape[1] / 2 - dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1] / 2 + dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1] / 2 + dst_size, image.shape[0] - 2 * dst_size - bottom_offset],
                              [image.shape[1] / 2 - dst_size, image.shape[0] - 2 * dst_size - bottom_offset]])

    ###########################################################################
    # 2. Apply perspective transform
    ###########################################################################
    warped = perspect_transform(image, source, destination)

    ###########################################################################
    # 3. Apply color threshold to identify navigable terrain/obstacles/rock
    ###########################################################################
    navigable_mask = color_thresh(warped, rgb_thresh=(160, 160, 160))
    obstacle_mask = np.ones_like(navigable_mask) - navigable_mask
    sample_mask = color_between_thresh(warped, (0, 105, 0), (255, 220, 65))

    ###########################################################################
    # 4. Update Rover.vision_image (displayed on bottom left of simulator)
    ###########################################################################
    Rover.vision_image[:, :, 0] = obstacle_mask * 255
    Rover.vision_image[:, :, 1] = sample_mask * 255
    Rover.vision_image[:, :, 2] = navigable_mask * 255

    ###########################################################################
    # 5. Convert map image pixel values to rover-centric coords
    ###########################################################################
    xpix_navigable, ypix_navigable = rover_coords(navigable_mask)
    xpix_obstacle, ypix_obstacle = rover_coords(obstacle_mask)
    xpix_sample, ypix_sample = rover_coords(sample_mask)

    ###########################################################################
    # 6. Convert rover-centric pixel values to world coordinates
    ###########################################################################
    rover_xpos, rover_ypos = Rover.pos
    rover_yaw = Rover.yaw
    world_size = Rover.worldmap.shape[0]
    scale = 10

    x_world_navigable, y_world_navigable = pix_to_world(xpix=xpix_navigable, ypix=ypix_navigable, xpos=rover_xpos,
                                                        ypos=rover_ypos, yaw=rover_yaw, world_size=world_size,
                                                        scale=scale)

    x_world_obstacle, y_world_obstacle = pix_to_world(xpix=xpix_obstacle, ypix=ypix_obstacle, xpos=rover_xpos,
                                                      ypos=rover_ypos, yaw=rover_yaw, world_size=world_size,
                                                      scale=scale)

    x_world_sample, y_world_sample = pix_to_world(xpix=xpix_sample, ypix=ypix_sample, xpos=rover_xpos,
                                                  ypos=rover_ypos, yaw=rover_yaw, world_size=world_size,
                                                  scale=scale)

    ###########################################################################
    # 7. Update Rover worldmap
    ###########################################################################
    # Update worldmap if rover is stable - without too much roll or pitch
    if abs(Rover.pitch) < 5 and abs(Rover.roll) < 5:
        Rover.worldmap[y_world_obstacle, x_world_obstacle, 0] += 1
        Rover.worldmap[y_world_sample, x_world_sample, 1] += 1
        Rover.worldmap[y_world_navigable, x_world_navigable, 2] += 1

    ###########################################################################
    # 8. Convert rover-centric pixel positions to polar coordinates
    ###########################################################################
    dists_navigable, angles_navigable = to_polar_coords(xpix_navigable, ypix_navigable)
    dists_sample, angles_sample = to_polar_coords(xpix_sample, ypix_sample)
    # mean_dir_navigable = np.mean(angles_navigable)
    # mean_dist_navigable = np.mean(dists_navigable)

    print("current mode:", Rover.mode)
    # Rover.nav_dists = dists_navigable
    # Rover.nav_angles = angles_navigable

    if Rover.mode == RoverMode.FORWARD:
        if np.count_nonzero(sample_mask[sample_mask > 0]) > 10:
            # If sample in sight, approach sample.
            Rover.mode = RoverMode.APPROACH_SAMPLE
            Rover.prev_sample_dists = dists_sample
            Rover.prev_sample_angles = angles_sample
            Rover.nav_dists = Rover.prev_sample_dists
            Rover.nav_angles = Rover.prev_sample_angles
        else:
            Rover.nav_dists = dists_navigable
            Rover.nav_angles = angles_navigable

    elif Rover.mode == RoverMode.APPROACH_SAMPLE:
        if sample_mask.size > 0:
            Rover.nav_dists = dists_sample
            Rover.nav_angles = angles_sample
        else:
            Rover.nav_dists = Rover.prev_sample_dists
            Rover.nav_angles = Rover.prev_sample_angles

    elif Rover.mode == RoverMode.STOP:
        Rover.nav_dists = dists_navigable
        Rover.nav_angles = angles_navigable

    return Rover
