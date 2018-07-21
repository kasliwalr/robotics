import numpy as np
import time
from perception import to_polar_coords

def slice_polar_coords(dist, angles, dist_range, angle_range):
    """
    Returns a subset of pixel radial distances and angles, such that any pixel's radial distance and angle fall within specified range
    :param dist: list of pixel radial distances
    :param angles: list of corresponding pixel angles in degrees
    :param dist_range: binding range for radial distances
    :param angle_range: binding range for angles in degrees
    :return: list of distances and corresponding list of angles
    """
    # find an intersection of indices that satisfy distance and angle thresholds
    indices_slice = list(filter(lambda idx: angles[idx] <= angle_range[1] and \
                                            angles[idx] >= angle_range[0] and \
                                            dist[idx] <= dist_range[1] and \
                                            dist[idx] >= dist_range[0],range(len(angles))))

    # create a list of distance and angles that correspond to above indices
    dist1 = list()
    angles1 = list()
    dist1 = [dist[idx] for idx in indices_slice]
    angles1 = [angles[idx] for idx in indices_slice]

    if len(dist1) == 0:  # if no index found i.e dist1 is empty, return None
        return None, None
    else:
        return dist1, angles1

def no_obstacle_ahead(dist, angles, thresh_dist, angle_thresh, min_dist_from_wall):
    """
    Determines if terrain is navigable and returns a list of navigable-beam bounding angles
    :param dist: array of radial distances
    :param angles: array of polar angles in degrees
    :param thresh: minimum distance threshold
    :param angle_thresh: minimum navigable-beam angular width
    :return: boolean indicating no obstacle (True) and list of tuples, where each tuple is a pair of bounding angles of a navigable region
    """

    # Method for determining beams
    #Create angular bins of size 0.5 degrees. Each bin angle corresponds to a beamlet
    angle_bins = np.linspace(-45, 45, 181)

    # copy dist and angles so as not to modify original
    new_dist = list(dist)
    new_angles = list(angles)

    # return indices to the bin to which each angle corresponds
    indices = np.digitize(new_angles, angle_bins)

    # create a dictionary with bin angle values as keys, and all distances that fall within that bin
    dict_of_dists_in_beamlets = {}
    for ang in angle_bins:
        dict_of_dists_in_beamlets[ang] = list()

    # populate each dictionary key by list of distances that correspond to
    # angles that belong to that key
    count = 0
    for idx in indices:
        if idx > 0 and idx <= len(angle_bins):
            dict_of_dists_in_beamlets[angle_bins[idx - 1]].append(new_dist[count])
        count += 1

    # for dictionary key that have empty list (no angle falls in that beamlet/potentially an obstacle),
    # assign -1
    for key, value in dict_of_dists_in_beamlets.items():
        if len(value) == 0:
            dict_of_dists_in_beamlets[key] = [-1]

    # find continuous beams, composed of consecutive beamlets
    list_of_nav_beamlets = []
    list_of_beamlet_bounding_angles = []
    in_ones = False
    # iterate through key, value pairs in the dictionary
    for key, value in dict_of_dists_in_beamlets.items():
        # create a list of all distances below a certain distance
        tmplist = list(filter(lambda elm: elm < (thresh_dist/abs(np.sin(45*np.pi/180)) + 20), value))
        if len(tmplist) != 0:  # If beamlet has potentially navigable terrain in near field, check for obstacles in the range
            # from threshold to near field end, its max pixel should be in the mid near field. If that's not the case, mid near field for that
            # beamlet is not navigable and therefore that beamlet is excluded
            if np.amax(tmplist) > thresh_dist/abs(np.sin(45*np.pi/180)):
                #list_of_mins.append(1)
                if not in_ones:
                    list_of_beamlet_bounding_angles.append([key, -45])
                    in_ones = True
                else:
                    list_of_beamlet_bounding_angles[-1].pop()
                    list_of_beamlet_bounding_angles[-1].append(key)
            else:
                if in_ones:
                    in_ones = False

        else:
            in_ones = False

    list_of_nav_beam_angles = list(filter(lambda elm: (elm[1] - elm[0]) > angle_thresh, list_of_beamlet_bounding_angles))

    dist1 = []
    angle1 = []

    for elm in list_of_nav_beam_angles:
        for ang in np.linspace(elm[0], elm[1], (elm[1] - elm[0]) // 0.5 + 1):
            dist1.append(15)
            angle1.append(ang)

    return (True if (len(list_of_nav_beam_angles) > 0 and \
                     get_steer_angle(list_of_nav_beam_angles, 5) != None) else False, \
                     list_of_nav_beam_angles)

def get_steer_angle(list_angles_pairs, max_allowed_angle):
    """
    :param list_angles_pairs: list of tuples, where each tuple represents angular bounds of navigable-beam (in degrees)
    :param max_allowed_angle: maximum value for angular direction in degrees. in rover terms, left corresponds to positive, right negative
    :return: steer angle
    """
    #sort list of angle pairs by the second element in the pair (larger or lefter angles)
    list_angles_pairs.sort(reverse = True, key=lambda x:x[1])
    for angle_pair in list_angles_pairs:
        if max_allowed_angle > angle_pair[1]:
            return (angle_pair[1] + angle_pair[0])/2
        elif max_allowed_angle <= angle_pair[1] and max_allowed_angle >= angle_pair[0]:
            return (max_allowed_angle + angle_pair[0])/2
        else:
            pass
    return None

def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if (Rover.nav_angles is not None):
            # determine if obstacle ahead
            no_obs, list_of_angle_pairs = no_obstacle_ahead(Rover.nav_dists, Rover.nav_angles*180/np.pi, 30, 10,40)

            # Forward mode
            if Rover.mode == 'forward':
                # if there is no obstacle, determine steer angle and move in its direction
                if no_obs:
                    Rover.steer = get_steer_angle(list_of_angle_pairs, Rover.max_allowed_left)
                    # if velocity is below maximum allowable velocity, set throttle to max possible
                    if Rover.vel < Rover.max_vel:
                        Rover.throttle = Rover.throttle_set
                        Rover.brake = 0
                    else:  # Else coast
                        Rover.throttle = 0
                        Rover.brake = 0

                # if there is obstacle ahead, brake (in proportion to velocity) and move to stop mode
                else:
                    Rover.throttle = 0
                    Rover.brake = max(Rover.brake_set, Rover.vel)
                    Rover.steer = 0
                    Rover.mode = 'stop'

            # Stop mode
            elif Rover.mode == 'stop':
                # If we're in stop mode but still moving keep braking
                if Rover.vel > 0.1:
                    Rover.throttle = 0
                    Rover.brake = max(Rover.brake_set, Rover.vel)
                    Rover.steer = 0
                # If we're not moving (vel < 0.2) then do something else
                elif Rover.vel <= 0.1:  # stopped
                    # obstacle ahead
                    if not no_obs:
                        Rover.throttle = 0
                        # Release the brake to allow turning
                        Rover.brake = 0
                        # turn to right by 10 degree
                        Rover.steer = -10

                    # no obstacle ahead, therefore increase speed
                    if no_obs:
                        # Set throttle back to stored value
                        Rover.throttle = Rover.throttle_set
                        # Release the brake
                        Rover.brake = 0
                        # Set steer to mean angle
                        Rover.steer = get_steer_angle(list_of_angle_pairs, Rover.max_allowed_left)
                        Rover.mode = 'forward'

            # if rover's velocity is not increasing despite throttle, it is blocked
            elif Rover.mode == 'blocked':
                # Now we're stopped and we have vision data to see if there's a path forward
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                Rover.steer = -20  # Could be more clever here about which way to turn

                Rover.mode = 'forward'

    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
            Rover.throttle = Rover.throttle_set
            Rover.steer = 0
            Rover.brake = 0


    # # If in a state where want to pickup a rock send pickup command
    # if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
    #     Rover.send_pickup = True

    return Rover