#!/usr/bin/python
# RUN ON LAPTOP USING PYTHON 3.6

import socket
import time
from queue import Queue
import numpy as np
import cv2
import threading
import math
from circle_intersect import circle_intersect
import perpendicular_intersect_circle
from point import Point
from calc_exit import calc_exit_point
#####HSV Colour Ranges#################
#If the point is red (0-10) or (170-180)
redLowMask = (0,120,110)
redHighMask = (25, 255, 255)

#If the circle is blue
blueLowMask = (100, 120, 90)
blueHighMask = (180, 255, 255)

# if the rect is black 
# these need to be double checked TODO
blackLowMask = (0, 0, 0)
blackHighMask = (180, 255, 30)

#If the ball is yellow
yellowLowMask = (30, 50, 70)
yellowHighMask = (35, 160, 255)

tireLowMask = (20, 0, 0)
tireHighMask = (180, 120, 100)

brownLowMask = (10, 15, 160)
brownHighMask = (60, 90, 210)
########################################


class Trailer:
    def __init__(self, center, front_hitch, back_hitch, axel_width, orientation):
        self.center = center
        self.front_hitch = front_hitch
        self.back_hitch = back_hitch
        self.axel_width = axel_width
        self.orientation = orientation # is a unit vecctor in tuple form

    def get_center(self):
        return self.center
    
    def get_front_hitch(self):
        return self.front_hitch
    
    def get_back_hitch(self):
        return self.back_hitch
    
    def get_axel_width(self):
        return self.axel_width

    def get_orientation(self):
        return self.orientation

    def set_center(self, center):
        self.center = center

    def set_front_hitch(self, front_hitch):
        self.front_hitch = front_hitch

    def set_back_hitch(self, back_hitch):
        self.back_hitch = back_hitch

    def set_orientation(self, orientation):
        self.orientation = orientation

class Vehicle:
    def __init__(self, center, back_hitch, axel_width, orientation):
        self.center = center
        self.back_hitch = back_hitch
        self.axel_width = axel_width
        self.orientation = orientation # is unit vector in tuple form
        self.tan_velocity = 150 # TODO test this value

    def get_orientation(self):
        return self.orientation
    
    def get_center(self):
        return self.center
    
    def get_back_hitch(self):
        return self.back_hitch
    
    def get_axel_width(self):
        return self.axel_width
    
    def set_orientation(self, orientation):
        self.orientation = orientation
    
    def set_center(self, center):
        self.center = center

    def set_back_hitch(self, back_hitch):
        self.back_hitch = back_hitch


def distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def unit_vector(p1, p2):
    # Calculate the vector from p1 to p2
    v = Point(p2.x - p1.x, p2.y - p1.y)
    
    # Calculate the magnitude of the vector
    magnitude = math.sqrt(v.x**2 + v.y**2)
    
    # Normalize the vector to get the unit vector
    unit_v = Point(v.x / magnitude, v.y / magnitude)
    
    return unit_v

def rotate_vector(unit_v, theta):
    # Extract the components of the unit vector
    x = unit_v.x
    y = unit_v.y
    
    # Apply the rotation matrix
    x_rotated = x * math.cos(theta) - y * math.sin(theta)
    y_rotated = x * math.sin(theta) + y * math.cos(theta)
    
    return (x_rotated, y_rotated)

def find_valid_configuration(vehicle, trailers):
    # use the back hitch of the vehicle as the starting point
    # append the vehicle to the configuration list 
    # find the the trailer with the hitch closest to the back hitch of the vehicle
    # pop that trailer from the trailers list
    # set the hitch that matched to be the fron hitch and the other hitch of the trailer to be the back hitch
    # append the trailer to the configuration list
    # the back hitch of the trailer is now the new starting point
    # repeat until all trailers are attached
    # return the configuration list\

    configuration = []
    current_hitch = vehicle.get_back_hitch()
    configuration.append(vehicle)

    while len(trailers) > 0:
        # find the trailer with a hitch1 or hitch2 that has the same location as the current hitch
        for trailer in trailers: # TODO: the trailer objects need some work to deal with hit1_2 and front back issues
            if trailer[1] == current_hitch or trailer[2] == current_hitch:
                if trailer[1] == current_hitch:
                    front_hitch = trailer[1]
                    back_hitch = trailer[2]
                elif trailer[2] == current_hitch:
                    front_hitch = trailer[2]
                    back_hitch = trailer[1]

                t_vector = unit_vector(trailer[0],front_hitch)

                
                configuration.append(Trailer(trailer[0], front_hitch, back_hitch, trailer[3], t_vector))
                trailers.remove(trailer)
                current_hitch = back_hitch
                break
    return configuration

def u_trailer_constructor(tracker):#arguements of some kind):
    # this function will be used to find atrributes to create a trailer object
    # it will be called with the information from tracker. 
    # it will detect tires as black rectangles and the hitch as red dots.
    # the center of the trailer is a yellow dot.
    # it will run a radius detection on the yellow dot to determine which tires and hitches belong to it. 
    # it will then find the distance of between the two tires to be the axel_width.
    # the trailer object will have the hitch locations stored in it as as attribute (hitch_1 and hitch_2)
    # the trailer object will have distance between the two tires stored in it as an attribute ( the 2d distance)
    # this trailer constructor is used to create a trailer object that is not attached to a vehicle. the trailers ar oriented as they are attached to the vehicle

    trailer_center_loc = tracker.trailerPoints

    tire_locs = tracker.tires # needs fixing TODO
    hitch_locs = tracker.hitchPoints

    trailer_list = []
    for center in trailer_center_loc:
        # search through the tire locations to find the 2 closest tires to the vehicle center
        tire1 = None
        tire2 = None
        min_dist = float('inf')
        second_min_dist = float('inf')

        for tire in tire_locs:

            dist = distance(tire, center)
            if dist < min_dist:
                tire2 = tire1
                second_min_dist = min_dist
                tire1 = tire
                min_dist = dist
            elif dist < second_min_dist and tire != tire1:
                tire2 = tire
                second_min_dist = dist

        # calculate the length of the axel of the vehicle
        axel_length = distance(tire1, tire2)
        
        # search through the hitch points to find the 2 hitches closest to the expected back hitch location
        hitch1 = None
        hitch2 = None
        min_dist = float('inf')
        second_min_dist = float('inf')


        for hitch in hitch_locs:
            dist = distance(hitch, center)
            if dist < min_dist:
                hitch2 = hitch1
                second_min_dist = min_dist
                hitch1 = hitch
                min_dist = dist
            elif dist <= second_min_dist and hitch != hitch1:
                hitch2 = hitch
                second_min_dist = dist

        trailer_list.append((center, hitch1, hitch2, axel_length))

    return trailer_list

def vehicle_constructor(tracker):#arguements of some kind):
    # this function will be used to find atrributes to create a vehicle object
    # it will call tracker's get location method. 
    # it will detect tires as black rectangles and the hitch as red dots. and the center of vehicle as a blue dot
    # it will run a radius detection on the blue dot to determine which tires and the hitch that belongs to it. 
    # it will then find the center of the the tires and the distance between them is the axel_width.
    # the line from the back hitch to the v_center_loc reprsents the center line of the vehicle. v_theta is the angle between this line and 0 radians
    # the hitches will be the closest red dot cluster to this line at a disctance of hitch_length.
    # the vehicle object will have the hitch locations stored in it as an attribute (front and back)
    # the vehicle object will have distance between the two tires stored in it as an attribute ( the 2d distance)
    # the vehicle object will have the distance from the axel_center point to the hitch stored in it as an attribute
    # the vehicle object will have the orientation of the vehicle stored in it as an attribute
    #TODO
    
    # get the location of the tires, vehicle center and hitches
    V_center_loc = tracker.vehiclePoint
    tire_locs = tracker.tires # needs fixing TODO
    hitch_locs = tracker.hitchPoints

    # search through the tire locations to find the 2 closest tires to the vehicle center
    tire1 = None
    tire2 = None
    min_dist = float('inf')
    second_min_dist = float('inf')
    #print("in side vehicle constructor")
    for tire in tire_locs:
        dist = distance(tire, V_center_loc)
        if dist < min_dist:
            tire2 = tire1
            second_min_dist = min_dist
            tire1 = tire
            min_dist = dist
        elif dist < second_min_dist and tire != tire1:
            tire2 = tire
            second_min_dist = dist
    #print("after for loop")
    # calculate the length of the axel of the vehicle
    axel_length = distance(tire1, tire2)
    
    # search through the hitch points to find the hitch closest to the expected back hitch location
    back_hitch = None
    min_dist = float('inf')
    #print("in side vehicle constructor before hitches")
    for hitch in hitch_locs:
        dist = distance(hitch, V_center_loc)
        if dist < min_dist:
            back_hitch = hitch
            min_dist = dist
    
    # a vector from the back_hitch location to the v_center_loc represents the location and direction of the vehicle
    # v_theta will reprsent the vehicles orientation as from the back hitch to the v_center_loc
    u_vector = unit_vector(back_hitch,V_center_loc)

    #print("v_theta")
    vehicle = Vehicle(V_center_loc, back_hitch, axel_length, u_vector)
    #print("before return vehicle")
    return vehicle

def forward_method(visual):
    # this function will be used to drive the vehicle forward to get the last trailer to the goal point
    # it does this by using standard forward kinematics to drive 2 curves and 2 straight lines to get the trailers and vehicle aligned in a straigh line
    # start in mode 0
    # the vehicle will drive in a forward curved path to the right until the orientation of the vehicle is pointing at the goalpoint with h degree tolerance
    # then mode 1
    # the vehicle will drive in a straigh line towards the goal until the vehicle is at least buffer_distance away from the goalpoint
    # then mode 2
    # the vehicle will drive in a forward curved path to the right until the orientation of the vehicle is pointing away from the goalpoint
    # then mode 3 
    # the vehicle will drive in a forward straight path until the last trailer is at least buffer_distance away from the goalpoint
    # then mode 4
    # it then backs up the configuration in a stable straight line to the goal point
    # the vehicle will stop driving after the last trailer is within goal_threshold of the goal_point
    print("entered forward method")
    configuration = visual.capture_positions()
    goal_detector = False
    goal_threshold = 100

    mode = 0
    
    while mode < 4:
        print(f"current mode {mode}")
        if mode == 0:
            forward_curved_path(configuration,visual, 0)
            mode = 1
        elif mode == 1:
            forward_straight_path(configuration,visual, 1)
            mode = 2
        elif mode == 2:
            forward_curved_path(configuration,visual, 2)
            mode = 3
        elif mode == 3:
            forward_straight_path(configuration,visual, 3)
            mode = 4
    print(f"current mode {mode}")
    goal_detector = distance(configuration[-1].get_center(), visual.tracker.goalPoint)
    configuration = visual.capture_positions()
    while goal_detector>goal_threshold:
        # if the reverse function doesn't work, we simplify this code chunk to providing small time stamp equal velocities to each tire and checking the jack knife detector
        """temp_configuration = [None]*len(configuration) # TODO maybe len -1?
        for unit in range((len(configuration)-1), 0, -1):
            if unit == len(configuration)-1:
                sub_goal = calc_sub_goal(configuration[unit].get_back_hitch(), visual.tracker.goalPoint) # point object at x_s, y_s

                last_trailer = reverse_last_trailer_movement(configuration[unit], sub_goal, visual.tracker.goalPoint)
                temp_configuration[unit] = last_trailer[0]
            elif unit != len(configuration)-1 and unit != 0:
                sub_goal = configuration[unit+1].get_back_hitch() # TODO this has to be changed to account for the exit point of the curve to acount for hitch offset.
                trailer = reverse_mid_trailer_movement(configuration[unit], sub_goal, visual.tracker.goalPoint)
                temp_configuration[unit] = trailer[0]
            else:
                sub_goal = configuration[unit+1].get_back_hitch()
                vehicle = reverse_vehicle_movement(configuration[unit], sub_goal, visual.tracker.goalPoint)
                temp_configuration[unit] = vehicle[0]"""
        
        if jackknife_detector(configuration):
            print("uh oh! jackknife detected!")
            configuration = visual.capture_positions()
            forward_readjustment_path(visual, configuration)
            configuration = visual.capture_positions()
        else:
            print("dist to goal", goal_detector)
            dist = distance(configuration[-1].get_back_hitch(), configuration[-1].get_front_hitch())
            print("reversing distance", dist)
            # calculate the distance the vehicle must drive to get get to the buffer distance
            # calculate the time that will take to get to the buffer distance
            t = (dist / configuration[0].tan_velocity) * (1/visual.pixelPS_per_DPS)
            # calculate the v_l and v_r for the vehicle (should be the same value to drive straight)
            v_l = -dist/t
            v_r = -dist/t
            # move the vehicle
            visual.server.sendStr(f"{v_l}, {v_r}, {t}")
            # wait for vehicle to finish movement and it's response

            #send v_l and v_r to vehicle
            # wait for vehicle to finish movement and it's response
            response = visual.server.pollData()
            goal_detector = distance(configuration[-1].get_center(), visual.tracker.goalPoint)
            configuration = visual.capture_positions()
            time.sleep(t/2)
            if response == "DONE":
                print("Movement executed successfully.")
            elif response == "RESET":
                print("Movement reset requested by client.")
                break
            else:
                print("Unexpected response from client:", response)

    print("Reverse Complete")

def is_within_tolerance(p1, p2, reference_vector):

    h = 5
    # Calculate the vector v1 from p1 to p2
    v1 = (p2[0] - p1[0], p2[1] - p1[1])
    
    # Normalize the vector v1
    v1_magnitude = math.sqrt(v1[0]**2 + v1[1]**2)
    v1_normalized = (v1[0] / v1_magnitude, v1[1] / v1_magnitude)
    
    # Calculate the dot product of the normalized vectors
    dot_product = v1_normalized[0] * reference_vector[0] + v1_normalized[1] * reference_vector[1]
    
    # Calculate the angle between the vectors in degrees
    angle = math.degrees(math.acos(dot_product))
    
    # Check if the angle is within the tolerance h
    return abs(angle) <= h

def forward_curved_path(configuration, visual, mode):
    # this function will be used to drive the vehicle in a forward curved path with a mode toggle of 0 or 2
    # drives the vehicle in a forward curved path to the right
    # the curve is of radius r = 20 cm
    # if mode is 0 the vehicle will stop driving after the orientation of the vehicle is pointing at the goalpoint with h degree tolerance
    # if mode is 2 the vehicle will stop driving after the orientation of the vehicle is pointing away from the goalpoint
    
    # if mode == 0:
    configuration = visual.capture_positions()
    if mode == 0:
        while not is_within_tolerance( configuration[0].get_center(),visual.tracker.goalPoint, configuration[0].get_orientation()):
        # while the the vehicle at configuration[0] does not have a orientation pointing at the goal point with h degree tolerance
            # drive forward along curve to the right of radius r = 20 cm
            R = 4*distance(configuration[-1].get_back_hitch(),configuration[-1].get_center()) # TODO 
            omega = math.pi/6
            d = 0.5*configuration[0].get_axel_width() # TODO conversion to cm? 
            # calculate the v_l and v_r for the vehicle
            v_l = omega * (R + d)
            v_r = omega * (R - d)
            time = 0.5
            # move the vehicle
            visual.server.sendStr(f"{v_l}, {v_r}, {time}")

            # wait for vehicle to finish movement and it's response
            response = visual.server.pollData()
            if response == "DONE":
                print("Movement executed successfully.")
            elif response == "RESET":
                print("Movement reset requested by client.")
                break
            else:
                print("Unexpected response from client:", response)
            configuration = visual.capture_positions()
        return
    # if mode == 2:
    if mode ==2:
        configuration = visual.capture_positions()
        while not is_within_tolerance(configuration[0].get_center(),visual.tracker.goalPoint, rotate_vector(configuration[0].get_orientation(),math.pi)):
            R = 2*distance(configuration[-1].get_back_hitch(),configuration[-1].get_front_hitch()) # TODO 
            omega = math.pi/6
            d = 0.5*configuration[0].get_axel_width() # TODO conversion to cm? 
            # calculate the v_l and v_r for the vehicle
            v_l = omega * (R + d)
            v_r = omega * (R - d)
            time = 0.5
            # move the vehicle
            visual.server.sendStr(f"{v_l}, {v_r}, {time}")

            # wait for vehicle to finish movement and it's response
            response = visual.server.pollData()
            if response == "DONE":
                print("Movement executed successfully.")
            elif response == "RESET":
                print("Movement reset requested by client.")
                break
            else:
                print("Unexpected response from client:", response)
            configuration = visual.capture_positions()
        return   

def forward_straight_path(configuration, visual, mode):
    #TODO
    # this function will be used to drive the vehicle in a forward straight path with a mode toggle of 0 or 1
    # if mode is 1 the vehicle will stop driving after the vehicle is at least buffer_distance away from the goalpoint
    # if mode is 3 the vehicle will stop driving after driving straight for a distance of Length_of_config
    buffer_distance = 400 # TODO test this value
    # if mode == 1:
    
    if mode == 1:
        print("entered forward path")
        # while the vehicle at configuration[0] is greater than or equal to buffer_distance away from the goalpoint
        while distance(configuration[0].get_center(),visual.tracker.goalPoint) > buffer_distance:
            dist = distance(configuration[0].get_center(),visual.tracker.goalPoint)-buffer_distance
            # calculate the distance the vehicle must drive to get get to the buffer distance
            # calculate the time that will take to get to the buffer distance
            time = (dist / configuration[0].tan_velocity) * (1/visual.pixelPS_per_DPS)
            # calculate the v_l and v_r for the vehicle (should be the same value to drive straight)
            v_l = dist/time
            v_r = dist/time
            # move the vehicle
            visual.server.sendStr(f"{v_l}, {v_r}, {time}")

            # wait for vehicle to finish movement and it's response
            response = visual.server.pollData()
            if response == "DONE":
                print("Movement executed successfully.")
            elif response == "RESET":
                print("Movement reset requested by client.")
                break
            else:
                print("Unexpected response from client:", response)
            configuration = visual.capture_positions()
        return            

    # if mode == 3:
    if mode == 3:
        # calculate the total length of the configuration by summing of the distances between the front and back hitches of each trailer and add the vehicle to back hitch distance
        length = 0
        for unit in range(len(configuration)-1,1,-1):
            length += distance(configuration[unit].get_back_hitch(), configuration[unit].get_front_hitch())
        length += 2*distance(configuration[0].get_back_hitch(), configuration[0].get_center())
        # calcualte the time to drive this length.
        time = length/configuration[0].tan_velocity
        # calculate the v_l and v_r for the vehicle (should be the same value to drive straight)
        v = configuration[0].tan_velocity*(1/visual.pixelPS_per_DPS)
        # move the vehicle
        visual.server.sendStr(f"{v}, {v}, {time}")

        # wait for vehicle to finish movement and it's response
        response = visual.server.pollData()
        if response == "DONE":
            print("Movement executed successfully.")
        elif response == "RESET":
            print("Movement reset requested by client.")
        else:
            print("Unexpected response from client:", response)
        configuration = visual.capture_positions()
    return

def forward_readjustment_path(visual, configuration):
    #TODO
    # this function will be used to drive the vehicle forward to straighten out the configuation
    # the vehicle will drive forward for a distance of adjust_distance
    adjust_distance = distance(configuration[-1].get_back_hitch(),configuration[-1].get_front_hitch()) #TODO check this value
    time = adjust_distance/configuration[0].tan_velocity
    v = configuration[0].tan_velocity*(1/visual.pixelPS_per_DPS)
    visual.server.sendStr(f"{v}, {v}, {time}")

    # wait for vehicle to finish movement and it's response
    response = visual.server.pollData()
    if response == "DONE":
        print("Movement executed successfully.")
    elif response == "RESET":
        print("Movement reset requested by client.")
    else:
        print("Unexpected response from client:", response)
    configuration = visual.capture_positions()
    
    return
    
def jackknife_detector(configuration):
    # This function will be used to detect if the vehicle is in a jackknife position
    # The jackknife position is when the units in the trailer are at angles greater than the specified tolerance
    # The function will return True if the vehicle is in a jackknife position
    # The function will return False if the vehicle is not in a jackknife position
    
    tolerance = math.pi/5 # TODO will have to be tested.
    for i in range(len(configuration) - 1):
        # Get the unit vectors of the two adjacent units
        unit_vector_1 = configuration[i].get_orientation()
        unit_vector_2 = configuration[i + 1].get_orientation()
        
        # Calculate the dot product of the unit vectors
        dot_product = unit_vector_1[0] * unit_vector_2[0] + unit_vector_1[1] * unit_vector_2[1]
        
        # Calculate the angle between the vectors in degrees
        angle = math.acos(dot_product)
        print(angle)
        # Check if the angle is greater than the specified tolerance
        if angle > tolerance:
            return True
    return False 


def reverse(visual):
    #TODO
    # this function will be used to drive the vehicle in reverse to get the last trailer to the goal point
    # it does this by using a nonstandard kinematic model for differential steering to calculate the v_l and v_r for the vehicle
    # while last trailer is not at the goal point do loop
        # if the jackknife_detector returns true the vehicle will call forward_readjustment_path and break the loop
        # the last trailer is projected to move its back_hitch to the goal point by 1 small unit.
        # the differential model of steering is applyied and solved in inverse to get the v_l and v_r for the trailer and to project where it's front hitch would be
        # for each remaining trailer in the configuration, their back hitch is moved to the front hitch of the trailer that was just calculated
        # the vehicle is then moved to the front hitch of the last trailer
        # this produces the v_l and v_r for the vehicle
        # the v_l and v_r are then used to move the vehicle
    configuration = visual.capture_positions()
    goal_detector = False
    goal_threshold = 100
    goal_detector = distance(configuration[-1].get_center(), visual.tracker.goalPoint)
    while goal_detector>goal_threshold:
        temp_configuration = [None]*len(configuration) #TODO check this in forward method, might need to be len-1?
        for unit in range( (len(configuration)-1), 0, -1):
            if unit == len(configuration)-1:
                sub_goal = calc_sub_goal(configuration[unit].get_back_hitch(), visual.tracker.goalPoint) # point object at x_s, y_s

                last_trailer = reverse_last_trailer_movement(configuration[unit], sub_goal, visual.tracker.goalPoint)
                temp_configuration[unit] = last_trailer[0]
            elif unit != len(configuration)-1 and unit != 0:
                sub_goal = configuration[unit+1].get_back_hitch() # TODO this has to be changed to account for the exit point of the curve to acount for hitch offset.
                trailer = reverse_mid_trailer_movement(configuration[unit], sub_goal, visual.tracker.goalPoint)
                temp_configuration[unit] = trailer[0]
            else:
                sub_goal = configuration[unit+1].get_back_hitch()
                vehicle = reverse_vehicle_movement(configuration[unit], sub_goal, visual.tracker.goalPoint)
                temp_configuration[unit] = vehicle[0]
        if jackknife_detector(temp_configuration):
            print("uh oh! jackknife detected!")
            forward_readjustment_path(visual, configuration)
            break
        else:
            configuration = temp_configuration
            goal_detector = distance(configuration[-1].get_center(), visual.tracker.goalPoint)
            print("step towards goal done! Goal Detector distance!: ", goal_detector)
            print("configuration: ", configuration)
            print("Sending v_l and v_r to vehicle")
            #send v_l and v_r to vehicle
            v_l = vehicle[1]*(1/visual.pixelPS_per_DPS)
            v_r = vehicle[2]*(1/visual.pixelPS_per_DPS)
            time = vehicle[3]
            visual.server.sendStr(f"{v_l}, {v_r}, {time}")

            # wait for vehicle to finish movement and it's response
            response = visual.server.pollData()
            if response == "DONE":
                print("Movement executed successfully.")
            elif response == "RESET":
                print("Movement reset requested by client.")
                break
            else:
                print("Unexpected response from client:", response)

    print("Reverse Complete")

def calc_sub_goal(back_hitch, goal_point):
    #TODO
    # this function will be used to calculate the sub goal for the last trailer
    # the sub goal is the point that is 1 small unit away from the hitch of the last trailer in the direction of the goal point
    # the sub goal is calculated by making a unit_vector from the back hitch to the goal point
    # this vector from the back hitch to the goal point is then multiplied by a small scalar_unit
    # the scalar_unit*unit_vector is then added to the back hitch to get the sub goal
    unit_vector = np.array([goal_point.x - back_hitch.x, goal_point.y - back_hitch.y])/distance(goal_point, back_hitch)
    scalar = 1 # TODO test this scalar, may need to be bigger
    sub_goal = Point(back_hitch.x + scalar*unit_vector[0], back_hitch.y + scalar*unit_vector[1])
    return sub_goal

def reverse_last_trailer_movement(unit, subgoal, goalPoint):
    # this function will be used to calculate the movement of the last trailer in reverse, it will return a new sub_goal for the next trailer and the new prospective trailer
    # the last trailer will move its back hitch to the subgoal along an arc but stopping when its center is at the exit point, with the orientation of the goal being tangent to the curve
    # this tangent lines up with the goal point

    exit_point = calc_exit_point(unit = unit,subgoal=subgoal, goal=goalPoint, type="last") # gets exit point from curve to align with goal via tangent of curve -> goal

    theta_si = math.atan2(exit_point.y-unit.get_center().y, exit_point.x - unit.get_center().x)

    Radius = ((1/2)*distance(exit_point,unit.get_center()))/math.cos(theta_si)

    theta_xi = (math.pi - math.pi/2 - theta_si)*2

    unit_vector = rotate_vector(unit.get_orientation(), theta_xi) # TODO this may be wrong and we may have to  check that the theta is in the right quadrant

    front_hitch = unit.get_front_hitch()
    back_hitch = unit.get_back_hitch()
    trailer_length = distance(front_hitch, back_hitch)
    front_hitch = Point(exit_point.x + (trailer_length/2)*unit_vector[0], exit_point.y + (trailer_length/2)*unit_vector[1])
    back_hitch = Point(exit_point.x - (trailer_length/2)*unit_vector[0], exit_point.y - (trailer_length/2)*unit_vector[1])

    new_trailer = Trailer(exit_point, front_hitch, back_hitch, unit.get_axel_width(), unit_vector)

    return (new_trailer,new_trailer.get_back_hitch())

def reverse_mid_trailer_movement(unit, subgoal, goalPoint):
    # this function will be used to calculate the movement of the mid trailer in reverse, it will return a new sub_goal for the next trailer and the new prospective trailer
    # the mid trailer will move its back hitch to the subgoal along an arc
    # this tangent lines up with the goal point

    exit_point = calc_exit_point(unit,subgoal, goalPoint, "middle" ) # gets exit point from curve to align with goal via tangent of curve -> goal

    theta_si = math.atan2(exit_point.y-unit.get_center().y, exit_point.x - unit.get_center().x)

    Radius = ((1/2)*distance(exit_point,unit.get_center()))/math.cos(theta_si)

    theta_xi = (math.pi - math.pi/2 - theta_si)*2

    unit_vector = rotate_vector(unit.get_orientation(), theta_xi) # TODO this may be wrong and we may have to  check that the theta is in the right quadrant

    front_hitch = unit.get_front_hitch()
    back_hitch = unit.get_back_hitch()
    trailer_length = distance(front_hitch, back_hitch)
    front_hitch = Point(exit_point.x + (trailer_length/2)*unit_vector[0], exit_point.y + (trailer_length/2)*unit_vector[1])
    back_hitch = Point(exit_point.x - (trailer_length/2)*unit_vector[0], exit_point.y - (trailer_length/2)*unit_vector[1])

    new_trailer = Trailer(exit_point, front_hitch, back_hitch, unit.get_axel_width() , unit_vector)

    return (new_trailer,new_trailer.get_back_hitch())
    

def reverse_vehicle_movement(unit, subgoal, goalPoint):
    # this function will be used to calculate the movement of the vehicle in reverse
    # the vehicle will move its back hitch to the subgoal along an arc

    exit_point = calc_exit_point(unit,subgoal, goalPoint, "vehicle") # gets exit point from curve to align with goal via tangent of curve -> goal

    theta_si = math.atan2(exit_point.y-unit.get_center().y, exit_point.x - unit.get_center().x)

    Radius = ((1/2)*distance(exit_point,unit.get_center()))/math.cos(theta_si)

    theta_xi = (math.pi - math.pi/2 - theta_si)*2

    unit_vector = rotate_vector(unit.get_orientation(), theta_xi) # TODO this may be wrong and we may have to  check that the theta is in the right quadrant

    center = unit.get_center()
    back_hitch = unit.get_back_hitch()
    vehicle_length = distance(center, back_hitch)
    back_hitch = Point(exit_point.x - (vehicle_length/2)*unit_vector[0], exit_point.y - (vehicle_length/2)*unit_vector[1])

    new_vehicle = Vehicle(exit_point, back_hitch, unit.get_axel_width(), unit_vector)

    arc_len = theta_xi*Radius

    time = arc_len / unit.tan_velocity

    omega = theta_si/time

    v_l = omega * (Radius - unit.get_axel_width()/2)
    v_r = omega * (Radius + unit.get_axel_width()/2)

    return (new_vehicle, v_l, v_r, time)


class Tracker:

    def __init__(self, vehicleColor, trailerColor, hitchColor, goalColor):
        self.vehiclePoint = None
        self.trailerPoints = []
        self.hitchPoints = []
        self.goal = None
        self.tires = []
        self.goalPoint = None
        self.ready = False
        self.vc = cv2.VideoCapture(0, cv2.CAP_DSHOW) #DO NOT USE CAP_DSHOW IN DEMO
        self.vc.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
        self.vc.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
        self.vc.set(cv2.CAP_PROP_FPS, 30)
        width = self.vc.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.vc.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"Resolution set to: {width}x{height}")
        
        self.thread = threading.Thread(target=self.TrackerThread, args=(vehicleColor, trailerColor, hitchColor, goalColor), daemon=True)
        self.thread.start()
        
    

    def TrackerThread(self, vehicleColor, trailerColor, hitchColor, goalColor):
        print("Tracker Started")
        # Get the camera
        
        vc = self.vc
        if vc.isOpened(): # try to get the first frame
            rval, frame = vc.read()
        else:
            rval = False
        print("press q if largest and only rectangle is goal")
        while True:
            # Capture each frame
            ret, frame = vc.read()

            # If the frame was captured successfully
            if ret:
                # Display the frame in a window named 'Webcam'
                center = frame.shape
                y_center = center[1]/2
                x_center = center[0]/2
                scale = 0.6

                y_low = y_center - y_center*scale
                y_high = y_center + y_center*scale
                x_low = x_center - x_center*scale
                x_high = x_center + x_center*scale
                #frame = frame[int(x_low-100):int(x_high+100), int(y_low+60):int(y_high-60)] #crop frame so it only contains green carpet

                frame = frame[int(x_low):int(x_high), int(y_low+110):int(y_high-110)]
                

                RectGoal = self.detect_rectangles(frame, "w", strictRect=True)
                self.draw_rectangles(frame, RectGoal)
                frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
                cv2.imshow("Detecting Goal", frame)
                
                # Break the loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.goal = RectGoal[0]
                    self.goalPoint = RectGoal[0][1]
                    cv2.destroyAllWindows()
                    break
            else:
                print("Error: Could not retrieve frame.")
                break
        
        while rval:
            # Handle current frame
            try:
                rval, frame = vc.read()
                center = frame.shape
                y_center = center[1]/2
                x_center = center[0]/2
                scale = 0.6

                y_low = y_center - y_center*scale
                y_high = y_center + y_center*scale
                x_low = x_center - x_center*scale
                x_high = x_center + x_center*scale
                #frame = frame[int(x_low-100):int(x_high+100), int(y_low+60):int(y_high-60)] #crop frame so it only contains green carpet

                frame = frame[int(x_low):int(x_high), int(y_low+110):int(y_high-110)]

                vehiclePoint = self.GetLocation(frame, vehicleColor)
                self.vehiclePoint = vehiclePoint[0][0]
                self.vehiclePoint = Point(x = self.vehiclePoint[0], y = self.vehiclePoint[1])

                trailerPoints = self.GetLocation(frame, trailerColor)
                self.trailerPoints = []
                for i in trailerPoints[0]:
                    self.trailerPoints.append(Point(x = i[0], y = i[1]))

                hitchPoints = self.GetLocation(frame, hitchColor)
                self.hitchPoints = []
                for i in hitchPoints[0]:
                    self.hitchPoints.append(Point(x = i[0], y = i[1]))

                self.DrawCircles(frame, vehiclePoint, (255, 0, 0))
                self.DrawCircles(frame, trailerPoints, (0, 255, 0))
                self.DrawCircles(frame, hitchPoints, (0, 0, 255))
                
                tires = self.detect_rectangles(frame, "Tire")
                self.tires = tires
                self.draw_rectangles(frame, tires)
                
                temp = []
                for tire in tires:
                    temp.append(tire[1])
                self.tires = temp

                self.draw_rectangles(frame, RectGoal)
                # pretty sure these is staments only update one point and goal at a time so we have to updtate them to show all points and the goal

                # Shows the original image with the detected circles drawn.
                frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
                cv2.imshow("Result", frame)

                # check if esc key pressed
                key = cv2.waitKey(20)
                if key == 27:
                    break
                self.ready = True
            except:
                continue
        
        vc.release()
        cv2.destroyAllWindows()
        print("Tracker Ended")

    def GetLocation(self, frame, color): #gets circles
    # Uncomment for gaussian blur
    #blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        blurred = cv2.medianBlur(frame,11)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if color == 'r':
            # Red Tracking
            mask = cv2.inRange(hsv, redLowMask, redHighMask)
        if color == 'b':
            # Blue Tracking
            mask = cv2.inRange(hsv, blueLowMask, blueHighMask)
        if color == 'k':
            # Green Tracking
            mask = cv2.inRange(hsv, blackLowMask, blackHighMask)
        if color == 'y':
            # Yellow Tracking
            mask = cv2.inRange(hsv, yellowLowMask, yellowHighMask)
        if color == 'w':
            # brown Tracking
            mask = cv2.inRange(hsv, brownLowMask, brownHighMask)
        
        # Perform erosion and dilation in the image (in 11x11 pixels squares) in order to reduce the "blips" on the mask
        mask = cv2.erode(mask, np.ones((3, 3), np.uint8), iterations=2)
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=3)
        
        
        # Mask the blurred image so that we only consider the areas with the desired colour
        masked_blurred = cv2.bitwise_and(blurred,blurred, mask= mask)
        # masked_blurred = cv2.bitwise_and(frame,frame, mask= mask)
        # Convert the masked image to gray scale (Required by HoughCircles routine)
        result = cv2.cvtColor(masked_blurred, cv2.COLOR_BGR2GRAY)
        
        # Detect circles in the image using Canny edge and Hough transform
        circles = cv2.HoughCircles(result, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=8, minRadius=4, maxRadius=20)
        result = cv2.resize(result, (0, 0), fx=0.5, fy=0.5)
        return circles #returns (x, y r) of circles
            
    def DrawCircles(self, frame, circles, dotColor = (255, 0, 0)):
    # ensure at least some circles were found
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                #print("Circle: " + "("+str(x)+","+str(y)+")")
                # draw the circle in the output image, then draw a rectangle corresponding to the center of the circle
                # The circles and rectangles are drawn on the original image.
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), dotColor, -1)

    def detect_rectangles(self, frame, color, strictRect = False):
        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define color ranges
        if color == 'b':
            mask = cv2.inRange(hsv, blueLowMask, blueHighMask)
        elif color == 'y':
            mask = cv2.inRange(hsv, yellowLowMask, yellowHighMask)
        elif color == 'r':
            mask = cv2.inRange(hsv, redLowMask, redHighMask)
        elif color == 'k':
            mask = cv2.inRange(hsv, blackLowMask, blackHighMask)
        elif color == "Tire":
             mask = cv2.inRange(hsv, tireLowMask, tireHighMask)
        elif color == "w":
            mask = cv2.inRange(hsv, brownLowMask, brownHighMask)
        else:
            return []

        # Perform erosion and dilation to reduce noise
        mask = cv2.erode(mask, np.ones((3, 3), np.uint8), iterations=5)
        mask = cv2.dilate(mask, np.ones((6, 6), np.uint8), iterations=5)

        
        # Find contours in the masked image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        rectangles = []
        for contour in contours:
            # Approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            # If the polygon has 4 vertices, it is a rectangle
            if (strictRect and len(approx) == 4) or not strictRect:
                # Calculate the center point of the rectangle
                M = cv2.moments(approx)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    center = Point(cX, cY)
                    rectangles.append((approx, center))

        return rectangles

    def draw_rectangles(self, frame, rectangles):
        if rectangles == []:
            return
        for rect in rectangles:
            cv2.drawContours(frame, [rect[0]], -1, (0, 0, 255), 2)



# This class handles the Server side of the communication between the laptop and the brick.
class Server:
    def __init__(self, host, port):
        # setup server socket
        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        # We need to use the IP address that shows up in ipconfig for the USB ethernet adapter that handles the communication between the PC and the brick
        print("Setting up Server\nAddress: " + host + "\nPort: " + str(port))
        
        serversocket.bind((host, port))
        # queue up to 5 requests
        serversocket.listen(5) 
        self.cs, addr = serversocket.accept()
        print("Connected to: " + str(addr))
        pass

    # Sends set of angles to the brick via TCP.
    # Input: base_angle [Float]: The angle by which we want the base to move
    #        joint_angle [Float]: The angle by which we want to joint to move
    #        queue [Thread-safe Queue]: Mutable data structure to store (and return) the messages received from the client
    def sendAngles(self, base_angle, joint_angle, queue):
        # Format in which the client expects the data: "angle1,angle2"

        data = str(base_angle) + "," + str(joint_angle)
        print("Sending Data: (" + data + ") to robot.")
        self.cs.send(data.encode("UTF-8"))
        # Waiting for the client (ev3 brick) to let the server know that it is done moving
        reply = self.cs.recv(128).decode("UTF-8")
        queue.put(reply)

    def sendStr(self, string):
        print("Sending Data: (" + string + ") to robot.")
        self.cs.send(string.encode("UTF-8"))

    # Sends a termination message to the client. This will cause the client to exit "cleanly", after stopping the motors.
    def sendTermination(self):
        self.cs.send("EXIT".encode("UTF-8"))

    # Lets the client know that it should enable safety mode on its end
    def sendEnableSafetyMode(self):
        self.cs.send("SAFETY_ON".encode("UTF-8"))
    
    # Lets the client know that it should disable safety mode on its end
    def sendDisableSafetyMode(self):
        self.cs.send("SAFETY_OFF".encode("UTF-8"))

    def pollData(self):
        print("Waiting for Data")
        data = self.cs.recv(128).decode("UTF-8")
        print("Data Received")
        return data


class VisualServoing:
    def __init__(self, server):
        self.vehicle_col = "b"
        self.goal_pos_col = "w"
        self.trailer_col = "y"
        self.hitch_col = "r"
        self.pixelPS_per_DPS = None
        self.server = server
        self.queue = Queue()
        
        self.tracker = Tracker(self.vehicle_col,self.trailer_col, self.hitch_col, self.goal_pos_col )  # Initialize tracker for vehicle, tailers and goal
        while not self.tracker.ready:
            time.sleep(0.5)
        self.stop_threshold = 75
        self.caliberate_velocity()
        

        
    def capture_positions(self, r = False):
        while True:
            try:
                rval, frame = self.tracker.vc.read()
                if rval:
                    vehicle_pos = self.tracker.GetLocation(frame, self.vehicle_col)
                    goal_pos = self.tracker.GetLocation(frame, self.goal_pos_col)
                    trailer_pos = self.tracker.GetLocation(frame, self.trailer_col)
                    hitch_pos = self.tracker.GetLocation(frame, self.hitch_col)

                    if vehicle_pos is not None and goal_pos is not None and trailer_pos is not None and hitch_pos is not None:
                        """vehicle = Point(vehicle_pos[0][0][0], vehicle_pos[0][0][1])
                        hitches = [Point(hitch[0], hitch[1]) for hitch in hitch_pos[0]]
                        trailers = [Trailer(Point(trailer[0], trailer[1]), None, None) for trailer in trailer_pos[0]]
                        trailer_length = 100  # Example length, adjust as needed"""

                        vehicle = vehicle_constructor(self.tracker)

                        trailers = u_trailer_constructor(self.tracker)
                        configuration = find_valid_configuration(vehicle, trailers)                   
                        if configuration and len(configuration) > 2:
                            print("Valid configuration found:")
                            for trailer in configuration:
                                print(f"Trailer center: ({trailer.center.x}, {trailer.center.y})")
                                return configuration
                        else:
                            continue
                            print("No valid configuration found.")
                else:
                    continue
            except:
                continue

    def compute_error(self, current_pos, desired_pos):
        return desired_pos - current_pos
    
    def run(self):
        self.capture_positions()
        mode = 0 #TODO change me to wanted method of reversing
        if mode == 0:
            forward_method(self)
        elif mode == 1:
            reverse(self)

    def caliberate_velocity(self):
        #caliberation parameters
        if self.tracker.ready:
            speedDPS = 75
            t = 10

            current_pos = self.tracker.vehiclePoint
            self.server.sendStr(f"{speedDPS},{speedDPS},{t}")
            time.sleep(t+3)
            new_pos = self.tracker.vehiclePoint
            dist_traveled = distance(current_pos, new_pos)
            velocity = dist_traveled/t
            pixelPS_per_DPS = velocity/speedDPS
            self.pixelPS_per_DPS = pixelPS_per_DPS
            print(f"pixels travelled: {dist_traveled}, velocity ratio:{pixelPS_per_DPS}")
        return pixelPS_per_DPS
        


def main():
    # Create a server object
    host = "192.168.0.2"
    port = 9999
    server = Server(host, port)
    queue = Queue()
    visual = VisualServoing(server)


    visual.run()

if __name__ == "__main__":
    main()