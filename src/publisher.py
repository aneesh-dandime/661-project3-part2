#!/usr/bin/python3
from typing import List, Tuple
import rospy
import sys
from controller import controller

class Robot():
    def __init__(self, rpm1, rpm2, clearance):
        self.RPM1, self.RPM2 = rpm1, rpm2
        self.clearence =  clearance
        self.radius, self.wheelDistance, self.dt = 0.038, 0.354, 0.1 

def read_data(path: str) -> Tuple[List[List[float]], List[List[float]]]:
    points = []
    actions = []
    with open(path, 'r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines):
            if i == 0:
                continue
            values = list(map(float, line.split()))
            points.append([values[0], values[1]])
            if i > 1:
                actions.append([values[2], values[3]])
    return points, actions
   
if __name__ == '__main__':
    try:
        clearance = float(sys.argv[6]) / 1000.0
        rpm1, rpm2 = int(sys.argv[7]), int(sys.argv[8])
        file_path = sys.argv[9]
        robot = Robot(rpm1, rpm2, clearance)
        waypoints, actions = read_data(file_path)
        waypoints = [[wp[0] / 1000, wp[1] / 1000] for wp in waypoints]
        if len(actions) == 0:
            exit()
        rospy.sleep(3)
        controller(robot, (actions, waypoints))            

    except rospy.ROSInterruptException:
        pass