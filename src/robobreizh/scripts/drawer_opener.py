from utils import *
import tf
import math
import time

class DrawerOpener:
    def __init__(self):
        print('Drawer Opener')

    def moveto_right_drawers(self):
        move_base_goal(0.0,0.50, -90)

    def moveto_left_drawers(self):
        move_base_goal(0.40,0.50, -90)

    def move_arm(self, joints_pose):
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.allow_replanning(True)
        #move_group.set_workspace([-2.0, -2.0, 2.0, 2.0])
        joint_goal = move_group.get_current_joint_values()

        joints_index = {'arm_lift_joint':0, 'arm_flex_joint':1, 'arm_roll_joint':2, 'wrist_flex_joint':3, 'wrist_roll_joint':4, 'wrist_ft_sensor_frame_joint':5,}
                
        move_group.set_joint_value_target(joints_pose)
        move_group.go(joints_pose, wait=True)
        move_group.stop()

    def move_arm_drawer_bottom(self):
        joint_values_lower_pos = [0.08115144922017052,
        -1.9806082170413877,
        0.014554405428915285,
        0.32878092387203584,
        1.57,
        0.0]

        self.move_arm(joint_values_lower_pos)
    
    def move_arm_drawer_top(self):
        joints_pose_top = [0.2768508471131582,
        -1.7628517972586382,
        0.16632037112523212,
        0.09242120022747358,
        1.57,
        0.0]
        self.move_arm(joints_pose_top)

    def move_distance(self, dist, angle):
        speed = 0.05
        start = time.time()
        end = start + (dist / speed)
            
        while time.time() < end:
            move_base_vel(speed, 0.0, angle)
        
    def approach_drawer(self):
        self.replace_robot_angular(-1.57)
        rospy.sleep(.5)
        self.move_distance(0.12,0)
        rospy.sleep(.5)

        self.replace_robot_angular(-1.57)
        rospy.sleep(.5)


    def approach_drawer_top(self):
        self.replace_robot_angular(-1.57)
        rospy.sleep(.5)

        self.move_distance(0.2,0)

        rospy.sleep(.5)
        self.replace_robot_angular(-1.57)
        rospy.sleep(.5)
    
    def pull_drawer(self):
        speed = -5
        start = time.time()
        dist = 6
        end = start + (dist / -speed)
        while time.time() < end:
            move_base_vel(speed, 0.0, 0.0)
        
    def move_back(self):
        speed = -0.05
        start = time.time()
        dist = 0.2
        end = start + (dist / -speed)
        while time.time() < end:
            move_base_vel(speed, 0.0, 0.0)

    def move_ang(self, angle):
        speed_compute = 5
        start = time.time()
        end = start + (abs(angle) / speed_compute)
        print (end-start)
        
        if abs(angle) <= 5:
            return 0
        
        if angle <= 0:
            speed = speed_compute
        else:
            speed = -speed_compute
        
        print(speed)
            
        while time.time() < end:
            move_base_vel(0.0, 0.0, speed)


    def replace_robot_angular(self, req_angle):
        # TODO: Only works with negative values, but sufficient for RoboCup
        current_ang_rad = whole_body.get_current_joint_values()[2]
        print(current_ang_rad)
        angle_diff_rad = current_ang_rad - req_angle
        angle_diff_deg = math.degrees(angle_diff_rad)
        print(angle_diff_deg)
        self.move_ang(angle_diff_deg)
    

    def open_bottom_right_drawer(self):
        self.moveto_right_drawers()
        
        self.move_arm_drawer_bottom()

        move_hand(1)

        self.approach_drawer()

        move_hand(0)

        self.pull_drawer()

        move_hand(1)

        self.move_back()

        move_arm_init()

    def open_top_right_drawer(self):
        self.moveto_right_drawers()
        
        self.move_arm_drawer_top()

        move_hand(1)

        self.approach_drawer_top()

        move_hand(0)

        self.pull_drawer()

        move_hand(1)

        self.move_back()

        move_arm_init()
    
    def open_bottom_left_drawer(self):
        self.moveto_left_drawers()
        
        self.move_arm_drawer_bottom()

        move_hand(1)

        self.approach_drawer()

        move_hand(0)

        self.pull_drawer()

        move_hand(1)

        self.move_back()

        move_arm_init()

    
    def open_drawers(self):
        # 1. Open right bottom drawer
        self.open_bottom_right_drawer()
        # 2. Open right top drawer
        self.open_top_right_drawer()
        # 3. Open bottom left drawer
        self.open_bottom_left_drawer()
        move_hand(0)