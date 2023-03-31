#Francesco Huber
import random
import rclpy
from rclpy.node import Node
from rclpy.task import Future

import sys
from math import pow, sin, cos, atan2, sqrt

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, Spawn, Kill

class TurtleManagerNode(Node):
    def __init__(self, goal_pose, tolerance, name='turtle1'):
        # Creates a node with name 'move2goal'
        super().__init__('TurtleManage'+ str(name))
        
        # Create attributes to store the goal and current poses and tolerance
        self.goal_pose = goal_pose
        self.tolerance = tolerance
        self.current_pose = None

        # Create a publisher for the topic '/turtle1/cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, '/'+ str(name) +'/cmd_vel', 10)

        # Create a subscriber to the topic '/turtle1/pose', which will call self.pose_callback every 
        # time a message of type Pose is received
        self.pose_subscriber = self.create_subscription(Pose, '/'+ str(name) +'/pose', self.pose_callback, 10)

        ### CUSTOM ### 
        self.state = "Drawing"
        self.tname = name
        self.list = get_usi_list()            # list of lists of poses
        self.pose_list_idx = 0                # corresponding list index, at what letter are we at?
        self.pose_letter_idx = 0              # in a list of a letter eg:"u", corresponding pose index
        self.currentList = self.list[0]       # current letter list consisting of poses
        self.enemy_pose = None                # Position of turtle 1 which is teleoperated
        self.previous_pose = None             # Useful to reset goal pose after pursuit of enemy 
        self.lastPoppedElement = [0,0]
        self.angrytime = 0                    # cycles spent angry
        self.poseBeforeAngry = self.goal_pose #
        self.readyToKill = False              # Flag for killing operations
        self.resetting = False                # reset flag to avoid other chasing encounters
        self.enemies = self.get_enemy_list()  # List of enemy turtle names
        self.enemyDict = {}                   # dictionary<name, Pose> of the enemies
        self.currentEnemy = 'turtle1'         # if angry, turtle to pursue
        self.currentEnemyIdx = 0              # current enemy idx in the list
        self.tempPose = None
        
        print(self.tname, "enemies:", self.enemies)

        # Clients
        self.cli = self.create_client(SetPen, '/'+ str(name) +'/set_pen') # pen service
        self.spa = self.create_client(Spawn, '/spawn')
        self.kill = self.create_client(Kill, '/kill') # kill service

        self.send_request_pen(200,200,200,2,1) #turn off pen at init

        self.enemy_pose_subscriber = self.create_subscription(Pose, '/'+ str(self.currentEnemy) +'/pose', self.test_enemy_poses_callback, 10)

        
    def start_moving(self):
        
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(0.1, self.move_callback)

        ### CUSTOM ### 
        # This timer should also give a chance to the other turtle to escape
        #self.timerDist = self.create_timer(0.5, self.get_enemy_distance)

        self.timerDist = self.create_timer(0.5, self.get_enemy_distance_test)

        #timer for detecting new enemies
        self.timerEnemy = self.create_timer(2, self.get_enemy_list)

        #timer for printing the current dictionary
        self.timerDict = self.create_timer(2, self.print_dict)
        
        # Create a Future object which will be marked as "completed" once the turtle reaches the goal
        self.done_future = Future()
        
        return self.done_future
        
    def pose_callback(self, msg):
        """Callback called every time a new Pose message is received by the subscriber."""
        self.current_pose = msg
        self.current_pose.x = round(self.current_pose.x, 4)
        self.current_pose.y = round(self.current_pose.y, 4)
        
    def move_callback(self):
        """Callback called periodically by the timer to publish a new command."""
        nextpose = Pose()

        if self.current_pose is None:
            # Wait until we receive the current pose of the turtle for the first time
            return

        if self.state == "Angry":
                if self.angrytime == 0: #first time being angry
                    self.send_request_pen(200,200,200,2,1)
                    self.poseBeforeAngry = self.current_pose
                                                                                          # List Manipulation Trick  
                    self.currentList.insert(0, [self.goal_pose.x, self.goal_pose.y])      # add goal pose to current list as first elem
                    self.currentList.insert(0, [self.current_pose.x, self.current_pose.y])# add current to restart from there
                    self.list.insert(self.pose_list_idx,[])                               # add an empty list at current index in general list
                    self.currentList = self.list[self.pose_list_idx]                      # set current list to the empty just created
                                                                                          # Once angry is over, the turtle will go to next list 
                self.angrytime += 1

                #INTERCEPTION
                nextpose = self.enemy_pose    
                #self.goal_pose = self.get_intercepted_pose(nextpose, 3)
                self.goal_pose = self.enemy_pose 
 

                if self.readyToKill == True:
                    self.readyToKill = False
                    self.send_request_kill(self.currentEnemy) #send request kill turtle 1
                    #self.enemy_pose = None
                    self.state = 'Drawing'
                    self.goal_pose = self.poseBeforeAngry
                    self.get_logger().info('resetting')
                    self.resetting = True

   
        if self.euclidean_distance(self.goal_pose, self.current_pose) >= self.tolerance:
            # We still haven't reached the goal pose. Use a proportional controller to compute velocities
            # that will move the turtle towards the goal (https://en.wikipedia.org/wiki/Proportional_control)
        
            # Twist represents 3D linear and angular velocities, in turtlesim we only care about 2 dimensions:
            # linear velocity along the x-axis (forward) and angular velocity along the z-axis (yaw angle)\
            cmd_vel = Twist() 
            cmd_vel.linear.x = self.linear_vel(self.goal_pose, self.current_pose)
            cmd_vel.angular.z = self.angular_vel(self.goal_pose, self.current_pose)
            
            # Publish the command
            self.vel_publisher.publish(cmd_vel)


            
        else:
            self.get_logger().info("Goal reached")

            if self.state == "Drawing":
                self.angrytime = 0 # resetter!


                # If current list is empty, then move to the next
                if len(self.currentList) == 0:                      
                    self.send_request_pen(200,200,200,2,0)

                    if self.resetting == True:
                        self.send_request_pen(200,200,200,2,1)
                        self.resetting = False


                    self.get_logger().info("Finished Letter index " + str(self.pose_list_idx)) 
                    self.pose_list_idx += 1
                    if self.pose_list_idx < len(self.list):
                        self.currentList = self.list[self.pose_list_idx]
                        self.pose_letter_idx = 0     
                    else:
                        # Mark the future as completed, which will shutdown the node
                        self.get_logger().info("Finished Drawing")
                        self.done_future.set_result(True)
                        return

                self.lastPoppedElement = self.currentList.pop(0) 
                posvals = self.lastPoppedElement
                nextpose = Pose()
                nextpose.x = float(posvals[0])
                nextpose.y = float(posvals[1])

                # When transitioning from to a letter -> turn off pen.  
                # Reactivate pen once moving to 1st pose of current letter
                if self.pose_letter_idx == 0: 
                    self.send_request_pen(200,200,200,2,1)
                elif self.pose_letter_idx == 1:
                    self.send_request_pen(200,200,200,2,0)

                self.pose_letter_idx += 1
                

            self.goal_pose = nextpose
            self.previous_pose = None
      
    def euclidean_distance(self, goal_pose, current_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - current_pose.x), 2) +
                    pow((goal_pose.y - current_pose.y), 2))

    def angular_difference(self, goal_theta, current_theta):
        """Compute shortest rotation from orientation current_theta to orientation goal_theta"""
        return atan2(sin(goal_theta - current_theta), cos(goal_theta - current_theta))

    def linear_vel(self, goal_pose, current_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose, current_pose)

    def steering_angle(self, goal_pose, current_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x)

    def angular_vel(self, goal_pose, current_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        goal_theta = self.steering_angle(goal_pose, current_pose)
        return constant * self.angular_difference(goal_theta, current_pose.theta)

    #CUSTOM
    def enemy_pose_callback(self, msg):
        """Callback called every time a enemyPose message is received by the subscriber."""
        # self.enemy_pose_subscriber.topic = '/'+ str(self.enemies[self.currentEnemy]) +'/pose'
        self.enemy_pose = msg
        self.enemy_pose.x = round(self.enemy_pose.x, 4)
        self.enemy_pose.y = round(self.enemy_pose.y, 4)

    def send_request_pen(self, r,g,b,w,off):
        # int = red, green, blue, width, on/off (0/1)
        #Services
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pen service, waiting again...')
        # self.req = SetPen.Request()

        self.req = SetPen.Request()
        self.req.r = r
        self.req.g = g
        self.req.b = b
        self.req.width = w
        self.req.off = off

        self.future = self.cli.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_request_spawn(self, x, y, t, name):
        # float32 x, y , theta
        # string name
        self.req2 = Spawn.Request()
        self.req2.x = float(x)
        self.req2.y = float(y)
        self.req2.theta = float(t)
        self.req2.name = str(name)

        self.future2 = self.spa.call_async(self.req2)
        rclpy.spin_until_future_complete(self, self.future2)
        return self.future2.result()

    def send_request_kill(self, name):
        self.req3 = Kill.Request()
        self.req3.name = str(name)

        self.future3 = self.kill.call_async(self.req3)
        if self.enemyDict.get(self.currentEnemy) != None:
            del self.enemyDict[self.currentEnemy]

        self.currentEnemyIdx = 0
        return self.future3.result()

    def get_enemy_distance(self):
        a = self.current_pose != None
        b = self.enemyDict[self.currentEnemy] != None
        self.enemy_pose = self.enemyDict[self.currentEnemy]
        k2 = 3
        k1 = 1
        if a and b and self.readyToKill == False and self.resetting == False:
            dist = self.euclidean_distance(self.current_pose, self.enemy_pose)
            if dist < k2:
                if not self.state.__eq__("Angry"):
                    self.get_logger().info('Angry, following')
                    self.state = "Angry"
                if dist < k1 and self.state == "Angry":
                    self.get_logger().info('Kill')
                    self.readyToKill = True

            else:
                if not self.state.__eq__("Drawing"):
                    self.get_logger().info('Chilled, back to drawing')
                    self.state = "Drawing"

    def get_enemy_distance_test(self):
        k2 = 3
        dists = []
        poses = []
        ens = []
        if self.resetting == False and len(self.enemyDict) != 0 and self.state != "Angry":
            for el in self.enemyDict:
                enPose = self.enemyDict.get(el)
                if self.current_pose != None and enPose != None:
                    dists.append(self.euclidean_distance(self.current_pose, enPose))
                    poses.append(enPose)
                    ens.append(el)

            dist = min(dists)
            if dist < k2:
                idx = dists.index(dist)
                self.enemy_pose = poses[idx]
                self.currentEnemy = ens[idx]
                self.state = "Angry"

        if self.state == "Angry":
            self.get_enemy_distance()
        #         if not self.state.__eq__("Angry"):
        #             self.get_logger().info('Angry, following')
        #             self.state = "Angry"
        #             print(self.currentEnemy)
        #             #self.goal_pose = self.enemy_pose ###########TEMP
        #         if dist < k1 and self.state == "Angry":
        #             self.get_logger().info('Kill')
        #             self.readyToKill = True
        #     else:
        #         if not self.state.__eq__("Drawing"):
        #             self.get_logger().info('Chilled, back to drawing')
        #             self.state = "Drawing"
        # else:
        #     if self.current_pose != None and self.enemy_pose != None:
        #         dist = self.euclidean_distance(self.current_pose, self.enemy_pose)
        #         if dist < k2:
        #             if not self.state.__eq__("Angry"):
        #                 self.get_logger().info('Angry, following')
        #                 self.state = "Angry"
        #                 #self.goal_pose = self.enemy_pose ###########TEMP
        #             if dist < k1 and self.state == "Angry":
        #                 self.get_logger().info('Kill')
        #                 self.readyToKill = True


    def get_intercepted_pose(self, pose_, m):
        pose = Pose()
        pose.x = pose_.x
        pose.y = pose_.y
        pose.theta = pose_.theta
        pose.angular_velocity = pose_.angular_velocity
        pose.linear_velocity = pose_.linear_velocity

        
        dist = self.euclidean_distance(self.current_pose, pose)
        if dist < m:
            m = dist
        pose.x += cos(pose.theta) * m
        pose.y += sin(pose.theta) * m
        return pose

    def get_enemy_list(self):
        pubs = self.get_node_names()        
        enemies = []
        turtleNode = None
        for elem in pubs:
            if elem == 'turtlesim':
                tuts = self.get_subscriber_names_and_types_by_node('turtlesim', '/')
                tuts =  [x[0].split('/')[1] for x in tuts]
                for el in tuts:
                    if "turtle" in el and not el == self.tname:
                        enemies.append(el)
        
        self.enemies = enemies
        
        # dictenemy = {}
        # for en in enemies:
        #     dictenemy.update({en:Pose()})
        # self.enemyCount = len(enemies)
        # self.enemyDict = dictenemy

        return enemies

    def test_enemy_poses_callback(self, msg):

        self.tempPose = msg
        self.tempPose.x = round(self.tempPose.x,4)
        self.tempPose.y = round(self.tempPose.y,4)

        self.enemyDict.update({self.currentEnemy: self.tempPose})

        if self.state != "Angry":
            if len(self.enemies) == 0:
                self.currentEnemy = 'turtle1'
                return 

            if self.currentEnemyIdx +1 >= len(self.enemies):
                self.currentEnemyIdx = -1
   
            self.currentEnemyIdx += 1
            self.currentEnemy = self.enemies[self.currentEnemyIdx]

        else:
            self.enemy_pose = msg
            self.enemy_pose.x = round(self.enemy_pose.x, 4)
            self.enemy_pose.y = round(self.enemy_pose.y, 4)

        self.destroy_subscription(self.enemy_pose_subscriber)
        self.enemy_pose_subscriber = self.create_subscription(Pose, '/'+ str(self.currentEnemy) +'/pose', self.test_enemy_poses_callback, 10)
         
    def print_dict(self):
        print('Dict',self.enemyDict)
        print("enemies", self.enemies)
        
 
def get_usi_list():
    uList = [
    [1,10],
    [1,9], #soft
    [1,2],
    [3,2],
    [3,3], #soft
    [3,10]]

    sList = [
    [7,10],
    [4,9], #soft
    [4,6],
    [7,6],
    [7,3], #soft
    [4,2]]

    iList = [
    [9,2],
    [9,3],
    [9,10]]

    letters = []
    letters.append(uList)
    letters.append(sList)
    letters.append(iList)

    return letters


def test():
    rclpy.init(args=sys.argv)
    goal_pose = Pose()
    goal_pose.x = 2.0 #initalx
    goal_pose.y = 10.0 #initialy

    # Request turtle1 to spawn turtle2, then destroy the its node and order turtle2 to draw
    node = TurtleManagerNode(goal_pose, tolerance=0.01, name='turtle1')
    node.send_request_spawn(1, 1, 1, 'turtle2')
    node.destroy_node()

    # Start the behaviour of turtle2
    node = TurtleManagerNode(goal_pose, tolerance=0.01, name='turtle2')
    done = node.start_moving()
    rclpy.spin_until_future_complete(node, done)

    #TODO
    # get turtle to follow till x DONE
    # once x is reached, return to drawing DONE
    # fix refresh rate of turtle DONE
    # get ahead of teleop turtle DONE
    # kill teleop turtle DONE
 
def main():
    test()


if __name__ == '__main__':
    main()


