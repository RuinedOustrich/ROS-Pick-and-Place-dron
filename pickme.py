#!/usr/bin/env python3
import sys
import math

## Ros ##
import rospy

## MoveIt and TF ##
import moveit_commander
from tf.transformations import quaternion_from_euler

## Import msgs and srvs ##
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory, Grasp, PlaceLocation
from moveit_msgs.srv import ApplyPlanningScene


def openGripper(posture):
    
    ## - Инициализируем Хват - ##
    ## Добавляем джоинты для хвата ##
    posture.joint_names = [str for i in range(2)]
    posture.joint_names[0] = "panda_finger_joint1"
    posture.joint_names[1] = "panda_finger_joint2"

    ## Устанавливаем их окрытыми, чтобы между поместился объект ##
    posture.points = [JointTrajectoryPoint()]
    posture.points[0].positions = [float for i in range(2)]
    posture.points[0].positions[0] = 0.05
    posture.points[0].positions[1] = 0.05
    posture.points[0].time_from_start = rospy.Duration(0.5)
    ## ##

def closedGripper(posture):
    
    ## - Инициализируем Хват - ##
    ## Добавляем джоинты для хвата ##
    posture.joint_names = [str for i in range(2)]
    posture.joint_names[0] = "panda_finger_joint1"
    posture.joint_names[1] = "panda_finger_joint2"

    ## Подаем сигнал для закрытия ##
    posture.points = [JointTrajectoryPoint()]
    posture.points[0].positions = [float for i in range(2)]
    posture.points[0].positions[0] = 0.03
    posture.points[0].positions[1] = 0.03
    posture.points[0].time_from_start = rospy.Duration(0.5)
    ## ##


def pick(move_group):

    ## Захват обьекта - ##
    ## Устанавливаем направление движения руки ##
    grasps = [Grasp() for i in range(1)]

    ## Настраиваем позицию расположения хвата ( позицию джоинта pands_link8)  ##

    grasps[0].grasp_pose.header.frame_id = "panda_link0"
    orientation = quaternion_from_euler(0, -math.pi, 3*math.pi / 4 )
    grasps[0].grasp_pose.pose.orientation.x = orientation[0]
    grasps[0].grasp_pose.pose.orientation.y = orientation[1]
    grasps[0].grasp_pose.pose.orientation.z = orientation[2]
    grasps[0].grasp_pose.pose.orientation.w = orientation[3]
    grasps[0].grasp_pose.pose.position.x = 0.715
    grasps[0].grasp_pose.pose.position.y = 0
    grasps[0].grasp_pose.pose.position.z = 0.24

    ## Устанавливаем положение хвата  ##
    # Указываем точку начала отсчета координат
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link8"
    # Устанавливаем положение хвата по оси
    grasps[0].pre_grasp_approach.direction.vector.z = 1.0
    grasps[0].pre_grasp_approach.min_distance = 0.095
    grasps[0].pre_grasp_approach.desired_distance = 0.115

    ## Устанавливаем положение хвата  ##
    # Указываем точку начала отсчета координат
    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0"
    # Устанавливаем положение хвата по оси
    grasps[0].post_grasp_retreat.direction.vector.z = 0.5
    grasps[0].post_grasp_retreat.min_distance = 0.1
    grasps[0].post_grasp_retreat.desired_distance = 0.25

    ## Настраиваем хват перед подбором обьекта ##
    openGripper(grasps[0].pre_grasp_posture)
    ## ##

    ## ##
    ## позиция хвата во время захвата обьекта ##
    closedGripper(grasps[0].grasp_posture)

    ##  ##
    move_group.set_support_surface_name("table1")

    # пиднимаем обьект со стола
    move_group.pick("object", grasps)
    ## конец функции ##


def place(group):
    
    ##  ##
    # Создаем вектор направления для опускания обьекта.
    place_location = [PlaceLocation() for i in range(1)]

    ## Устанавливае позицию для  ##
    place_location[0].place_pose.header.frame_id = "panda_link0"
    orientation = quaternion_from_euler(0, -2*math.pi, -math.pi / 2 )
    place_location[0].place_pose.pose.orientation.x = orientation[1]
    place_location[0].place_pose.pose.orientation.y = orientation[0]
    place_location[0].place_pose.pose.orientation.z = orientation[2]
    place_location[0].place_pose.pose.orientation.w = orientation[3]

    place_location[0].place_pose.pose.position.x = 0
    place_location[0].place_pose.pose.position.y = -0.6
    place_location[0].place_pose.pose.position.z = 0.25

    place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0"
    place_location[0].pre_place_approach.direction.vector.z = (-1.0) 
    place_location[0].pre_place_approach.min_distance = 0.095
    place_location[0].pre_place_approach.desired_distance = 0.115
    
    place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0"
    place_location[0].post_place_retreat.direction.vector.z = (0.1) 
    place_location[0].post_place_retreat.min_distance = 0.1
    place_location[0].post_place_retreat.desired_distance = 0.25
    place_location[0].post_place_retreat.direction.vector.y = (0.5) 
    place_location[0].post_place_retreat.min_distance = 0.1
    place_location[0].post_place_retreat.desired_distance = 0.25

    openGripper(place_location[0].post_place_posture)

    #group.set_support_surface_name("table2")

    group.place("object", place_location[0])
  


def addCollisionObjects(planning_scene_interface):

    ## Инициализируем обьекты в модели ##
    collision_objects_names = [str for i in range(13)]
    collision_object_sizes = [str for i in range(13)]
    collision_objects = [PoseStamped() for i in range(13)]

    ## добавить стол 1 ##
    collision_objects_names[0] = "table1"
    collision_objects[0].header.frame_id = "panda_link0"

    collision_object_sizes[0] = (0.3, 0.3, 0.05)  # Размер стола 1

    ## Расположение стола ##
    collision_objects[0].pose.position.x = 0.715
    collision_objects[0].pose.position.y = 0
    collision_objects[0].pose.position.z = 0.025
    ## ##

    ## - Создаем стол 2 -##
    collision_objects_names[2] = "table2"
    collision_objects[2].header.frame_id = "panda_link0"

    collision_object_sizes[2] = (0.4, 0.4, 0.2)  # Размер стола 2

    ## Расположение стола 2 ##
    collision_objects[2].pose.position.x = 0
    collision_objects[2].pose.position.y = -0.6
    collision_objects[2].pose.position.z = 0.1

    ## Инициализируем обьект перемещения ##
    collision_objects_names[1] = "object"
    collision_objects[1].header.frame_id = "panda_link0"

    collision_object_sizes[1] = (0.07, 0.07, 0.1)  # размеры обьекта

    ## Разположение обьекта перемещения ##
    collision_objects[1].pose.position.x = 0.7
    collision_objects[1].pose.position.y = 0
    collision_objects[1].pose.position.z = 0.1
    ####
    collision_objects_names[3] = "object1"
    collision_objects[3].header.frame_id = "panda_link0"

    collision_object_sizes[3] = (0.025, 0.025, 0.2)  # размеры обьекта

    ##  ##
    collision_objects[3].pose.position.x = 0.8525
    collision_objects[3].pose.position.y = 0.1375
    collision_objects[3].pose.position.z = 0.1
    
    collision_objects_names[4] = "object2"
    collision_objects[4].header.frame_id = "panda_link0"

    collision_object_sizes[4] = (0.025, 0.025, 0.2)  # размеры обьекта

    ## ##
    collision_objects[4].pose.position.x = 0.8525
    collision_objects[4].pose.position.y = - 0.1375
    collision_objects[4].pose.position.z = 0.1
    
    collision_objects_names[5] = "object3"
    collision_objects[5].header.frame_id = "panda_link0"

    collision_object_sizes[5] = (0.025, 0.025, 0.2)  # размеры обьекта

    ##  ##
    collision_objects[5].pose.position.x = 0.5765
    collision_objects[5].pose.position.y = 0.1375
    collision_objects[5].pose.position.z = 0.1
    
    
    collision_objects_names[6] = "object4"
    collision_objects[6].header.frame_id = "panda_link0"

    collision_object_sizes[6] = (0.025, 0.025, 0.2)  # размеры обьекта

    ##  ##
    collision_objects[6].pose.position.x = 0.5765
    collision_objects[6].pose.position.y = - 0.1375
    collision_objects[6].pose.position.z = 0.1
    
    collision_objects_names[7] = "object5"
    collision_objects[7].header.frame_id = "panda_link0"

    collision_object_sizes[7] = (0.025, 0.3, 0.025)  # размеры обьекта

    ##  ##
    collision_objects[7].pose.position.x = 0.5765
    collision_objects[7].pose.position.y = 0
    collision_objects[7].pose.position.z = 0.1875
    
    collision_objects_names[8] = "object6"
    collision_objects[8].header.frame_id = "panda_link0"

    collision_object_sizes[8] = (0.3, 0.025, 0.025)  # размеры обьекта

    ##  ##
    collision_objects[8].pose.position.x = 0.715
    collision_objects[8].pose.position.y = -0.1375
    collision_objects[8].pose.position.z = 0.1875
    
    collision_objects_names[9] = "object7"
    collision_objects[9].header.frame_id = "panda_link0"

    collision_object_sizes[9] = (0.3, 0.025, 0.025)  # размеры обьекта

    ##  ##
    collision_objects[9].pose.position.x = 0.715
    collision_objects[9].pose.position.y = 0.1375
    collision_objects[9].pose.position.z = 0.1875
    
    collision_objects_names[10] = "object8"
    collision_objects[10].header.frame_id = "panda_link0"

    collision_object_sizes[10] = (0.025, 0.3, 0.025)  # размеры обьекта

    ##  ##
    collision_objects[10].pose.position.x = 0.8525
    collision_objects[10].pose.position.y = 0
    collision_objects[10].pose.position.z = 0.1875
    
    collision_objects_names[11] = "object9"
    collision_objects[11].header.frame_id = "panda_link0"

    collision_object_sizes[11] = (0.3, 0.15, 0.01)  # размеры обьекта

    ##  ##
    collision_objects[11].pose.position.x = 0.715
    collision_objects[11].pose.position.y = 0.2
    collision_objects[11].pose.position.z = 0.205
    
    collision_objects_names[12] = "object10"
    collision_objects[12].header.frame_id = "panda_link0"

    collision_object_sizes[12] = (0.3, 0.15, 0.01)  # размеры обьекта

    ##  ##
    collision_objects[12].pose.position.x = 0.715
    collision_objects[12].pose.position.y = -0.2
    collision_objects[12].pose.position.z = 0.205
    
    ## Добавляем обьекты в сцену ##
    for (name, pose, size) in zip(
            collision_objects_names, collision_objects, collision_object_sizes
    ):
        planning_scene_interface.add_box(name=name, pose=pose, size=size)
        


if __name__ == "__main__":
	
	## Инициализируем ноду ##
	rospy.init_node("panda_arm_pick_place")

  	## Инициализируем moveit_commander ##
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.loginfo(
        "Conneting moveit default moveit 'apply_planning_scene' service.")
	rospy.wait_for_service("apply_planning_scene")
	try:
		planning_scene_srv = rospy.ServiceProxy("apply_planning_scene", ApplyPlanningScene)
		rospy.loginfo("Moveit 'apply_planning_scene' service found!")
	except rospy.ServiceException as e:
		rospy.logerr("Moveit 'apply_planning_scene' service initialization failed: %s" % e)
		shutdown_msg = "Shutting down %s node because %s service connection failed." % (rospy.get_name(),planning_scene_srv.resolved_name,)
		rospy.logerr(shutdown_msg)
		sys.exit(0)

    ## Обьявляем RobotCommander ##
	robot = moveit_commander.RobotCommander(robot_description="robot_description", ns="/")
	rospy.logdebug("Robot Groups: %s", robot.get_group_names())

    ## создаем сцену и инициализируем управляемую группу ##
	move_group = robot.get_group("panda_arm")
	planning_scene_interface = moveit_commander.PlanningSceneInterface(ns="/")

    ## Плагин ##
	move_group.set_planner_id("TRRTkConfigDefault")
    ####
	rospy.sleep(5.0)
    ## Создаем обьекты ##
	addCollisionObjects(planning_scene_interface)
    ####
	rospy.sleep(5.0)
	pick(move_group)
	rospy.sleep(5.0)
    ## Опускаем ##
	place(move_group)  	
    	
