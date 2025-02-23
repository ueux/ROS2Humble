import os
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    #this name has to match the robot name int the xacro file
    robotXacroNmae="differential_drive_robot"
    #this is the name of our package at the same time this is the name of the folder that will be used to define the paths
    namePackage="mobile_dd_robot"
    #this is the relative path to the xacro file
    modelFileRelativePath='model/robot.xacro'
    #this is the relative path to the Gazebo world file
    worldFileRelativePath='model/empty_world.world'
    # this is the absolute path to the model
    pathModelFile=os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)
    # this is the absolute path to the world
    pathWorldFile=os.path.join(get_package_share_directory(namePackage),worldFileRelativePath)
    #this is the robot description from the xacro model file
    robotDescription=xacro.process_file(pathModelFile).toxml()
    #this is the launch file from the gazebo_ros package
    gazebo_rosPackage=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'))
    #this is the launch description
    gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackage,launch_arguments={'world':pathWorldFile}.items())
    #here we crete a gazebo_ros node
    spawnModeNode=Node(package='gazebo_ros',executable='spawn_entity.py',arguments=['-topic','robot_description','-entity',robotXacroNmae],output='screen')
    #Robot state publisher node
    robotStatePublisherNode=Node(package='robot_state_publisher',executable='robot_state_publisher',output='screen',parameters=[{"robot_description":robotDescription, "use_sim_time":True}])
    #here we create an empty launch description object
    ld=LaunchDescription()
    #we add the gazebo launch description to the launch description object
    ld.add_action(gazeboLaunch)
    #we add the spawn model node to the launch description object
    ld.add_action(spawnModeNode)
    #we add the robot state publisher node to the launch description object
    ld.add_action(robotStatePublisherNode)
    return ld
#this is the launch file that will be used to spawn the robot in the Gazebo simulation


