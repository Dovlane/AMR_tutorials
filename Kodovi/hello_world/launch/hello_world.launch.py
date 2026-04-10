from launch import LaunchDescription 

from launch_ros.actions import Node 

 
def generate_launch_description() : 
    return LaunchDescription([ 
        Node( 
            package='hello_world', 
               executable='talker', 
               name='talker', 
               output='screen', 
        ), 
        Node( 
               package='hello_world', 
               executable='listener', 
               name='lisntene', 
               output='screen', 
        ), 
    ]) 
