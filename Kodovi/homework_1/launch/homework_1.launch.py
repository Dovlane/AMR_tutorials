from launch import LaunchDescription

from launch_ros.actions import Node 

 
def generate_launch_description() : 
    return LaunchDescription([ 
        Node( 
            package='homework_1', 
               executable='senzor_temperatura', 
               name='senzor_temperatura', 
               output='screen', 
        ), 
        Node( 
               package='homework_1', 
               executable='senzor_vlaznost', 
               name='senzor_vlaznost', 
               output='screen', 
        ), 
        Node( 
               package='homework_1', 
               executable='senzor_osvetljenje', 
               name='senzor_osvetljenje', 
               output='screen', 
        ), 
        Node( 
               package='homework_1', 
               executable='vizuelizacija_sistema', 
               name='vizuelizacija_sistema', 
               output='screen', 
        ),
        Node(
               package='homework_1',
               executable='brava',
               name='brava',
               output='screen',
        ),
        Node(
               package='homework_1',
               executable='alarm',
               name='alarm',
               output='screen',
        ),
    ])
