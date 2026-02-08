from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='offboard_rl',          # Nome del pacchetto
            executable='px4_monitor_node',       # Nome dell'eseguibile (definito in CMakeLists.txt)
            name='px4_monitor_node',        # Nome del nodo (sovrascrive quello nel codice C++)
            output='screen',                # Stampa i log nel terminale
            emulate_tty=True                # Migliora la formattazione dei colori nei log
        ),
        Node(
            package='offboard_rl',          # Nome del pacchetto
            executable='trajectory_drone_manager',       # Nome dell'eseguibile (definito in CMakeLists.txt)
            name='trajectory_drone_manager',        # Nome del nodo (sovrascrive quello nel codice C++)
            output='screen',                # Stampa i log nel terminale
            emulate_tty=True                # Migliora la formattazione dei colori nei log
        ),
        Node(
            package='offboard_rl',          # Nome del pacchetto
            executable='go_to_point_server',       # Nome dell'eseguibile (definito in CMakeLists.txt)
            name='go_to_point_server',        # Nome del nodo (sovrascrive quello nel codice C++)
            output='screen',                # Stampa i log nel terminale
            emulate_tty=True                # Migliora la formattazione dei colori nei log
        )
    ])