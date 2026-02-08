#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String  # Aggiunto String

class GripMonitorNode(Node):
    def __init__(self):
        super().__init__('grip_monitor_node')
        
        # Stato interno
        self.is_attached = False
        
        # Subscriber al topic del BRIDGE (ora è una Stringa)
        self.subscription = self.create_subscription(
            String,
            '/panda/state',
            self.state_callback,
            10)
            
        # Publisher periodico (Manteniamo Bool per comodità degli altri nodi)
        self.publisher_ = self.create_publisher(Bool, '/panda/current_grip_status', 10)
        
        self.timer = self.create_timer(0.5, self.timer_callback)

    def state_callback(self, msg):
        # Analizziamo la stringa che arriva da Gazebo
        if msg.data == "attached":
            self.is_attached = True
            self.get_logger().info('Stato rilevato: PRESA (Attached)')
        elif msg.data == "detached":
            self.is_attached = False
            self.get_logger().info('Stato rilevato: RILASCIO (Detached)')
        else:
            self.get_logger().warn(f'Stato sconosciuto ricevuto: {msg.data}')

    def timer_callback(self):
        msg = Bool()
        msg.data = self.is_attached
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GripMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()