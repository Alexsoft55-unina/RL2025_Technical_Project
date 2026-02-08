#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class PX4ToTFBridge(Node):
    def __init__(self):
        super().__init__('px4_to_tf_bridge')

        # ==========================================
        #  CONFIGURAZIONE OFFSET (Modifica qui!)
        # ==========================================
        
        # 1. POSIZIONE: Sposta il drone rispetto all'origine (0,0) del Panda
        self.offset_x = 0.0   # 1 metro a "Est" rispetto al Panda
        self.offset_y = 0.0   # Allineato sull'asse Y
        self.offset_z = 0.24  # Alza i piedini al livello del suolo
        
        # 2. ROTAZIONE: Ruota il drone su se stesso (in GRADI)
        # 0   = Orientamento originale (Nord)
        # 90  = Ruotato a Sinistra (Ovest)
        # 180 = Girato di spalle (Sud)
        # -90 = Ruotato a Destra (Est)
        self.yaw_offset_degrees = 90.0  # <--- CAMBIA QUESTO VALORE PER RUOTARLO

        # ==========================================

        # QoS Profile per PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(f"Bridge avviato. Offset Yaw: {self.yaw_offset_degrees} gradi.")

    def odometry_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'      
        t.child_frame_id = 'base_link'  

        # --- 1. POSIZIONE (NED -> ENU + Offset) ---
        px4_north = float(msg.position[0])
        px4_east  = float(msg.position[1])
        px4_down  = float(msg.position[2])

        # Applico rotazione assi e offset
        t.transform.translation.x = self.offset_x + px4_east
        t.transform.translation.y = self.offset_y + px4_north
        t.transform.translation.z = self.offset_z - px4_down

        # --- 2. ORIENTAMENTO (NED -> ENU + Rotazione Manuale) ---
        
        # A. Recupero Quaternione PX4
        q_w = float(msg.q[0])
        q_x = float(msg.q[1])
        q_y = float(msg.q[2])
        q_z = float(msg.q[3])

        # B. Conversione NED -> ENU di base (Visualizzazione Standard)
        # Mappatura empirica per allineare gli assi visuali
        # ROS (x, y, z, w)
        base_x = q_y
        base_y = q_x
        base_z = -q_z
        base_w = q_w

        # C. Calcolo Quaternione di Offset (Rotazione attorno a Z)
        # Formula: q_rot = [0, 0, sin(theta/2), cos(theta/2)]
        rads = math.radians(self.yaw_offset_degrees)
        off_z = math.sin(rads / 2.0)
        off_w = math.cos(rads / 2.0)
        
        # D. Moltiplicazione Quaternioni (Offset * Base)
        # q_final = q_offset * q_base
        # (Matematica standard dei quaternioni)
        final_x = off_w * base_x + off_z * base_y
        final_y = off_w * base_y - off_z * base_x
        final_z = off_w * base_z + off_z * base_w
        final_w = off_w * base_w - off_z * base_z

        # E. Assegnazione
        t.transform.rotation.x = final_x
        t.transform.rotation.y = final_y
        t.transform.rotation.z = final_z
        t.transform.rotation.w = final_w

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PX4ToTFBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()