import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import math

class StaticTF(Node):
    def __init__(self):
        super().__init__('static_tf')
        self.br = StaticTransformBroadcaster(self)

        # base_footprint -> base_link (static, zero offset)
        base_tf = TransformStamped()
        base_tf.header.stamp = self.get_clock().now().to_msg()
        base_tf.header.frame_id = 'base_footprint'
        base_tf.child_frame_id = 'base_link'
        base_tf.transform.translation.x = 0.0
        base_tf.transform.translation.y = 0.0
        base_tf.transform.translation.z = 0.0
        base_tf.transform.rotation.z = 0.0
        base_tf.transform.rotation.w = 1.0

        # base_link -> laser
        laser_tf = TransformStamped()
        laser_tf.header.stamp = self.get_clock().now().to_msg()
        laser_tf.header.frame_id = 'base_link'
        laser_tf.child_frame_id = 'laser'
        laser_tf.transform.translation.x = 0.18
        laser_tf.transform.translation.y = 0.0
        laser_tf.transform.translation.z = 0.14

        yaw = math.pi
        laser_tf.transform.rotation.z = math.sin(yaw * 0.5)
        laser_tf.transform.rotation.w = math.cos(yaw * 0.5)

        # laser -> imu static transform
        imu_tf = TransformStamped()
        imu_tf.header.stamp = self.get_clock().now().to_msg()
        imu_tf.header.frame_id = 'base_link'
        imu_tf.child_frame_id = 'imu_link'
        imu_tf.transform.translation.x = 0.0
        imu_tf.transform.translation.y = -0.05
        imu_tf.transform.translation.z = 0.43

        yaw2 = 0.0
        imu_tf.transform.rotation.z = math.sin(yaw2 * 0.5)
        imu_tf.transform.rotation.w = math.cos(yaw2 * 0.5)

        # base_link -> camera_link (mounting offset; adjust as needed)
        camera_tf = TransformStamped()
        camera_tf.header.stamp = self.get_clock().now().to_msg()
        camera_tf.header.frame_id = 'base_link'
        camera_tf.child_frame_id = 'camera_link'
        camera_tf.transform.translation.x = 0.25
        camera_tf.transform.translation.y = 0.0
        camera_tf.transform.translation.z = 0.34

        camera_roll = 0.0
        camera_pitch = 0.0
        camera_yaw = 0.0
        half_roll = camera_roll * 0.5
        half_pitch = camera_pitch * 0.5
        half_yaw = camera_yaw * 0.5
        cy = math.cos(half_yaw)
        sy = math.sin(half_yaw)
        cp = math.cos(half_pitch)
        sp = math.sin(half_pitch)
        cr = math.cos(half_roll)
        sr = math.sin(half_roll)

        camera_tf.transform.rotation.x = sr * cp * cy - cr * sp * sy
        camera_tf.transform.rotation.y = cr * sp * cy + sr * cp * sy
        camera_tf.transform.rotation.z = cr * cp * sy - sr * sp * cy
        camera_tf.transform.rotation.w = cr * cp * cy + sr * sp * sy

        # send both transforms
        self.br.sendTransform([base_tf, laser_tf, imu_tf, camera_tf])

def main():
    rclpy.init()
    node = StaticTF()
    rclpy.spin(node)  # 也可以 sleep 一下后退出
    rclpy.shutdown()

if __name__ == '__main__':
    main()
