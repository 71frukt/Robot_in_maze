import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Клиент к экшн-серверу follow_path (из Nav2 controller_server)
        self.client = ActionClient(self, FollowPath, 'follow_path')

        # Публикация пути для RViz (топик /followed_path)
        self.path_pub = self.create_publisher(Path, 'followed_path', 10)

        # Таймер запуска отправки пути
        self.timer = self.create_timer(3.0, self.send_path)
        self.sent = False

    def send_path(self):
        if self.sent:
            return

        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("FollowPath server not available yet.")
            return

        path = Path()
        path.header.frame_id = "map"

        for i in range(5):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = 1.0 + i
            pose.pose.position.y = 1.0
            pose.pose.orientation.w = 1.0  # Ориентация по умолчанию

            path.poses.append(pose)
            self.get_logger().info(f"Добавлена точка {i}: x={pose.pose.position.x}, y={pose.pose.position.y}")

        # Публикуем путь в топик для визуализации
        self.path_pub.publish(path)

        # Отправляем путь на выполнение в Nav2
        goal_msg = FollowPath.Goal()
        goal_msg.path = path

        self.client.send_goal_async(goal_msg)
        self.sent = True
        self.get_logger().info('Путь отправлен в follow_path!')


def main():
    rclpy.init()
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()