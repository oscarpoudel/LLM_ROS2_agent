#!/usr/bin/env python3
import math
import threading
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class RobotNavigator:
    """
    Nav2‐based Robot Navigator using TF2 lookup of map→base_link only.
    """

    def __init__(self,
                 node_name: str = "robot_navigator_tf2",
                 action_name: str = "navigate_to_pose",
                 map_frame: str = "map",
                 base_frame: str = "base_link"):
        # rclpy.init()
        self._node = Node(node_name)
        self._map_frame  = map_frame
        self._base_frame = base_frame

        # TF2 buffer & listener
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)

        # Nav2 action client
        self._nav_client = ActionClient(self._node, NavigateToPose, action_name)

        # background spinner
        executor = MultiThreadedExecutor()
        executor.add_node(self._node)
        threading.Thread(target=executor.spin, daemon=True).start()

    async def _get_map_base_pose(self, timeout: float = 2.0) -> PoseStamped:
        """
        Lookup map→base_link via TF2, waiting up to `timeout` seconds.
        Returns a fully‐populated PoseStamped in the map frame.
        """
        end = self._node.get_clock().now() + rclpy.duration.Duration(seconds=timeout)
        while rclpy.ok() and self._node.get_clock().now() < end:
            try:
                tf: TransformStamped = self._tf_buffer.lookup_transform(
                    self._map_frame,
                    self._base_frame,
                    rclpy.time.Time()
                )
                ps = PoseStamped()
                ps.header = tf.header
                ps.pose.position = Point(
                    x=tf.transform.translation.x,
                    y=tf.transform.translation.y,
                    z=tf.transform.translation.z
                )
                ps.pose.orientation = tf.transform.rotation
                return ps
            except (LookupException, ExtrapolationException, ConnectivityException):
                await asyncio.sleep(0.05)

        raise TimeoutError(f"TF2 lookup {self._map_frame}→{self._base_frame} timed out")

    async def _send_goal(self, x: float, y: float, yaw: float) -> str:
        """
        Build & send a single NavigateToPose goal in the map frame.
        """
        

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        # only yaw around Z
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # wait for server
        await asyncio.get_event_loop().run_in_executor(
            None, self._nav_client.wait_for_server, 5.0
        )

        send_fut = self._nav_client.send_goal_async(goal)
        handle = await send_fut
        if not handle.accepted:
            return "Nav2 goal rejected"

        result_fut = handle.get_result_async()
        result    = await result_fut
        status    = result.status

        return "succeeded" if status == GoalStatus.STATUS_SUCCEEDED else f"failed (status={status})"

    async def navigate(self, distance: float = 0.0, angle_deg: float = 0.0) -> str:
        """
        Move `distance` meters forward (along current heading) and then end up
        facing `angle_deg` degrees offset (CCW positive) in a single Nav2 goal.
        """
        try:
            ps = await self._get_map_base_pose()
        except TimeoutError as e:
            return str(e)

        # extract current pose
        x0, y0 = ps.pose.position.x, ps.pose.position.y
        q       = ps.pose.orientation
        yaw0    = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )
        print("current poes",ps.pose)
        # compute new pose
        yaw1 = yaw0 + math.radians(angle_deg)
        x1   = x0 + distance * math.cos(yaw0)
        y1   = y0 + distance * math.sin(yaw0)
        # return
        print("moving robot to positon",x1, y1, yaw1)
        return await self._send_goal(x1, y1, yaw1)

    async def move(self, distance: float) -> str:
        """Shortcut: just translate forward/backward."""
        return await self.navigate(distance=distance, angle_deg=0.0)

    async def turn(self, angle_deg: float) -> str:
        """Shortcut: just rotate in place."""
        return await self.navigate(distance=0.0, angle_deg=angle_deg)


# quick test
async def main():
    nav = RobotNavigator()
    print(await nav.navigate(1.0, 30.0))
    print(await nav.move(1.0))
    print(await nav.turn(45.0))

if __name__ == "__main__":
    asyncio.run(main())
