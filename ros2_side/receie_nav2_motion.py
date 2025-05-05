#!/usr/bin/env python3
import asyncio
import json
import base64
import subprocess

import psutil
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nats.aio.client import Client as NATS

from robot_navigator_tf2 import RobotNavigator

# 1) Initialize rclpy exactly once here:
rclpy.init()

# 2) Create your Navigator (no init inside):
navigator = RobotNavigator()
# Map of named locations to coordinates and yaw in radians


PREDEFINED_LOCATIONS = {
    "boat": (0.5, 3.0, 1.57),
    "firehydrant": (0.0, 0.0, 3.14),
    "bag": (5.5, 3.0, 1.57),
}

async def handle_nav(data: str) -> str:
    try:
        params = json.loads(data)
        location = params.get("location", "").strip().lower()
        if location not in PREDEFINED_LOCATIONS:
            return f"Unknown location '{location}'. Known: {list(PREDEFINED_LOCATIONS.keys())}"
        
        x, y, yaw = PREDEFINED_LOCATIONS[location]
        print(f"[NAV] Navigating to {location} @ ({x:.2f}, {y:.2f}, {yaw:.2f})")
        return await navigator._send_goal(x, y, yaw)
    except Exception as e:
        return f"Nav error: {e}"


def init_ros2():
    node = Node('ros_bridge_node')
    bridge = CvBridge()
    last_frame = {'image': None}

    def image_callback(msg: Image):
        last_frame['image'] = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    node.create_subscription(Image, '/camera/image_raw', image_callback, 10)
    return node, last_frame

async def handle_motion(data: str) -> str:
    params = json.loads(data)
    action = params.get('action')
    d = params.get('distance') or 0.0
    a = params.get('angle')    or 0.0
    # delegate to Navigator
    return await navigator.navigate(distance=d, angle_deg=a)

def handle_status(data: str) -> str:
    queries = json.loads(data).get('queries', [])
    metrics = {}
    metrics['battery'] = "70%"
    metrics['cpu'] = "30 percent usage"
    metrics["motor status"] ="all motors working well"
    metrics['health'] = 'OK'
    return json.dumps(metrics)

async def handle_vision(data: str, context) -> str:
    ros_node, last_frame = context
    rclpy.spin_once(ros_node, timeout_sec=0.5)
    frame = last_frame.get('image')
    if frame is None:
        return json.dumps({'image': ''})
    import cv2
    success, jpeg = cv2.imencode('.jpg', frame)
    b64 = base64.b64encode(jpeg.tobytes()).decode('utf-8') if success else ''
    return json.dumps({'image': b64})

def handle_ros_cli(data: str) -> str:
    try:
        return subprocess.check_output(data.split(), stderr=subprocess.STDOUT).decode()
    except subprocess.CalledProcessError as e:
        return f'Error: {e.output.decode()}'

async def ros_bridge():
    nc = NATS()
    await nc.connect("nats://localhost:4222")

    ros_node, last_frame = init_ros2()
    context = (ros_node, last_frame)

    async def handler(msg):
        subject = msg.subject
        payload = msg.data.decode()
        print(f"[ROSBridge] Received on {subject}: {payload}")

        if subject == 'motion.cmd':
            reply = await handle_motion(payload)
        elif subject == 'status.cmd':
            reply = handle_status(payload)
        elif subject == 'vision.cmd':
            reply = await handle_vision(payload, context)
        elif subject == 'nav.cmd':
            reply = await handle_nav(payload)
        elif subject == 'ros_cmd':
            reply = handle_ros_cli(payload)
        else:
            reply = ''

        if msg.reply:
            print(f"[ROSBridge] Replying: {reply}")
            await nc.publish(msg.reply, reply.encode('utf-8'))

    for subj in ['motion.cmd','status.cmd','vision.cmd','ros_cmd','nav.cmd']:
        await nc.subscribe(subj, cb=handler)

    print("[ROSBridge] Listening on motion.cmd, status.cmd, vision.cmd, ros_cmd,nav.cmd")
    await asyncio.Event().wait()

if __name__ == '__main__':
    # rclpy.init() was already called at top
    asyncio.run(ros_bridge())
