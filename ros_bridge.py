import os
import asyncio
import json
import subprocess
from nats.aio.client import Client

async def ros_bridge():
    nats_url = os.getenv("NATS_URL", "nats://127.0.0.1:4222")
    nc = await Client.connect(nats_url)
    js = nc.jetstream()

    sub = await nc.subscribe("agent.commands")
    async for m in sub.messages:
        payload = json.loads(m.data.decode())
        cmd = payload["command"].split()
        # run ROS2 CLI, request JSON output where possible
        out = subprocess.check_output(cmd)
        # republish raw bytes (JSON or plain text) to ros.events
        await js.publish("ros.events", out)

if __name__ == "__main__":
    asyncio.run(ros_bridge())
