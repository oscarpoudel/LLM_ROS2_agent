# tools/ros_cli_tool.py
from pydantic import BaseModel
from pydantic_ai import Tool
from nats.aio.client import Client as NATS
# single NATS client for the tool
nc = NATS()

class RosCliParams(BaseModel):
    command: str

class RosCliResponse(BaseModel):
    success: bool
    payload: str

async def ros_cli_tool_fn(params: RosCliParams) -> RosCliResponse:
    print("*"*10 + "ros cli tool called"+"*"*10)
    # lazy‑connect
    if not nc.is_connected:
        await nc.connect("nats://localhost:4222")

    # send a request and wait for one reply
    msg = await nc.request("ros_cmd", params.command.encode(), timeout=5)
    output = msg.data.decode()

    return RosCliResponse(success=True, payload=output)

ros_cli_tool = Tool(
    ros_cli_tool_fn,
    name="ros_cli_tool",
    description="Send a ROS 2 CLI command " \
    " over NATS and return its output call this only for status related commands not for generic questions"
)
