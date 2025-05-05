# tools/motion_tool.py
from typing import Optional
from pydantic import BaseModel, root_validator
from pydantic_ai import Tool
from nats.aio.client import Client as NATS
from nats.errors import TimeoutError as NatsTimeout



class MotionParams(BaseModel):
    distance: Optional[float] = 0.0  # Forward/backward in meters
    angle: Optional[float] = 0.0     # Left/right turn in degrees

    @root_validator(pre=True)
    def validate_motion(cls, values):
        d, a = values.get("distance"), values.get("angle")
        if d == 0 and a == 0:
            raise ValueError("Must specify at least one of 'distance' or 'angle'")
        values["distance"] = float(d or 0.0)
        values["angle"] = float(a or 0.0)
        return values

class MotionResponse(BaseModel):
    success: bool
    details: str

async def motion_tool_fn(params: MotionParams) -> MotionResponse:
    print("********** motion tool called **********")
    print(params.json().encode())
    tc = NATS()
    # if not tc.is_connected:
    print("tc connected")
    await tc.connect("nats://localhost:4222")

    try:
        msg = await tc.request("motion.cmd", params.json().encode(), timeout=50.0)
        print('success===',msg.data.decode())
        return MotionResponse(success=True, details=f"the robot movement is {msg.data.decode()}")
    except NatsTimeout:
        return MotionResponse(success=False, details="Timeout talking to robot")
    except Exception as e:
        return MotionResponse(success=False, details=f"Motion error: {e}")


motion_tool = Tool(
    motion_tool_fn,
    name="motion_tool",
    description=(
        "Move and/or rotate the robot in one action.\n"
        "- distance (meters): +forward, -backward\n"
        "- angle (degrees): +counterclockwise (left), -clockwise (right)\n"
        "Examples:\n"
        "- move forward 2 meters: {distance: 2.0, angle: 0.0}\n"
        "- rotate left 90 degrees: {distance: 0.0, angle: 90.0}\n"
        "- turn right 45 degrees: {distance: 0.0, angle: -45.0}"
    ))
