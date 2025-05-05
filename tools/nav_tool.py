# tools/nav_tool.py
from pydantic import BaseModel
from pydantic_ai import Tool
from nats.aio.client import Client as NATS
from nats.errors import TimeoutError as NatsTimeout

tc = NATS()

class NavParams(BaseModel):
    location: str  # e.g. "kitchen"

class NavResponse(BaseModel):
    success: bool
    message: str

async def nav_tool_fn(params: NavParams) -> NavResponse:
    print("*"*10 + "nav2  tool called"+"*"*10)
    print(params)
    if not tc.is_connected:
        await tc.connect("nats://localhost:4222")
    try:
        msg = await tc.request("nav.cmd", params.json().encode(), timeout=50.0)
        return NavResponse(success=True, message=msg.data.decode())
    except NatsTimeout:
        return NavResponse(success=False, message="Timeout")
    except Exception as e:
        return NavResponse(success=False, message=f"Nav error: {e}")

nav_tool = Tool(
    nav_tool_fn,
    name="nav_tool",
    description="Navigate to a predefined location like 'bag', 'firehydrant', or 'boat'.",
)
