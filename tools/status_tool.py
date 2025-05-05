from typing import List, Dict, Any
from pydantic import BaseModel
from pydantic_ai import Tool
from nats.aio.client import Client as NATS
from typing import Optional

# Single NATS client for status queries
sc = NATS()

class StatusParams(BaseModel):
    queries: List[str]                # e.g. ["battery", "cpu", "health"]

class StatusResponse(BaseModel):
    metrics: Optional[str] 

async def status_tool_fn(params: StatusParams) -> StatusResponse:
    print("*"*10 + "status tool called"+"*"*10)
    if not sc.is_connected:
        await sc.connect("nats://localhost:4222")

    msg = await sc.request(
        "status.cmd", params.json().encode(), timeout=5
    )
    # Parse or decode as needed
    return StatusResponse(metrics=msg.data.decode())

# Use positional function argument, not `func=`
status_tool = Tool(
    status_tool_fn,
    name="status_tool",
    description=(
        "Query robot system status metrics (battery, cpu, health, etc.) when user asks for status of the robot this tool should be called"
        "via NATS subject 'status.cmd'."
    ),
)
