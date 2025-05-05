import base64
import json
from typing import Optional
from pydantic import BaseModel
import asyncio
from pydantic_ai import Tool, BinaryContent, Agent
# from pydantic_ai.models.openai import OpenAIModel
# from pydantic_ai.providers.openai import OpenAIProvider
from nats.aio.client import Client as NATS
from models.models import llava_model
# # Configure LLaVA vision analysis agent

agent_vision = Agent(
    model=llava_model,
    system_prompt=(
        "You are a vision agent with strong image analysis and reading skills. "
        "Given image data, extract all relevant information."
    )
)

# NATS client for raw image retrieval
_vc = NATS()

class VisionParams(BaseModel):
    mode: str  # e.g., 'analyze_scene'

class VisionResponse(BaseModel):
    analysis: Optional[str]  # textual analysis by LLaVA

async def vision_tool_fn(params: VisionParams) -> VisionResponse:
    print("*"*10 + "vision  tool called"+"*"*10)
    # 1) retrieve raw image as base64 JSON from bridge
    if not _vc.is_connected:
        await _vc.connect("nats://localhost:4222")
    msg = await _vc.request("vision.cmd", params.json().encode(), timeout=10)
    payload = json.loads(msg.data.decode())
    image_b64 = payload.get('image', '')

    # 2) decode to bytes
    img_bytes = base64.b64decode(image_b64)

    # 3) let LLaVA agent analyze
    # note: block with run_sync for simplicity
    analysis_result = await asyncio.to_thread(
        lambda: agent_vision.run_sync([
            BinaryContent(data=img_bytes, media_type="image/jpeg")
        ])
    )
    print(analysis_result)
    # handle both structured and string returns
    if hasattr(analysis_result, "output") and hasattr(analysis_result.output, "content"):
        analysis_text = analysis_result.output.content
    else:
        analysis_text = str(analysis_result)  
    # extract the textual output
    # analysis_text = getattr(result.output, "content", None) or getattr(result, "output", str(result))

    return VisionResponse( analysis=analysis_text)

vision_tool = Tool(
    vision_tool_fn,
    name="vision_tool",
    description="Capture camera image and perform detailed analysis using a vision agent."
)