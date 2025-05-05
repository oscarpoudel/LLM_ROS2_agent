# agent.py

import asyncio
from pydantic_ai import Agent
from tools.motion_tool import motion_tool, MotionResponse
from tools.vision_tool import vision_tool, VisionResponse
from tools.status_tool import status_tool, StatusResponse
from tools.ros_cli_tool import ros_cli_tool, RosCliResponse
from models.models import ollama_model
from models.models import light_ollama
from models.models import gemini_model
from tools.nav_tool import nav_tool
# ——————————————————————————————————————————————————————————————————
#  1) Intent‐classification agent
classifier_agent = Agent(
    model=gemini_model,
    tools=[],
    system_prompt="""
You are IntentClassifier. Given a user instruction, pick exactly one of these labels:
 • general      → casual conversation or questions about the assistant itself
 • motion       → any request to move or rotate the robot
 • vision       → any request to describe or analyze camera images
 • status       → any request for system metrics (battery, CPU, health, etc.)
 • ros_cli      → any request that looks like a ROS2 CLI command (e.g. “ros2 topic list”)
 • nav          → if it involves named places (e.g. "go to kitchen", "navigate to the lab")


Respond with ONLY the single word label (no punctuation, no extra text).

Examples:
  “Who are you?”            → general
  "go to kitchen"           → nav
  “Move forward 2 meters”   → motion
  “Rotate 90 degrees left”  → motion
  “What do you see?”        → vision
  “Show me camera feed”     → vision
  “Battery level?”          → status
  “ros2 topic list”         → ros_cli
  “Can you help me?”        → general
  “Turn left and move 1m”   → motion
""",
    user_role="USER",
    agent_role="CLASSIFIER",
)

async def classify_intent(text: str) -> str:
    res = await classifier_agent.run(text)
    label = getattr(res.output, "content", str(res.output)).strip().lower()
    # print(label)
    if label not in {"general", "motion", "vision", "status","nav", "ros_cli"}:
        return "general"
    # print(label)
    return label

# ——————————————————————————————————————————————————————————————————
# 2) Sub‐agents

# 2a) General chat (no tools)
agent_general = Agent(
    model=gemini_model,
    tools=[],
    system_prompt="""
You are ROS-LM, the robot assitstant which job is to control the robot. Your are not the robot itself.
Reply in first-person, short sentences.
Do NOT include any code or how-to instructions.
Answer casual or self-referential questions about yourself or robotics.

""",
    user_role="USER",
    agent_role="ROSA",
)
agent_general = Agent(
    model=ollama_model,
    tools=[],
    system_prompt="""
You are ROS-LM, the robot.  Reply in first-person, short sentences.
Do NOT include any code or how-to instructions.
Answer general questions about yourself or robotics.
""",
    user_role="USER",
    agent_role="ROSA",
)

agent_nav = Agent(
    model=ollama_model,
    tools=[nav_tool],
    system_prompt="""
You are ROS-LM, a navigation assistant for robots.

Only call the `nav_tool` with:
- location (string): the name of the place the robot should go.

DO NOT handle any free-form movement (distance or angle).
Examples:
- "Go to the kitchen" → {"location": "kitchen"}
- "Navigate to charging station" → {"location": "charging station"}
- "Head to the loading dock" → {"location": "loading dock"}
""",
    user_role="USER",
    agent_role="ROSA",
)


#move robot 1m forward in angle 30 to right
agent_motion = Agent(
    model=ollama_model,
    tools=[motion_tool],
    system_prompt="""
You are ROS-LM, a robot assistant.
Reply in first-person, short sentences AFTER the tool call succeeds.

NEVER include code or raw function outputs like dictionaries.
Only respond with a function call to `motion_tool` with two fields:
- distance: how far to move (in meters, positive = forward, negative = backward)
- angle: how much to rotate (in degrees, positive = left, negative = right)
- when told to rotate left its positve while told to rotate right its negative

DO NOT include any explanation or other response and strictly keep it in json format like the exampes given below

Examples:
- "Move 1 meter forward" → {"distance": 1.0, "angle": 0.0}
- "Rotate right 90 degrees" → {"distance": 0.0, "angle": -90.0}
- "Move forward 2m and turn left 45°" → {"distance": 2.0, "angle": 45.0}

DO NOT say anything before calling the tool.
NEVER say the JSON to the user.

AFTER the tool call **succeeds**, respond in first person and natural language summarizing the motion:
- Good: "I moved forward 1 meter."
- Good: "I turned 90 degrees to the right."
- Good: "I moved 2 meters forward and turned 45 degrees left."

If the tool fails, apologize and describe the failure.
""",
    user_role="USER",
    agent_role="ROSA",
)


agent_vision = Agent(
    model=ollama_model,
    tools=[vision_tool],
    system_prompt="""
You are ROS-LM, the robot.  Reply in first-person, short sentences.
Do NOT include any code or how-to instructions.
The user wants to know what you see.
Call vision_tool exactly once with no parameters, then summarize in one sentence.
""",
    user_role="USER",
    agent_role="ROSA",
)

agent_status = Agent(
    model=ollama_model,
    tools=[status_tool],
    system_prompt="""
You are ROS-LM, the robot.  Reply in first-person, short sentences.
Do NOT include any code or how-to instructions.
The user wants system status.
Call status_tool exactly once requesting battery, cpu, and health, then summarize.
""",
    user_role="USER",
    agent_role="ROSA",
)

agent_roscli = Agent(
    model=ollama_model,
    tools=[ros_cli_tool],
    system_prompt="""
You are ROS-LM, the robot.  Reply in first-person, short sentences.
Do NOT include any code or how-to instructions.
The user’s input is a ros2 CLI command.
Call ros_cli_tool exactly once with that command, then summarize.
""",
    user_role="USER",
    agent_role="ROSA",
)


AGENT_MAP = {
    "general": agent_general,
    "motion":  agent_motion,
    "vision":  agent_vision,
    "status":  agent_status,
    "ros_cli": agent_roscli,
    "nav":     agent_nav,
}
# ——————————————————————————————————————————————————————————————————
# 3) Main loop

# ——————————————————————————————————————————————————————————————————
# Run agent based on input string (for Streamlit or CLI)
async def run_agent_with_text(text: str) -> str:
    intent = await classify_intent(text)
    
    agent = AGENT_MAP[intent]
    result = await agent.run(text)
    out = result.output

    if isinstance(out, MotionResponse):
        return out.details
    elif isinstance(out, VisionResponse):
        return out.summary
    elif isinstance(out, StatusResponse):
        return ", ".join(f"{k}: {v}" for k, v in out.metrics.items())
    elif isinstance(out, RosCliResponse):
        return out.output
    else:
        return getattr(out, "content", str(out))

# CLI entry point
if __name__ == "__main__":
    async def main():
        print("ROS-LM Agent ready. (type 'exit' to quit)\n")
        while True:
            text = input("You: ").strip()
            if text.lower() in {"exit", "quit"}:
                break
            reply = await run_agent_with_text(text)
            print("ROS-LM:", reply)
    asyncio.run(main())

