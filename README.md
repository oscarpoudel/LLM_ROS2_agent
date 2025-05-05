html_readme = """
<h1>🤖 ROS-LM: Natural Language Robot Assistant</h1>
<p>ROS-LM is a multimodal LLM-powered agent that enables <strong>natural language control</strong> of a ROS2 robot using NATS-based communication. It supports robot motion, waypoint navigation, vision queries, system diagnostics, and ROS2 CLI control—via CLI or Streamlit web app.</p>

<hr>

<h2>🗂️ Project Layout</h2>
<pre><code>.
├── agent.py                 # CLI-based interface to run ROS-LM agent
├── streamlit_app.py         # Streamlit web interface
├── .env                     # Optional environment file
├── tools/                   # Agent tools that send NATS messages
│   ├── motion_tool.py
│   ├── nav_tool.py
│   ├── ros_cli_tool.py
│   ├── status_tool.py
│   └── vision_tool.py
├── models/
│   └── models.py            # Ollama + Gemini model wrappers
├── ros2_side/               # ROS2-side NATS receivers and nav utilities
│   ├── recevie_nav2_motion.py
│   └── robot_navigator_tf2.py
</code></pre>

<h2>🧠 System Overview</h2>
<ul>
<li><strong>User</strong> types natural language commands (via web UI or terminal).</li>
<li><strong>Intent Classifier</strong> categorizes the instruction (motion, nav, vision, etc.).</li>
<li><strong>Agent Tool</strong> maps to a backend ROS2 action.</li>
<li><strong>NATS Messaging</strong> bridges LLM frontend with ROS2.</li>
<li><strong>ROS2</strong> executes the physical action (move, navigate, query, etc.).</li>
</ul>

<h2>🔌 Requirements</h2>
<ul>
<li>Python ≥ 3.9</li>
<li>ROS 2 (Humble or Foxy) with Nav2 stack</li>
<li><a href="https://ollama.com" target="_blank">Ollama</a> or Gemini models for LLM</li>
<li>NATS server</li>
</ul>

<h3>Install Python requirements</h3>
<pre><code>pip install -r requirements.txt</code></pre>

<pre><code>streamlit
pydantic
pydantic-ai
nats-py
aiohttp
python-dotenv
</code></pre>

<h2>🐢 ROS 2 Setup</h2>
<ol>
<li>Launch your robot (real or simulated) with Nav2:
<pre><code>ros2 launch turtlebot3_navigation2 navigation2.launch.py</code></pre>
</li>
<li>Run the ROS2-side receiver that listens for LLM commands:
<pre><code>python ros2_side/recevie_nav2_motion.py</code></pre>
</li>
</ol>

<h2>🚀 Start NATS Server</h2>
<pre><code>docker run -d --name nats -p 4222:4222 nats</code></pre>

<h2>💬 Start the Agent</h2>
<h3>Option 1: Streamlit Web Interface</h3>
<pre><code>streamlit run streamlit_app.py</code></pre>
<ul>
<li>User-friendly web UI</li>
<li>Displays assistant replies and tool execution results</li>
</ul>

<h3>Option 2: Command-Line Interface</h3>
<pre><code>python agent.py</code></pre>
<ul>
<li>CLI interaction</li>
<li>Prints debug output and response directly</li>
</ul>

<h2>🛠️ Agent Capabilities</h2>
<table border="1" cellspacing="0" cellpadding="6">
<thead>
<tr>
<th>Intent</th><th>Trigger Phrase Example</th><th>Tool Used</th><th>Result in ROS2</th>
</tr>
</thead>
<tbody>
<tr><td>motion</td><td>"Move forward 1 meter"</td><td>motion_tool</td><td>Sends JSON <code>{distance: 1.0, angle: 0.0}</code></td></tr>
<tr><td>motion</td><td>"Turn left 90 degrees"</td><td>motion_tool</td><td><code>{distance: 0.0, angle: 90.0}</code></td></tr>
<tr><td>nav</td><td>"Go to the lab"</td><td>nav_tool</td><td>Sends goal name to Nav2 goal handler</td></tr>
<tr><td>vision</td><td>"What do you see?"</td><td>vision_tool</td><td>Snapshot from camera + analysis</td></tr>
<tr><td>status</td><td>"What's the battery level?"</td><td>status_tool</td><td>Replies with CPU, battery, and system health</td></tr>
<tr><td>ros_cli</td><td>"ros2 topic list"</td><td>ros_cli_tool</td><td>Sends ROS2 CLI command and returns output</td></tr>
<tr><td>general</td><td>"Who are you?"</td><td>(no tool)</td><td>General LLM response</td></tr>
</tbody>
</table>

<h2>⚠️ Notes & Tips</h2>
<ul>
<li><strong>Don’t run both Streamlit and CLI</strong> simultaneously on the same NATS topics.</li>
<li><strong>Streamlit UI</strong> may error after one interaction due to <code>asyncio</code> event loop—already handled in latest version.</li>
<li><strong>Customize tool files</strong> if using different robot topics or frames.</li>
</ul>

<h2>📬 Author</h2>
<p>Maintained by Oscar Poudel<br>
<a href="https://github.com/oscarpoudel">GitHub</a> | <a href="https://www.linkedin.com/in/oscar-poudel/">LinkedIn</a></p>
"""
