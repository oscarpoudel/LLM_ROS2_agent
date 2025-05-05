import streamlit as st
from agent import run_agent_with_text  # This comes from your external agent.py

# ——————————————————————————————————————————————————————————————————
import nest_asyncio
import asyncio

nest_asyncio.apply()  # <-- Add this line at the top of your script

loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

def get_reply(prompt: str) -> str:
    try:
        return loop.run_until_complete(run_agent_with_text(prompt))
    except Exception as e:
        return f" Error: {e}"


# ——————————————————————————————————————————————————————————————————
# Streamlit UI setup
st.set_page_config(page_title="ROS-LM Agent", page_icon="🤖", layout="centered")

st.markdown(
    """
    <h1 style='text-align: center;'>🤖 ROS-LM: Robotics Language Agent</h1>
    <h4 style='text-align: center; color: gray;'>Control your robot using natural language 🚀</h4>
    <hr>
    """,
    unsafe_allow_html=True
)

# ——————————————————————————————————————————————————————————————————
# Session state for chat history
if "messages" not in st.session_state:
    st.session_state.messages = []

# Display chat history
for role, content in st.session_state.messages:
    st.chat_message(role).write(content)

# User input
user_input = st.chat_input("Type your command here...")

if user_input:
    # Save user message
    st.session_state.messages.append(("user", user_input))
    st.chat_message("user").write(user_input)

    # Get and display assistant reply
    with st.chat_message("assistant"):
        with st.spinner("Thinking..."):
            reply = get_reply(user_input)
            st.write(reply)
        st.session_state.messages.append(("assistant", reply))
