import subprocess

# Terminal 1: Start LLM and log its output cleanly
ollama_cmd = (
    "cd ~ && "
    "script -q -f /tmp/xarm_mistral.log ollama run xarm_mistral && "
    "ollama run xarm_mistral"
)

# Terminal 2: ROS2 logfile_publisher with build and environment
publisher_cmd = (
    "cd ~/llm_xarm/ros2_ws && "
    "colcon build && "
    "source install/setup.bash && "
    "ros2 run llm_chat logfile_publisher"
)

# Terminal 3: ROS2 llm_listener with build and environment
listener_cmd = (
    "cd ~/llm_xarm/ros2_ws && "
    "colcon build && "
    "source install/setup.bash && "
    "ros2 run llm_chat llm_listener"
)

# Launch Terminal 1: LLM with log
subprocess.Popen([
    "gnome-terminal", "--title=LLM", "--", "bash", "-c", f"{ollama_cmd}; exec bash"
])

# Launch Terminal 2: Publisher
subprocess.Popen([
    "gnome-terminal", "--title=Publisher", "--", "bash", "-c", f"{publisher_cmd}; exec bash"
])

# Launch Terminal 3: Listener
subprocess.Popen([
    "gnome-terminal", "--title=Listener", "--", "bash", "-c", f"{listener_cmd}; exec bash"
])
