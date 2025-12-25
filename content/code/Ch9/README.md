# Chapter 9 Code Examples

**Chapter**: VLA Pipeline Architecture
**Purpose**: Language-conditioned robot control, multimodal AI integration

---

## Requirements

- ROS 2 Humble
- Python 3.10+
- OpenAI API key (for GPT-4) OR Ollama (for local Llama 3)
- Transformers library (Hugging Face)
- PyTorch
- Camera (real or simulated)

---

## Example Scripts

### `llm_task_planner.py`
**Demonstrates**: LLM integration for task planning from natural language

**What it does**:
- ROS 2 node subscribing to `/voice_command` topic
- Calls GPT-4 API to decompose commands into action primitives
- Publishes action sequence to `/action_sequence`
- Handles errors, retries

**Running**:
```bash
export OPENAI_API_KEY="sk-..."
ros2 run vla_system llm_task_planner

# Send command
ros2 topic pub /voice_command std_msgs/msg/String "data: 'Pick up the red cup'" --once
```

---

### `vision_grounding.py`
**Demonstrates**: Open-vocabulary object detection with OWL-ViT

**What it does**:
- Subscribes to `/camera/image_raw`
- Uses OWL-ViT to detect objects matching text query
- Publishes object pixel coordinates to `/object_position`
- Default query: "red cup" (modify in code)

**Dependencies**:
```bash
pip install transformers torch opencv-python
```

**Running**:
```bash
ros2 run vla_system vision_grounding
```

---

### `action_primitives.py`
**Demonstrates**: Skill library (pick, place, navigate) using MoveIt

**What it provides**:
- `ActionPrimitives` class with methods: `pick(pose)`, `place(pose)`, `navigate_to(pose)`
- Each primitive: plans motion, executes, returns success/failure
- Integration with MoveIt for manipulation

**Usage** (in other nodes):
```python
from action_primitives import ActionPrimitives

primitives = ActionPrimitives()
success = primitives.pick(object_pose)
```

---

## File Structure

```
vla_system/
├── llm_task_planner.py
├── vision_grounding.py
└── action_primitives.py
```

---

## Common Errors

**1. OpenAI API authentication failed**
- Fix: Set `export OPENAI_API_KEY=...` before running

**2. OWL-ViT model download failed**
- Fix: Ensure internet connection, or pre-download: `transformers-cli download google/owlvit-base-patch32`

**3. MoveIt not found (action_primitives)**
- Fix: `sudo apt install ros-humble-moveit`

---

**Last Updated**: 2025-12-23
