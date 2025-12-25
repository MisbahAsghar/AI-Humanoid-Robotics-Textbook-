# Vision-Language-Action Models

## Purpose
Guide implementation of VLA models (RT-1, RT-2, OpenVLA) for language-conditioned robot manipulation, including action primitives and LLM integration.

## When to Use
- Enabling natural language robot control ("pick the red cup")
- Implementing open-vocabulary object detection and grasping
- Building manipulation policies that generalize to new objects
- Integrating vision transformers with motor control
- Creating task planning with LLM high-level reasoning

## Book Chapters
- Chapter 9: Vision-Language-Action Models for Manipulation

## Example Usage
```
User: "How do I integrate an LLM with robot action primitives?"

Claude: Using the vision-language-action skill, Chapter 9 Section 1.2 shows the architecture:

Pipeline:
1. LLM parses command: "Pick up the red cup and place it on the table"
2. LLM outputs action sequence:
   ```python
   [
     {"action": "detect", "object": "red cup"},
     {"action": "pick", "target": detected_pose},
     {"action": "move_to", "location": "table"},
     {"action": "place", "target": table_pose}
   ]
   ```
3. Each primitive executes via motion planning + control

Action primitive library:
- pick(pose) → grasp planning + trajectory
- place(pose) → inverse kinematics + placement
- move_to(location) → navigation + collision avoidance

See Chapter 9, Section 1.2 for complete implementation with RT-2.
```
