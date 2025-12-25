# Capstone Integration

## Purpose
Guide end-to-end integration of voice control, vision, manipulation, and navigation into a complete humanoid robot assistant system.

## When to Use
- Building a complete robot application from components
- Integrating multiple subsystems (speech, vision, planning, control)
- Testing full task pipelines end-to-end
- Deploying trained models to hardware
- Debugging multi-component system failures

## Book Chapters
- Chapter 12: Capstone Project - Voice-Controlled Robot Assistant

## Example Usage
```
User: "Help me integrate all components for a voice-controlled pick-and-place robot"

Claude: Using the capstone-integration skill, Chapter 12 provides the complete architecture:

System components (Chapter 12, Section 1.1):
1. Voice interface: wake word → ASR → LLM → TTS
2. Vision pipeline: camera → object detection → pose estimation
3. Manipulation: grasp planning → IK → motion planning → control
4. Navigation: SLAM → path planning → obstacle avoidance

Integration flow (Chapter 12, Section 1.2):
```
User: "Pick up the red cup and place it on the table"
  ↓
ASR (Whisper): transcribe command
  ↓
LLM (Llama): parse → ["detect red cup", "pick", "move", "place"]
  ↓
Vision (OWL-ViT): detect cup → 3D pose
  ↓
Manipulation (MoveIt): plan grasp → execute trajectory
  ↓
Navigation (Nav2): move to table
  ↓
Manipulation: place object
  ↓
TTS (Piper): "Task completed"
```

Testing strategy (Chapter 12, Section 1.3):
- Unit tests: test each component
- Integration tests: test component pairs
- End-to-end tests: 20 voice commands with 80%+ success

See Chapter 12 for deployment checklist and troubleshooting guide.
```
