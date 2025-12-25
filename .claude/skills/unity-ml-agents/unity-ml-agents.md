# Unity ML-Agents

## Purpose
Guide setup of Unity ML-Agents environments for training humanoid robot policies with PPO, including observation spaces, action spaces, and reward design.

## When to Use
- Training locomotion or manipulation policies with RL
- Designing reward functions for robot tasks
- Creating parallel training environments for faster learning
- Integrating trained policies with real robot hardware
- Visualizing robot behavior during training

## Book Chapters
- Chapter 6: Unity ML-Agents for Physical AI

## Example Usage
```
User: "Design a reward function for a humanoid walking task"

Claude: Using the unity-ml-agents skill, Chapter 6 Section 1.3.2 recommends:

Reward components:
1. Forward velocity: +1.0 * velocity_x (encourage movement)
2. Energy penalty: -0.01 * Σ(torque²) (smooth motions)
3. Upright bonus: +0.5 * (1 - |tilt_angle|) (stay balanced)
4. Fall penalty: -10.0 if height < 0.3m (avoid falling)

Total reward per step:
r_t = v_x - 0.01·Σ(τ²) + 0.5·(1-|θ|) - 10·fall

Tune coefficients based on task priority. See Chapter 6, Section 1.3 for manipulation rewards.
```
