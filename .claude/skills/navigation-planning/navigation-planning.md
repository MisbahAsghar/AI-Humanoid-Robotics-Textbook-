# Navigation Planning

## Purpose
Guide implementation of path planning (A*, RRT), inverse kinematics, and motion planning for humanoid navigation and manipulation tasks.

## When to Use
- Planning collision-free paths in cluttered environments
- Solving inverse kinematics for arm reaching
- Implementing local obstacle avoidance
- Integrating SLAM for autonomous navigation
- Planning whole-body motions for humanoids

## Book Chapters
- Chapter 11: Navigation and Motion Planning

## Example Usage
```
User: "Implement collision-free path planning for a robot arm"

Claude: Using the navigation-planning skill, Chapter 11 Section 1.1 covers algorithms:

For high-dimensional spaces (7-DOF arm):

RRT (Rapidly-exploring Random Tree):
```python
def rrt_plan(start, goal, obstacles):
    tree = Tree(start)
    for i in range(max_iterations):
        q_rand = sample_random_config()
        q_near = tree.nearest(q_rand)
        q_new = extend(q_near, q_rand, step_size)

        if collision_free(q_near, q_new, obstacles):
            tree.add_node(q_new, parent=q_near)

            if distance(q_new, goal) < threshold:
                return extract_path(tree, q_new)

    return None
```

Typical parameters:
- max_iterations: 5000
- step_size: 0.1 rad
- threshold: 0.05 rad

See Chapter 11, Section 1.1 for A* and DWA alternatives.
```
