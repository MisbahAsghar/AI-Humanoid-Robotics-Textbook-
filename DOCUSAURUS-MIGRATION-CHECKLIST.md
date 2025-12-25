# Docusaurus Migration Checklist
## Transforming AI-Generated Textbook Chapters to MDX

**Purpose**: Convert long-form markdown chapters to Docusaurus-compatible MDX format
**Target**: Physical AI and Humanoid Robotics Textbook (12 chapters)
**Last Updated**: 2025-12-23

---

## 📋 Master Checklist

### Phase 1: File Structure
- [ ] Add frontmatter to all chapter files
- [ ] Add frontmatter to all summary files
- [ ] Add frontmatter to all reference files
- [ ] Rename files to match sidebars.js IDs
- [ ] Create `_category_.json` for each part directory
- [ ] Verify file paths match sidebar configuration

### Phase 2: Content Formatting
- [ ] Convert headings (fix special characters, add IDs)
- [ ] Transform code blocks (add language tags, titles)
- [ ] Convert callouts to Docusaurus admonitions
- [ ] Format math equations (inline and block)
- [ ] Update image references
- [ ] Create internal cross-references
- [ ] Format references section

### Phase 3: Validation
- [ ] Run `npm run build` to catch MDX errors
- [ ] Test all internal links
- [ ] Verify all images load
- [ ] Check math rendering (KaTeX)
- [ ] Test code syntax highlighting
- [ ] Validate responsive layout on mobile

---

## 1️⃣ Frontmatter Addition

### Rule: Every `.md` file needs YAML frontmatter

**BEFORE** (raw markdown):
```markdown
# Chapter 1: Introduction to Physical AI

This chapter covers...
```

**AFTER** (Docusaurus MDX):
```markdown
---
id: ch01-introduction-to-physical-ai
title: "Chapter 1: Introduction to Physical AI"
sidebar_label: "Ch 1: Intro to Physical AI"
sidebar_position: 1
description: "Foundations of embodied intelligence, sensor constraints, and the reality gap"
keywords: [physical ai, embodied intelligence, reality gap, sensor fusion]
---

# Chapter 1: Introduction to Physical AI

This chapter covers...
```

### Frontmatter Fields Explained

| Field | Required | Purpose | Example |
|-------|----------|---------|---------|
| `id` | ✅ Yes | Unique identifier (matches sidebars.js) | `ch01-introduction-to-physical-ai` |
| `title` | ✅ Yes | Browser tab title, SEO | `"Chapter 1: Introduction to Physical AI"` |
| `sidebar_label` | ⚠️ Recommended | Short name in sidebar | `"Ch 1: Intro to Physical AI"` |
| `sidebar_position` | ✅ Yes | Order in category | `1` |
| `description` | ⚠️ Recommended | SEO meta description | `"Foundations of embodied..."` |
| `keywords` | ❌ Optional | SEO keywords | `[physical ai, sensors]` |
| `tags` | ❌ Optional | For blog-style organization | `[foundations, theory]` |
| `hide_table_of_contents` | ❌ Optional | Hide TOC for short pages | `false` |

### Special Cases

**Summary Pages**:
```yaml
---
id: ch01-summary
title: "Chapter 1 Summary"
sidebar_label: "📝 Ch 1 Summary"
sidebar_position: 2
description: "Quick reference: key concepts, formulas, and takeaways"
---
```

**Reference Pages**:
```yaml
---
id: references
title: "Part 1 References"
sidebar_label: "📚 Part 1 References"
sidebar_position: 999
description: "Established, emerging, and tool documentation sources"
---
```

---

## 2️⃣ Heading Transformations

### Rule: Avoid special characters in headings (breaks anchor links)

**BEFORE** (problematic):
```markdown
## 2.1 RGB Cameras: FOV & Resolution
## What is ROS 2? (Architecture Overview)
## Section 3.5.2 — Advanced Topics
### Step #1: Install Dependencies
```

**AFTER** (Docusaurus-safe):
```markdown
## 2.1 RGB Cameras: FOV and Resolution {#rgb-cameras}
## What is ROS 2 (Architecture Overview) {#ros2-architecture}
## Section 3.5.2 - Advanced Topics {#advanced-topics}
### Step 1: Install Dependencies {#install-deps}
```

### Heading Best Practices

| Issue | Fix | Reason |
|-------|-----|--------|
| `&` symbol | Replace with `and` | Breaks URL encoding |
| `?` in heading | Move to parentheses or remove | Invalid anchor |
| `—` (em dash) | Replace with `-` (hyphen) | Encoding issues |
| `#` in heading | Use `Step 1:` instead | Conflicts with markdown syntax |
| `:` at end | OK to keep | Safe character |
| `/` forward slash | Replace with `or` | Breaks URL structure |

### Custom Anchor IDs

**Why**: Docusaurus auto-generates anchors, but you can control them for stable links.

```markdown
## This is a Very Long Heading That Would Generate an Ugly URL {#custom-id}
```

**Link to it**:
```markdown
See [Section 2.1](#custom-id) for details.
```

---

## 3️⃣ Code Block Formatting

### Rule: Always specify language + add titles for context

**BEFORE** (basic):
````markdown
```
import rclpy
from rclpy.node import Node

class MyNode(Node):
    pass
```
````

**AFTER** (Docusaurus-enhanced):
````markdown
```python title="simple_publisher.py" showLineNumbers
import rclpy
from rclpy.node import Node

class MyNode(Node):
    pass
```
````

### Supported Languages

```markdown
python | bash | shell | cpp | c | xml | yaml | json | javascript | typescript |
rust | go | java | csharp | dockerfile | sql | markdown | diff | ros | urdf
```

**Note**: `ros` and `urdf` may need custom Prism grammar (use `xml` as fallback).

### Code Block Enhancements

**1. Highlighting Lines**:
````markdown
```python {3-5,8} title="complex_node.py"
import rclpy
from rclpy.node import Node
# highlight-next-line
class ComplexNode(Node):
    def __init__(self):
        super().__init__('complex_node')
        # This line is highlighted
        self.publisher = self.create_publisher(String, 'topic', 10)
```
````

**2. Line Numbers**:
````markdown
```cpp showLineNumbers title="main.cpp"
#include <iostream>
int main() {
    std::cout << "Hello" << std::endl;
    return 0;
}
```
````

**3. Terminal Output** (no language tag):
````markdown
```bash title="Terminal Output"
$ ros2 topic list
/rosout
/parameter_events
/robot/cmd_vel
```
````

**4. Long Code Blocks** (use tabs for alternatives):
````markdown
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
  <TabItem value="python" label="Python">
    ```python
    # Python implementation
    ```
  </TabItem>
  <TabItem value="cpp" label="C++">
    ```cpp
    // C++ implementation
    ```
  </TabItem>
</Tabs>
````

### Common Pitfalls

❌ **Don't** use backticks inside code blocks (escape them):
````markdown
```python
# Wrong: `variable`
# Right: variable
```
````

❌ **Don't** forget closing fence:
````markdown
```python
# Missing closing ```
````

❌ **Don't** mix indentation (causes rendering issues):
````markdown
```python
def foo():
    pass  # 4 spaces
	pass  # tab - BREAKS RENDERING
```
````

---

## 4️⃣ Math Formula Formatting

### Rule: Use KaTeX syntax (must enable plugin in `docusaurus.config.js`)

**Enable in config**:
```javascript
{
  presets: [
    [
      'classic',
      {
        docs: {
          remarkPlugins: [require('remark-math')],
          rehypePlugins: [require('rehype-katex')],
        },
      },
    ],
  ],
  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.13.24/dist/katex.min.css',
      type: 'text/css',
      integrity: 'sha384-odtC+0UGzzFL/6PNoE8rX/SPcQDXBJ+uRepguP4QkPCm2LBxH3FA3y+fKSiJ+AmM',
      crossorigin: 'anonymous',
    },
  ],
}
```

### Inline Math

**BEFORE**:
```markdown
The velocity is v = d/t where d is distance.
```

**AFTER**:
```markdown
The velocity is $v = \frac{d}{t}$ where $d$ is distance.
```

### Block Math (Display Mode)

**BEFORE**:
```markdown
The quadratic formula is: x = (-b ± sqrt(b^2 - 4ac)) / 2a
```

**AFTER**:
```markdown
The quadratic formula is:

$$
x = \frac{-b \pm \sqrt{b^2 - 4ac}}{2a}
$$
```

### Common Math Patterns

| Concept | KaTeX Syntax |
|---------|--------------|
| Fraction | `$\frac{numerator}{denominator}$` |
| Square root | `$\sqrt{x}$` |
| Power | `$x^2$` or `$x^{10}$` |
| Subscript | `$x_i$` or `$x_{max}$` |
| Greek letters | `$\alpha, \beta, \gamma, \theta, \omega$` |
| Vectors | `$\vec{v}$` or `$\mathbf{v}$` |
| Matrix | `$\begin{bmatrix} a & b \\ c & d \end{bmatrix}$` |
| Summation | `$\sum_{i=1}^{n} x_i$` |
| Integral | `$\int_a^b f(x) dx$` |
| Partial derivative | `$\frac{\partial f}{\partial x}$` |

### Escaping Special Characters

**Problem**: `_` and `*` conflict with markdown italics.

**Solution 1** (Escape with backslash):
```markdown
The variable $a\_max$ is important. <!-- Wrong: renders as italic -->
The variable $a_{\text{max}}$ is correct.
```

**Solution 2** (Use \text{}):
```markdown
$P_{\text{sensor}}$ instead of $P_{sensor}$
```

### Math in Admonitions

```markdown
:::tip Formula Reference
The complementary filter is:

$$
\theta_{\text{fused}} = \alpha \cdot \theta_{\text{gyro}} + (1 - \alpha) \cdot \theta_{\text{accel}}
$$

where $\alpha = 0.98$ for typical IMU applications.
:::
```

---

## 5️⃣ Callout Conversion (Admonitions)

### Rule: Replace custom callouts with Docusaurus admonitions

Docusaurus supports: `note`, `tip`, `info`, `warning`, `danger`

### Standard Admonitions

**BEFORE** (generic markdown):
```markdown
> **Note**: This is important information.

> ⚠️ **Warning**: Hardware required for this section.

📝 **Key Takeaway**: Remember this concept.
```

**AFTER** (Docusaurus):
```markdown
:::note
This is important information.
:::

:::warning Hardware Required
Hardware required for this section.
:::

:::tip Key Takeaway
Remember this concept.
:::
```

### All Admonition Types

**1. Note** (neutral information):
```markdown
:::note
ROS 2 Humble is the LTS release supported until 2027.
:::
```

**2. Tip** (helpful suggestion):
```markdown
:::tip Pro Tip
Use `colcon build --symlink-install` for faster development iterations.
:::
```

**3. Info** (supplementary):
```markdown
:::info
This chapter assumes Ubuntu 22.04 LTS.
:::
```

**4. Warning** (caution):
```markdown
:::warning
Running `rm -rf /` will destroy your system. Never run as root.
:::
```

**5. Danger** (critical):
```markdown
:::danger Safety Critical
Humanoid robots can cause injury. Always enable emergency stop.
:::
```

### Custom Titles

```markdown
:::note Chapter Prerequisites
- Python 3.8+
- ROS 2 Humble installed
- 8GB RAM minimum
:::
```

### Nested Content

```markdown
:::warning Complex Setup Ahead
This section requires multiple dependencies:

1. NVIDIA Isaac Sim (30GB download)
2. RTX 2070+ GPU
3. 32GB RAM

See [Installation Guide](/docs/appendix/installation-guide) for details.

```bash
sudo apt install ros-humble-isaac-ros
```
:::
```

### Collapsible Admonitions (Custom)

```markdown
<details>
<summary>💡 Optional: Hardware Deployment Tips</summary>

:::tip
If deploying to real hardware:
- Use `ros2_control` for motor abstraction
- Add hardware safety limits in URDF
- Test in simulation first
:::

</details>
```

---

## 6️⃣ Image Reference Updates

### Rule: Move images to `/static/img/` and use absolute paths

**BEFORE** (relative paths in content folder):
```markdown
![Sensor Fusion Diagram](../../diagrams/Ch2/sensor_fusion.png)
![System Architecture](../diagrams/system.svg)
```

**AFTER** (Docusaurus static assets):
```markdown
![Sensor Fusion Diagram](/img/part-1/ch02-sensor-fusion.png)
![System Architecture](/img/part-1/ch02-system-architecture.svg)
```

### Migration Steps

**1. Reorganize Images**:
```bash
# From:
content/diagrams/Ch1/physical_ai_loop.png
content/diagrams/Ch2/sensor_fusion.png

# To:
static/img/part-1/ch01-physical-ai-loop.png
static/img/part-1/ch02-sensor-fusion.png
```

**2. Update All References**:
```bash
# Find and replace in chapters
# Pattern: ../../diagrams/Ch2/sensor_fusion.png
# Replace: /img/part-1/ch02-sensor-fusion.png
```

### Image Best Practices

**Add Alt Text** (accessibility + SEO):
```markdown
![Complementary filter block diagram showing gyroscope and accelerometer inputs](/img/part-2/ch02-complementary-filter.png)
```

**Use Markdown or HTML** (HTML for sizing):
```markdown
<!-- Markdown: simple -->
![Small icon](/img/icons/ros2-logo.png)

<!-- HTML: with size control -->
<img src="/img/part-4/ch07-isaac-sim-screenshot.png" alt="Isaac Sim interface" width="600" />
```

**Image with Caption** (custom component):
```jsx
import Figure from '@site/src/components/Figure';

<Figure
  src="/img/part-1/ch01-physical-ai-loop.png"
  alt="Physical AI perception-action loop"
  caption="Figure 1.1: The Physical AI loop showing sensor input, processing, and actuation"
/>
```

**Lazy Loading** (for large textbooks):
```markdown
<img src="/img/large-diagram.png" alt="Complex diagram" loading="lazy" />
```

---

## 7️⃣ Internal Cross-References

### Rule: Use relative doc paths or doc IDs for links

### Linking to Other Chapters

**BEFORE** (won't work in Docusaurus):
```markdown
See [Chapter 3](Ch3.md) for ROS 2 details.
Review [Section 2.1](Ch2.md#section-2-1) for sensor types.
```

**AFTER** (Docusaurus doc links):
```markdown
See [Chapter 3](../part-2/ch03-ros2-fundamentals) for ROS 2 details.
Review [Section 2.1](./ch02-humanoid-sensor-systems#rgb-cameras) for sensor types.
```

### Link Patterns

**1. Same Part** (sibling doc):
```markdown
[Next Chapter](./ch02-humanoid-sensor-systems)
[Previous Chapter](./ch01-introduction-to-physical-ai)
```

**2. Different Part** (use relative path):
```markdown
[See Chapter 5](../../part-3/ch05-gazebo-simulation)
[Jump to Capstone](../../part-6/ch12-autonomous-humanoid-capstone)
```

**3. With Anchor** (link to section):
```markdown
[RGB Camera Section](./ch02-humanoid-sensor-systems#rgb-cameras)
[Installation Steps](../appendix/installation-guide#ros2-humble)
```

**4. External Links** (open in new tab):
```markdown
[ROS 2 Documentation](https://docs.ros.org/en/humble/) (external)

<!-- Or with explicit target -->
<a href="https://docs.ros.org" target="_blank" rel="noopener noreferrer">ROS 2 Docs</a>
```

### Navigation Components (Custom)

**Chapter Navigation** (add at bottom of chapters):
```jsx
import ChapterNav from '@site/src/components/ChapterNav';

<ChapterNav
  prev={{
    label: "Ch 1: Introduction to Physical AI",
    link: "../part-1/ch01-introduction-to-physical-ai"
  }}
  next={{
    label: "Ch 3: ROS 2 Fundamentals",
    link: "../part-2/ch03-ros2-fundamentals"
  }}
/>
```

### Breadcrumbs

Docusaurus auto-generates breadcrumbs from sidebar structure. No action needed.

---

## 8️⃣ References Section Formatting

### Rule: Use consistent citation format (IEEE, APA, or custom)

**BEFORE** (inconsistent):
```markdown
## References
1. Some paper about ROS
2. https://nvidia.com/isaac
3. Smith, J. (2024). "Robot Learning"
```

**AFTER** (structured):
```markdown
## References

### Established Sources (Peer-Reviewed)

1. **ROS 2 Design** - Maruyama, Y. et al. (2016). "Exploring the performance of ROS2." *IEEE International Conference on Embedded and Real-Time Computing Systems and Applications*. [DOI: 10.1109/RTCSA.2016.13](https://doi.org/10.1109/RTCSA.2016.13)

2. **Visual SLAM Survey** - Cadena, C. et al. (2016). "Past, present, and future of simultaneous localization and mapping." *IEEE Transactions on Robotics*, 32(6), 1309-1332. [DOI: 10.1109/TRO.2016.2624754](https://doi.org/10.1109/TRO.2016.2624754)

### Tool Documentation

3. **Isaac Sim Documentation** - NVIDIA Corporation. (2024). *Isaac Sim Documentation*. Retrieved from [https://docs.omniverse.nvidia.com/isaacsim](https://docs.omniverse.nvidia.com/isaacsim)

4. **ROS 2 Humble Documentation** - Open Robotics. (2024). *ROS 2 Humble Hawksbill Documentation*. Retrieved from [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)

### Emerging Sources (Industry/Preprints)

5. **RT-2 Model** - Brohan, A. et al. (2023). "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control." *arXiv preprint arXiv:2307.15818*. [arXiv:2307.15818](https://arxiv.org/abs/2307.15818)

6. **Tesla Optimus** - Tesla AI Team. (2024). "Optimus Gen 2 Update." Retrieved from [https://tesla.com/optimus](https://tesla.com/optimus)
```

### Citation Component (Custom)

```jsx
import Citation from '@site/src/components/Citation';

<Citation
  authors="Brohan, A. et al."
  year={2023}
  title="RT-2: Vision-Language-Action Models"
  venue="arXiv preprint"
  url="https://arxiv.org/abs/2307.15818"
  type="emerging"
/>
```

### Footnotes

```markdown
This approach is based on recent VLA research[^1].

[^1]: Brohan, A. et al. (2023). "RT-2: Vision-Language-Action Models." arXiv:2307.15818
```

---

## 9️⃣ Common Docusaurus Pitfalls

### ❌ Problem 1: JSX-like syntax breaks MDX

**BEFORE** (breaks):
```markdown
The sensor returns data in <x, y, z> format.
Use the <Tab> key to autocomplete.
```

**AFTER** (escaped):
```markdown
The sensor returns data in `<x, y, z>` format.
Use the `<Tab>` key to autocomplete.
```

### ❌ Problem 2: Unescaped curly braces

**BEFORE** (breaks):
```markdown
Use {variable} to access the value.
The JSON format is {"key": "value"}.
```

**AFTER** (escaped):
```markdown
Use `{variable}` to access the value.
The JSON format is `{"key": "value"}`.
```

### ❌ Problem 3: HTML comments don't work

**BEFORE** (breaks):
```markdown
<!-- This is a comment -->
```

**AFTER** (use JSX comments):
```markdown
{/* This is a comment */}
```

### ❌ Problem 4: Markdown tables with pipes in cells

**BEFORE** (breaks):
```markdown
| Command | Description |
|---------|-------------|
| ros2 topic list | grep /cmd | Lists topics |
```

**AFTER** (escape pipes):
```markdown
| Command | Description |
|---------|-------------|
| `ros2 topic list \| grep /cmd` | Lists topics |
```

### ❌ Problem 5: Indented code blocks (use fenced blocks)

**BEFORE** (inconsistent rendering):
```markdown
Some text:

    import rclpy
    from rclpy.node import Node
```

**AFTER** (use fenced blocks):
````markdown
Some text:

```python
import rclpy
from rclpy.node import Node
```
````

### ❌ Problem 6: Multiple H1 headings

**BEFORE** (breaks sidebar TOC):
```markdown
# Chapter 1: Introduction
# Section 1.1
# Section 1.2
```

**AFTER** (single H1):
```markdown
# Chapter 1: Introduction
## Section 1.1
## Section 1.2
```

### ❌ Problem 7: Empty links

**BEFORE** (breaks build):
```markdown
[See documentation]()
[Link text](undefined)
```

**AFTER** (remove or fix):
```markdown
[See documentation](/docs/guide)
<!-- Or remove if link unavailable -->
```

---

## 🔟 Validation Commands

### Pre-Migration
```bash
# Check for problematic patterns
grep -r "^\# " content/chapters/  # Multiple H1s
grep -r "<[^>]*>" content/chapters/ | grep -v "```"  # Unescaped HTML/JSX
grep -r "{[^}]*}" content/chapters/ | grep -v "```"  # Unescaped curly braces
```

### Post-Migration
```bash
# Build and catch MDX errors
npm run build

# Expected output: BUILD SUCCESS
# If errors, read the stack trace for line numbers
```

### Link Validation
```bash
# Install link checker
npm install -g markdown-link-check

# Run on all chapters
find docs/ -name "*.md" -exec markdown-link-check {} \;
```

### Accessibility Check
```bash
# Install pa11y
npm install -g pa11y

# Test built site
npm run serve
pa11y http://localhost:3000/docs/physical-ai-book/part-1/ch01-introduction-to-physical-ai
```

---

## 🛠️ Automated Migration Script

```bash
#!/bin/bash
# migrate-chapter.sh - Convert single chapter to Docusaurus format

CHAPTER=$1  # e.g., "Ch1"
PART=$2     # e.g., "part-1"
NUM=$3      # e.g., "01"

# 1. Add frontmatter (manual step - see template above)

# 2. Fix headings (remove special chars)
sed -i 's/&/and/g' "content/chapters/${CHAPTER}.md"
sed -i 's/—/-/g' "content/chapters/${CHAPTER}.md"

# 3. Fix image paths
sed -i "s|../../diagrams/${CHAPTER}/|/img/${PART}/ch${NUM}-|g" "content/chapters/${CHAPTER}.md"

# 4. Convert callouts
sed -i 's/> \*\*Note\*\*:/:::note/g' "content/chapters/${CHAPTER}.md"
sed -i 's/> ⚠️ \*\*Warning\*\*:/:::warning/g' "content/chapters/${CHAPTER}.md"
# Add closing ::: manually (context-dependent)

# 5. Copy to docs folder
mkdir -p "docs/physical-ai-book/${PART}"
cp "content/chapters/${CHAPTER}.md" "docs/physical-ai-book/${PART}/ch${NUM}-chapter-name.md"

echo "✅ Migrated ${CHAPTER} to docs/${PART}/"
echo "⚠️ Manual steps remaining:"
echo "   1. Add frontmatter"
echo "   2. Close admonitions (:::)"
echo "   3. Verify build with: npm run build"
```

---

## 📚 Example: Full Chapter Transformation

### BEFORE (Original AI-Generated)
```markdown
# Chapter 2: Humanoid Sensor Systems

> **Note**: This chapter requires basic Python knowledge.

## 2.1 RGB Cameras: FOV & Resolution

RGB cameras are crucial for vision tasks.

![Diagram](../../diagrams/Ch2/camera.png)

The pinhole model is: f = Z * (x/X)

> ⚠️ **Hardware Required**: USB webcam for examples.

## Code Example

```
import cv2
cap = cv2.VideoCapture(0)
```

See [Chapter 1](Ch1.md) for background.

## References
1. Some camera paper
2. https://opencv.org
```

### AFTER (Docusaurus-Ready)
```markdown
---
id: ch02-humanoid-sensor-systems
title: "Chapter 2: Humanoid Sensor Systems"
sidebar_label: "Ch 2: Sensor Systems"
sidebar_position: 2
description: "RGB cameras, depth sensors, LiDAR, IMUs, and sensor fusion for humanoid robots"
keywords: [sensors, cameras, lidar, imu, sensor fusion]
---

# Chapter 2: Humanoid Sensor Systems

:::note Prerequisites
This chapter requires basic Python knowledge.
:::

## 2.1 RGB Cameras: FOV and Resolution {#rgb-cameras}

RGB cameras are crucial for vision tasks.

![Camera pinhole model diagram](/img/part-1/ch02-camera-pinhole-model.png)

The pinhole model is:

$$
f = Z \cdot \frac{x}{X}
$$

:::warning Hardware Required
USB webcam needed to run code examples. See [Hardware Guide](/docs/appendix/hardware-bom) for recommendations.
:::

## Code Example

```python title="camera_capture.py" showLineNumbers
import cv2

# Open default camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Cannot access camera")
    exit()
```

See [Chapter 1](../part-1/ch01-introduction-to-physical-ai#physical-ai-definition) for background on Physical AI.

## References

### Established Sources

1. **Pinhole Camera Model** - Hartley, R. & Zisserman, A. (2004). *Multiple View Geometry in Computer Vision* (2nd ed.). Cambridge University Press. [DOI: 10.1017/CBO9780511811685](https://doi.org/10.1017/CBO9780511811685)

### Tool Documentation

2. **OpenCV Documentation** - OpenCV Team. (2024). *OpenCV 4.x Documentation*. Retrieved from [https://docs.opencv.org/4.x/](https://docs.opencv.org/4.x/)

---

import ChapterNav from '@site/src/components/ChapterNav';

<ChapterNav
  prev={{label: "Ch 1: Introduction to Physical AI", link: "../part-1/ch01-introduction-to-physical-ai"}}
  next={{label: "Ch 3: ROS 2 Fundamentals", link: "../part-2/ch03-ros2-fundamentals"}}
/>
```

---

## ✅ Final Checklist Summary

**Before deploying**:
- [ ] All chapters have frontmatter
- [ ] No special characters in headings (`&`, `?`, `—`)
- [ ] All code blocks have language tags
- [ ] Math uses `$...$` (inline) and `$$...$$` (block)
- [ ] Callouts converted to `:::admonition`
- [ ] Images moved to `/static/img/` with absolute paths
- [ ] Internal links use relative doc paths
- [ ] No multiple H1 headings per file
- [ ] No unescaped `<>` or `{}` outside code blocks
- [ ] `npm run build` succeeds without errors
- [ ] All images load (check Network tab)
- [ ] Math renders correctly (check browser console)
- [ ] Internal links work (click through all chapters)
- [ ] Mobile responsive (test on 320px viewport)

**Tools**:
- `npm run build` - Catch MDX errors
- `npm run serve` - Local testing
- `markdown-link-check` - Validate links
- `pa11y` - Accessibility testing

---

**Document Version**: 1.0
**Last Updated**: 2025-12-23
**Compatible With**: Docusaurus 3.x, Physical AI Textbook v1.0
**Maintainer**: Documentation Engineering Team
