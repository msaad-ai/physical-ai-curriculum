# Diagram Prompt Template for Claude-Generated Images

## Standard Format

Use this format when requesting diagrams:

```markdown
<!-- DIAGRAM_PROMPT: [Brief description of the diagram needed] -->
```

## Example Diagram Prompts

### ROS 2 Architecture Diagram
```
Create a diagram showing the ROS 2 architecture with:
- Nodes as rectangular boxes
- Topics as arrows between nodes
- Services as bidirectional arrows
- Parameters as cloud shapes connected to nodes
- Label each component clearly
- Use a humanoid robot as the central theme
```

### Node Communication
```
Create a diagram showing publisher-subscriber communication in ROS 2:
- Publisher node sending messages to a topic
- Multiple subscriber nodes receiving from the same topic
- Show the data flow with arrows
- Include message types
```

### URDF Structure
```
Create a diagram showing a simple humanoid URDF structure:
- Base link (torso)
- Links for head, arms, legs
- Joints connecting the links
- Show the tree structure
```

## Guidelines

- Be specific about what components to include
- Mention the humanoid robotics context when relevant
- Specify the type of diagram (flowchart, structure, process, etc.)
- Request clear labeling of components
- Ask for appropriate colors or visual distinctions