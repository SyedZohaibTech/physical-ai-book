---
title: "5. Integrating Visual Perception into VLA"
sidebar_label: "Integrating VLA"
sidebar_position: 5
---

# 5. Integrating Visual Perception into VLA

Our VLA pipeline can now understand spoken language and translate it into a sequence of abstract actions. However, the actions currently rely on predefined locations ("kitchen", "bedroom") and abstract object names ("mug"). For true autonomy, our robot needs to **visually perceive** these objects and their locations in the real world.

This chapter bridges the gap between language commands and visual understanding, allowing our robot to dynamically identify and interact with objects.

## The Role of Visual Perception

When a human says, "Go to the red cup," the robot needs to:
1.  **Identify "red cup"**: This requires an object detection or segmentation model.
2.  **Locate "red cup"**: This requires extracting its 3D position in the robot's coordinate frame.
3.  **Associate "red cup" with a goal**: Pass its 3D pose to a navigation or manipulation skill.

## Visual-Language Models (VLMs) and Object Grounding

Modern approaches often use **Visual-Language Models (VLMs)** that can take both text and images as input. These models can perform tasks like:
-   **Object Grounding**: Given an image and a text query ("red cup"), highlight the region in the image corresponding to the object.
-   **Referring Expression Comprehension**: Identify specific objects based on descriptive phrases.

For our purposes, we'll simplify and integrate a separate object detection pipeline that will feed its results into our cognitive planner.

## Object Detection Pipeline

We can use a pre-trained object detection model (like YOLO, EfficientDet, or even a foundation model like Segment Anything Model (SAM) combined with Grounding DINO) to identify objects in the robot's camera feed.

This pipeline typically looks like this:

```mermaid
graph TD
    subgraph Robot Perception
        A[Camera (Image Topic)] --> B(Object Detection Node);
        B -- Detected Objects (Bounding Boxes, Classes) --> C(3D Pose Estimation Node);
    end
    C -- 3D Object Poses --> D[Object Database / World Model];
```

### Components:

1.  **Object Detection Node**: Subscribes to the robot's camera image topic (`/camera/image_raw`). It runs an object detection model and publishes a list of detected objects, including their class (e.g., "cup", "book") and 2D bounding boxes in the image.
2.  **3D Pose Estimation Node**: Takes the 2D bounding boxes from the object detection node and combines them with depth information (from a depth camera or stereo camera) to estimate the 3D position and orientation (pose) of the detected objects relative to the robot.
3.  **Object Database / World Model**: A component (can be a simple list or a more complex knowledge graph) that stores the 3D poses and properties of all known objects in the environment. This is what our LLM-driven cognitive planner will query.

## Updating the Cognitive Planning Pipeline

Now, let's integrate this visual perception into our existing pipeline. The key change is that when the LLM suggests an action like `pick_up(mug)`, our action executor no longer relies on an abstract "mug" but needs to query the object database for the *actual 3D pose* of a mug in the environment.

```mermaid
graph TD
    subgraph "Human Interface"
        A[Voice Command (Whisper)]
    end
    subgraph "Cognitive Layer"
        B(LLM Planner)
        C(Action Executor)
        D{Object Database (from Perception)}
    end
    subgraph "Robot Execution"
        E(Navigation / Manipulation Skills)
    end
    
    A -- Transcribed Text --> B;
    B -- Abstract Plan (e.g., pick_up(mug)) --> C;
    C -- "Query mug's pose?" --> D;
    D -- "Mug at [x,y,z]" --> C;
    C -- Execute concrete ROS 2 actions --> E;
```

### Modifying the Action Executor

Our `ActionExecutor` node needs to be updated to:
1.  **Maintain an internal representation** of known objects and their 3D poses (this could be a simple dictionary). This representation would be updated by subscribing to a `/detected_objects` topic published by our perception nodes.
2.  **Modify skill execution**: When it receives an action like `pick_up(object_name)`, it will:
    - Look up `object_name` in its internal object database to get its 3D pose.
    - Pass this 3D pose to the underlying `pick_up` skill (e.g., a manipulation action server).
    - If the object is not found, it might ask the LLM for clarification or execute a "search" behavior.

This integration makes the robot truly intelligent. Instead of relying on a predefined world, it actively senses and understands its surroundings to fulfill commands.

## Example: A Simplified Object Database

Let's assume we have a perception node that detects objects and publishes their 3D poses to a `/detected_objects` topic (e.g., `geometry_msgs/msg/PoseStamped`). Our `ActionExecutor` would subscribe to this.

```python
# Inside ActionExecutor __init__
# ...
self.object_poses = {} # Dictionary to store object_name -> PoseStamped
self.object_sub = self.create_subscription(
    PoseStamped, # Assuming one object at a time for simplicity
    '/detected_objects',
    self.object_detection_callback,
    10
)

def object_detection_callback(self, msg):
    # In a real system, msg would contain object name, class, etc.
    # For simplicity, let's assume it's always "mug" for now, or infer from topic name.
    # We would need a custom message type for multiple objects.
    object_name = "mug" # This needs to be smarter
    self.object_poses[object_name] = msg.pose
    self.get_logger().info(f"Updated pose for {object_name}: {msg.pose.position.x}, {msg.pose.position.y}")

async def execute_pick_up(self, object_name):
    if object_name in self.object_poses:
        target_pose = self.object_poses[object_name]
        self.get_logger().info(f"Executing pick_up for {object_name} at {target_pose.position.x}, {target_pose.position.y}")
        # Call a manipulation action client with target_pose
        # ... (simplified for concept)
        await self.execute_manipulation_action(target_pose)
    else:
        self.get_logger().warn(f"Object '{object_name}' not found in current perception.")
        # Perhaps trigger a search behavior or ask LLM for clarification
```

This updated action executor shows how the robot can dynamically retrieve object information, making its actions context-aware and responsive to the environment. In the next chapter, we'll bring all these components together in a complete humanoid demonstration.
