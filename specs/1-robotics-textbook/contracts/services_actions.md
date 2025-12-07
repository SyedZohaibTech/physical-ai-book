# ROS 2 Services & Actions

This document defines the key ROS 2 services and actions used in the textbook.

## Services

Services are used for request/response interactions that are expected to complete quickly.

| Service Name | Service Type | Description |
|---|---|---|
| `/get_action_plan` | `custom_interfaces/srv/GetActionPlan` | Requests a high-level task plan from the LLM planner based on a natural language command. |
| `/clear_costmaps` | `nav2_msgs/srv/ClearAllCostmaps` | A standard Nav2 service to clear the costmaps, resetting obstacle data. |
| `/spawn_entity` | `gazebo_msgs/srv/SpawnEntity` | Spawns a model into the Gazebo simulation. |

### Custom Service Definitions

#### `custom_interfaces/srv/GetActionPlan.srv`
```c
# Request
string command
---
# Response
string[] action_plan
```

---

## Actions

Actions are used for long-running, feedback-producing tasks.

| Action Name | Action Type | Description |
|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | The standard Nav2 action to command the robot to move to a specific pose. |
| `/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Sends a sequence of joint positions for an manipulator to follow. Used for arm movements. |
| `/execute_task_plan` | `custom_interfaces/action/ExecuteTaskPlan` | A high-level action to execute a multi-step plan from the LLM. |

### Custom Action Definitions

#### `custom_interfaces/action/ExecuteTaskPlan.action`
```c
# Goal
string[] task_plan
---
# Result
bool success
string final_status
---
# Feedback
string current_step
float32 percent_complete
```
