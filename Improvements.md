# Main Ideas for Improvement

## 1. Enhanced dynamic obstacle avoidance strategy

Currently, the **Full Coverage Path Planner (FCPP) generates a single, complex cleaning path at the start of the run**.
While this works well in static environments, it creates challenges when large, dynamic obstacles block the robot’s path.

The main challenge lies in the tradeoff between two goals:

- **Sticking to the full coverage path** to ensure complete and efficient cleaning.

- **Deviating around dynamic obstacles** (that are not part of the static map) to keep the robot moving safely.

Since the path is only generated once, the robot cannot easily replan when blocked.
This often forces a choice between stopping completely or deviating in ways that disrupt the original coverage pattern.

Below are two potential strategies to address this limitation:

### Approach A: Retrigger the Full Coverage Path

The idea here is to **regenerate the full coverage path** when the robot encounters a situation where it is completely blocked.

- **How it would work:**

    - Continuously monitor which areas have already been cleaned.

    - If the robot becomes stuck or the current path is blocked, regenerate the coverage path **only for the remaining uncleaned areas**.

    - This ensures the robot can continue cleaning without repeating areas unnecessarily.

- **Advantages:**

    - Maintains **high coverage quality** by always planning over the entire uncleaned area.

    - More **robust handling of blocked situations**, ensuring the robot can continue working without manual intervention.

- **Challenges:**

    - Tracking previously covered areas accurately in real-time and mark them as occupied.

    - Handling cases where the robot must traverse already-cleaned zones to reach a new area, without incorrectly cleaning those areas again.


This approach would likely be robust and maintain high coverage quality, but it comes with added complexity.



### Approach B: Custom Behavior Tree (BT) Plugin

Another potential solution is to develop a **custom BT plugin within Nav2 to handle blocked paths more reactively.**

- **How it would work:**

    When the robot detects that it is stuck, the plugin would:

    - Identify the next valid waypoint in the coverage plan that is currently unoccupied.

    - Trigger a shortest-path algorithm (e.g., A*) to navigate directly to that waypoint.

    - Once reached, resume the full coverage plan from there.

- **Advantages:**

    - More lightweight than fully regenerating the entire coverage path.

    - Keeps the robot moving forward even in dynamic and cluttered areas.

- **Challenges:**

    - A next valid waypoint could lead to dead-end situations, where the robot reaches a blocked zone again.

    - This approach may be less robust overall, especially in environments with many moving obstacles.

This strategy prioritizes reactivity and simplicity, but may require fine-tuning and careful handling of edge cases to prevent ineffective navigation.

## 2. Integrating Opennav Coverage into the real robot for finer control of coverage areas

Currently, **Opennav Coverage** has only been tested in simulation, while the real robot relies on FCPP.
**Integrating it into the real robot would offer a more flexible alternative, ideal for cleaning specific, high-priority zones.**

**Advantages**

- **Controlled coverage areas:**
Unlike FCPP, which plans for the entire map, Opennav Coverage lets you define specific regions to clean, enabling more targeted and efficient tasks.

- **Future-ready with map decomposition:**
The current version cannot handle obstacles, but upcoming **map decomposition features** will allow it to create obstacle-aware subregions.
This could make it a robust and adaptive solution, potentially surpassing FCPP in environments that require more precise and flexible cleaning behavior.
It’s worth **keeping an eye** on the [opennav_coverage](https://github.com/open-navigation/opennav_coverage) repo, as they are working in collaboration with [FieldsToCover](https://github.com/Fields2Cover/Fields2Cover), which already has map decomposition capabilities that may soon be integrated.

**Challenges**

- **Limited to obstacle-free areas** until map decomposition is released.

- Not yet suitable for industrial settings like a sawmill.


## 3. Adding higher-level error handling and notification logic

Implement a system to detect when the robot is completely stuck or unable to continue due to various reasons such as:

- Being blocked by a large dynamic obstacle.

- Low battery voltage.

- Other issues, e.g., hose detected or tangled.

When such an event occurs, the robot should automatically **send an email via SMTP** to an operator with:

- The captured image.

- A clear status message, e.g., "<em>LOW BATTERY VOLTAGE", "BIG OBSTACLE DETECTED",</em> or <em>"HOSE DETECTED"</em>.

This would allow operators to quickly assess problems remotely and take appropriate action, greatly improving operational oversight and reducing downtime.



# Other Areas of Improvement (Lower Priority)

## 4. Hose detection pipeline

Hoses in the sawmill present a unique challenge because they can take many shapes and layouts, confusing navigation and causing entanglement.

An interesting improvement would be to *develop a custom hose detection pipeline**:

- Create a dataset of hose images from the sawmill.

- Train a model to recognize hoses in the environment.

- When detected, **send an SMTP** email to the operator with a picture and status update.

- Operator can then manually remove the hose.

## 5. Automated dumping process

Enable the robot to autonomously navigate to a dumping area when its dust container is full.

Safely perform the dumping action without human intervention and resume its mission.


## 6. Automated charging


Allow the robot to dock autonomously when the battery is low.
Explore [opennav_docking](https://github.com/open-navigation/opennav_docking) for implementation.

## 7. Cleaning progress & robot status dashboard

A dedicated GUI can be developed to provide operators with clear, real-time insights into the robot’s performance and state:

- **Cleaning Progress:**
Displays the percentage of the designated coverage area completed along with an estimated time remaining to finish the current mission.

- **Live Robot Status:**
Shows the current operating state, such as `ACTIVE`, `STOPPED`, `OBSTACLE DETECTED`, `HOSE DETECTED`, `LOW BATTERY VOLTAGE`, and other critical alerts.