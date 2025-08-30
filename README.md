# CS5917
According to the graduation project plan on MyAberdeen, the project would be completed after completing the "mapping" and "localization" components of VSLAM.

However, after completing the "Mapping" and "Localization" components, my supervisor suggested a higher-level bonus: adding a "Drone Delivery" component. The goal of this component was to place a drone on the robot. Once the robot and the drone reached a specific target location, the drone would automatically take off. This simulated a logistics scenario where when a robot reaches the customer's building, carrying a drone and a package, the drone could automatically take off and deliver the package through the window to the customer's home upstairs, eliminating the need for climbing stairs.

After completing the "Drone Delivery" component, my supervisor suggested another bonus: adding a "Navigation" component. The goal of this component was to enable the robot to automatically reach a specified destination after setting it.

The followings are the use case and outcomes of this engineering.

## Use Case for this engineering
<img width="653" height="378" alt="image" src="https://github.com/user-attachments/assets/5435171e-9a3d-4703-a439-39c0df30596f" />

There are **four actors** in the use case:

• **User:** The user can control the robot to move around, set a target position, and monitor the robot’s status.

• **Robot:** The robot can perform mapping, localization, and navigation tasks, and can also deliver items using a drone.

• **Drone:** The drone can be controlled by the robot to deliver items to a specified target position.

• **PC:** The PC is a personal computer that runs the uoa_robot_drone engineering package, which includes nodes for monitoring the robot’s status and controlling the drone.

There are **four main use cases**:

• **Mapping:** The robot can perform mapping tasks to create a map of the environment.

• **Localization:** The robot can determine its position in the environment using the map created during the mapping stage.

• **Drone Delivery:** When the robot reaches a specific goal position, it can send delivery signals to the drone and the drone can take off when received the signal.

• **Navigation:** The robot can navigate to a specified target position using the map created during the mapping stage and the localization information.

## The videos of the four outcomes are as follows:

**01_mapping_stage_result:**
https://drive.google.com/file/d/11al-9DhPHJnq1jxqijq0lqNm_AnaX8nR/view?usp=sharing

**02_localization_stage_result:**
https://drive.google.com/file/d/1WZKoLgzzQFTjpgGtTFRKW1KEhexPZDjv/view?usp=sharing

**03_drone-delivery_stage_result:**
https://drive.google.com/file/d/1YilGqrHxlOcHsXs2Wp1xFBH9J66aiw5k/view?usp=sharing

**04_navigation_stage_result:**
https://drive.google.com/file/d/1W5bc4YC-2hCE3udT41sNmwdKw-nS9bXH/view?usp=sharing
