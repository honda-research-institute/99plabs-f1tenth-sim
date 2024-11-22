# Getting Started - Simulation
[![Simulator Run](https://markdown-videos-api.jorgenkh.no/url?url=https%3A%2F%2Fwww.youtube.com%2Fwatch%3Fv%3DU_jUog8-W-c)](https://www.youtube.com/watch?v=U_jUog8-W-c)

> **PREREQUISITES:**
>
>[Docker](https://docs.docker.com/engine/install/) and [Docker Compose](https://docs.docker.com/compose/install/)


0. **Clone the repo and build the containers**

First, clone this repo and navigate to the `99plabs-f1tenth-sim` folder:

```bash
git clone <repo-url-here>
cd 99plabs-f1tenth-sim
```

Next, run the following command to build the containers:
```bash
docker compose up -d
```

1. **Make sure your Docker image has finished building, then enter the container.**
    * There should be two containers: `99p_gym_sim` and `99p_gym_sim-novnc-1`
    * Go into the container by running `docker exec -it 99p_gym_sim /bin/bash`

2. **Source & Build**
    * Run the following commands (you should be in the `/sim_ws` directory)
    ```bash
    source /opt/ros/${ROS_DISTRO}/setup.bash
    colcon build --packages-select 99p_gym_sim car_navigation f110_description f1_task_commander autonomous_control
    source install/setup.bash
    ```

3. **Launch the simulator**
    * Run `ros2 launch 99p_gym_sim gym_simulator_launch.py`
    * Navigate to http://localhost:8080/vnc.html in a browser and click `Connect`. You should see an RViz window with the simulator.

4. **Initialize the car** (optional)
    * The car will be initialized on startup but if you'd like to start in a different spot, you can give the car an initial pose by using the `2D Pose Estimate` button and clicking and dragging anywhere on the map
    * You can use this button to reset the car at any point. This is especially helpful if the car gets stuck at any point.
    
5. **Send the car a navigation goal(s)**
    * Set a navigation goal using the `Nav2 Goal` button for a single goal or the `Waypoints / Nav Through Poses Mode` button for multiple goals.

# License
The SOMEThings Project is released under the [MIT License](https://opensource.org/license/MIT).
