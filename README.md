# Back Massage Using TIAGo Pro
This repository contains all the necessary files to make the TIAGo Pro robot plan a trajectory on the back of the human while optimizing its arm configuration for a maximum manipulability.

## Setup

### Setting up the Docker Container
1. Install Docker
2. Use the following link to download PALs Docker Container for TIAGo Pro:

   https://cloud.pal-robotics.com/s/QtoRDxf32xcFQpP

   **Password**: XLkCmCoc

3. Launch the following command at the location of the downloaded script:
   ```
   bash kit-biorob-build-docker.sh --create
   ```
4. Start the container using:
   ```
   docker start -ai <container_id>
   ```
   To retrieve the container_id run `docker ps`
   

### Clone Git Repository
1. Clone the git repository into a folder "your_folder":
   ```
   git clone https://github.com/dkammaw/TiagoPro.git
   ```
2. Open the Repository in VS Code:
   ```
   cd path/to/your_folder
   code .
   ```
3. Copy the folder into your containers workspace:
   ```
   docker cp /path/to/your_folder <container_id>:/ros2_ws
   ```

### Setting up the workspace
1. Log into the container:
   ```
   docker exec -it <container_name> bash
   ```
2. Source the environment and build:
   ```
   source /opt/pal/alum/setup.bash
   colcon build --symlink-install
   ```
3. Direct into the workspace and source it before running any application:
   ```
   cd /ros2_ws
   source insall/setup.bash
   ```
   
## Simulation
1. Launch Gazebo simulation in a separate terminal:
   ```
   ros2 launch tiago_pro_gazebo tiago_pro_gazebo.launch.py
   ```
2. Launch RViz in a separate terminal:
   ```
   ros2 launch tiago_pro_moveit_config moveit_rviz.launch.py
   ```
3. In RViz add the PointCloud2 Display:
   1. On the left panel, click the "Add" button (this will add a new display).
   2. From the list of display types, choose PointCloud2 (under the "By topic" section).
   3. In the "PointCloud2" display section, click the "Topic" field and select the topic /filtered_cloud where the point cloud data is being published.

4. Start the simulation in your workspace:
   ```
   ros2 launch trajectory visual_tapping.launch.py
   ```
   





