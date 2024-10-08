# Initial Commit

# Cylindrical Object Detection and Exclusion ROS 2 Package

This ROS 2 package performs the following tasks:

1. **Spawn a Cylindrical Object**:
   - The node `spawn_cylinder.cpp` uses the `/spawn_entity` service to place a cylindrical object (30 cm diameter) in the simulation.
   - The cylinder is modeled in the `cylinder_model.sdf` file.

2. **Detect and Exclude the Cylindrical Object from Mapping**:
   - The node `detect_cylinder.cpp` subscribes to laser scans and identifies the cylindrical object by processing the scan data.
   - Once detected, the cylindrical object is excluded from the map and its position is published using the `/marker` topic.

## Node Descriptions

### `spawn_cylinder.cpp`
- **Functionality**: Spawns a cylindrical object of known dimensions (30 cm diameter) at a specified location in the simulation.
- **Service Used**: `/spawn_entity`
- **Expected Behavior**: The cylindrical object is placed in the simulation environment at random coordinates chosen during the lab.

### `detect_cylinder.cpp`
- **Functionality**: Subscribes to laser scan data to identify and locate the cylindrical object. The object's coordinates are published as a marker.
- **Expected Behavior**: The cylindrical object is identified and displayed on the map as a marker, with the correct coordinate transformation applied.

## Expected Results
- **Cylindrical Object Detection**: The cylindrical object is placed and detected in random locations.
- **Map Exclusion**: The cylindrical object is successfully excluded from the map and visualized on the map as a marker.

### Task Overview
- **Task**: Exclude a known cylindrical object from the map.
- **Subtasks**:
  - Reading laser scans (10 points)
  - Identifying the cylindrical object (25 points)
  - Correct coordinate transformation (15 points)
