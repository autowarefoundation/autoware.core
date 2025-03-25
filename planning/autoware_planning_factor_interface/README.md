# autoware_planning_factor_interface

## Overview

The `PlanningFactorInterface` is a C++ class designed to facilitate the addition and publication of planning factors.

## Design

The `PlanningFactorInterface` class is designed to be lightweight and efficient, with the following key components:

- **Add:** Methods to add planning factors to the interface.

- **Publisher:** The class includes a publisher for `PlanningFactorArray` messages, which are used to distribute planning factors to other nodes in the system.

The design emphasizes flexibility and ease of use, allowing developers to quickly integrate new planning factors into autoware.

## Usage

### Including the Header

To use the `PlanningFactorInterface`, include the header file in your code:

```cpp
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
```

### Creating an Instance

Instantiate the `PlanningFactorInterface` by providing a node and a name for the factor module:

```cpp

class PlannerInterface
{
public:
  virtual ~PlannerInterface() = default;
  PlannerInterface(
    rclcpp::Node & node, const LongitudinalInfo & longitudinal_info,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
    const EgoNearestParam & ego_nearest_param, const std::shared_ptr<DebugData> debug_data_ptr)
  : planning_factor_interface_{std::make_unique<
      autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "obstacle_cruise_planner")},
```

code example from src/universe/autoware_universe/planning/autoware_obstacle_cruise_planner/include/autoware/obstacle_cruise_planner/planner_interface.hpp

### Adding Planning Factors

The `add` method can be used to add planning factors. Here's an example from src/universe/autoware_universe/planning/autoware_obstacle_cruise_planner/src/pid_based_planner/pid_based_planner.cpp.

```cpp
planning_factor_interface_->add(
        stop_traj_points, planner_data.ego_pose, stop_traj_points.at(wall_idx).pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::NONE,
        autoware_internal_planning_msgs::msg::SafetyFactorArray{});
```

### Publishing Factors

After adding planning factors, you can publish them by calling the `publish` method:

```cpp
// Publish the added factors
planning_factor_interface_->publish();
```
