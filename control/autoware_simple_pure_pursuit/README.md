# Simple Pure Pursuit

The `simple_pure_pursuit` node receives a reference trajectory from `motion_velocity_smoother` and calculates the control command using the pure pursuit algorithm.

## Flowchart

![Flowchart](https://www.plantuml.com/plantuml/png/LOuxSWCn28PxJa5fNy5Rn4NkiKCaB3D1Q4T2XMyVeZZEH8q6_iV7TJXrdrN1nPMnsUvIkSFQ0roSFlcTd3QG6vvaO8u1ErD-l9tHxsnuUl0u0-jWG1pU3c3BSWCelSq3KvYTzzJCUzFuQoNBOVqk32tTEMDffF_xCDxbc1yguxvQyPdSbhGuY3-aS2RIj6kp8Zwp6EalS7kfmvcxMDd9Yl86aSLr8i0Bz0pziM21hk6TLRy0)

## Input topics

| Name                 | Type                                      | Description          |
| :------------------- | :---------------------------------------- | :------------------- |
| `~/input/odometry`   | `nav_msgs::msg::Odometry`                 | ego odometry         |
| `~/input/trajectory` | `autoware_planning_msgs::msg::Trajectory` | reference trajectory |

## Output topics

| Name                       | Type                                  | Description     | QoS Durability |
| :------------------------- | :------------------------------------ | :-------------- | :------------- |
| `~/output/control_command` | `autoware_control_msgs::msg::Control` | control command | `volatile`     |

## Parameters

{{ json_to_markdown("control/autoware_simple_pure_pursuit/schema/simple_pure_pursuit.schema.json") }}
