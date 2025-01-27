# Path Generator

The `path_generator` node receives a route from `mission_planner` and converts the center line into a path.
If the route has waypoints set, it generates a path passing through them.

This package is a simple alternative of `behavior_path_generator`.

## Path generation

When input data is ready, it first searches for the lanelet closest to the vehicle.
If found, it gets the lanelets within a distance of `backward_path_length` behind and `forward_path_length` in front.
Their center lines are concatenated to generate a path.

If waypoints exist in the route, it replaces the overlapped segment of the center line with them.
The overlap interval is determined as shown in the following figure.

![waypoint_group_overlap_interval_determination](./media/waypoint_group_overlap_interval_determination.drawio.svg)

## Flowchart

![Flowchart](https://www.plantuml.com/plantuml/svg/RL91RiCW4BppYZqwNt0EZTHh_u0sOHAZKa2mJkhV5qDmN2cMB2spEyCCSCh2IUOVfyIA0wNPgmefDGf_GniMFgenGtHqx3tI4x9N6cok2vt0PZcGJF0qBCW71PT1Wmy7HPGbH0Llx5MfMmfpf5L9HvQfT1joGr5cGKU9nXlaquMCB5_iuI373TUk8La_h4pMww8XsFKVlCHWlUYgdbjdTSn-eTwayYFTsL5DdEcCecMeYt_yIqIqgSU4kmrpxZRUk2YJ-1Nir1fUZw5MZyawq70DYS2NYIvOtP2p7bJjjIsRxN17SXmZ39rt0MgwTpORg_jq2xq4dkFRuFwc-ZUoCTikcIJGUYqzMK_nbj_PbRfdxdcrgWUWQjc2g5UbnZYVdStJawxg6pgLWV9K_m00)

```plantuml
@startuml
title run
start

:take_data;
:set_planner_data;
if (is_data_ready) then (yes)
else (no)
  stop
endif

group plan_path
  group generate_path
    :getClosestLanelet;
    :get_lanelets_within_route;
    :get_waypoint_groups;
    if (any waypoint interval starts behind lanelets?) then (yes)
      :get_previous_lanelet_within_route;
      :extend lanelets;
    endif
    while (for each center line point)
      if (overlapped by waypoint group?) then (yes)
        if (previously overlapped?) then
        else (no)
          :add waypoints to path;
        endif
      else (no)
        :add point to path;
      endif
    endwhile
  end group
end group

:publish path;

stop
@enduml
```

## Input topics

| Name                 | Type                                        | Description                      |
| :------------------- | :------------------------------------------ | :------------------------------- |
| `~/input/odometry`   | `nav_msgs::msg::Odometry`                   | ego pose                         |
| `~/input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin`     | vector map information           |
| `~/input/route`      | `autoware_planning_msgs::msg::LaneletRoute` | current route from start to goal |

## Output topics

| Name            | Type                                                   | Description    | QoS Durability |
| :-------------- | :----------------------------------------------------- | :------------- | :------------- |
| `~/output/path` | `autoware_internal_planning_msgs::msg::PathWithLaneId` | generated path | `volatile`     |

## Parameters

{{ json_to_markdown("planning/autoware_path_generator/schema/path_generator.schema.json") }}
