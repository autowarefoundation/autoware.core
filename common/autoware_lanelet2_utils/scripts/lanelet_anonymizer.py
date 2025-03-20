#!/usr/bin/env python3

# Copyright 2023 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import xml.etree.ElementTree as ET

origin_x = 100.0
origin_y = 100.0


def update_osm_latlon(osm_file, output_file, origin_id):
    tree = ET.parse(osm_file)
    root = tree.getroot()

    old_origin_local_xy = None

    for node in root.findall("node"):
        local_x_tag = node.find(".//tag[@k='local_x']")
        local_y_tag = node.find(".//tag[@k='local_y']")

        if node.attrib["id"] == str(origin_id):
            old_origin_local_xy = (float(local_x_tag.attrib["v"]), float(local_y_tag.attrib["v"]))
            local_x_tag.set("v", str(origin_x))
            local_y_tag.set("v", str(origin_y))
            break

    if old_origin_local_xy is None:
        print(f"could not find point of id {origin_id}")
        return

    (old_origin_x, old_origin_y) = old_origin_local_xy
    for node in root.findall("node"):
        # make origin value exactly (0.0, 0.0)
        if node.attrib["id"] == str(origin_id):
            continue

        local_x_tag = node.find(".//tag[@k='local_x']")
        local_y_tag = node.find(".//tag[@k='local_y']")

        local_x = float(local_x_tag.attrib["v"])
        local_y = float(local_y_tag.attrib["v"])
        adj_local_x = local_x - old_origin_x
        adj_local_y = local_y - old_origin_y

        local_x_tag.set("v", str(origin_x + adj_local_x))
        local_y_tag.set("v", str(origin_y + adj_local_y))

    tree.write(output_file, encoding="UTF-8", xml_declaration=True)
    print(f"Updated OSM file created: {output_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Update OSM file with new origin and adjusted coordinates."
    )
    parser.add_argument("input_osm", help="Path to the input OSM file")
    parser.add_argument("output_osm", help="Path to the output OSM file")
    parser.add_argument("origin_id", help="id of the point to reset as origin", type=int)
    args = parser.parse_args()

    update_osm_latlon(args.input_osm, args.output_osm, args.origin_id)
