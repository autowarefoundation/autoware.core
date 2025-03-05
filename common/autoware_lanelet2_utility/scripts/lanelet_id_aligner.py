#!/usr/bin/env python3

# Copyright 2025 TIER IV, Inc.
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


def renumber_osm_ids(input_file):
    tree = ET.parse(input_file)
    root = tree.getroot()

    id_map = {}
    new_id = 1

    # Collect nodes, ways, and relations with new ids
    for element in root.findall("node") + root.findall("way") + root.findall("relation"):
        old_id = element.attrib["id"]
        id_map[old_id] = str(new_id)
        element.set("id", str(new_id))
        new_id += 1

    # Update references in <nd> (inside <way>) and <member> (inside <relation>)
    for way in root.findall("way"):
        for nd in way.findall("nd"):
            old_ref = nd.attrib["ref"]
            if old_ref in id_map:
                nd.set("ref", id_map[old_ref])
            else:
                print(f"reference to ref={old_ref} was invalid")

    for relation in root.findall("relation"):
        for member in relation.findall("member"):
            old_ref = member.attrib["ref"]
            if old_ref in id_map:
                member.set("ref", id_map[old_ref])
            else:
                print(f"reference to ref={old_ref} was invalid")

        for tag in relation.findall("tag"):
            if tag.attrib["k"] != "intersection_area":
                continue

            old_ref = tag.attrib["v"]
            if old_ref in id_map:
                tag.set("k", id_map[old_ref])
            else:
                print(f"reference to ref={old_ref} was invalid")

    tree.write(input_file, encoding="utf-8", xml_declaration=True)
    print(f"Updated OSM file {input_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Update OSM file with new origin and adjusted coordinates."
    )
    parser.add_argument("input_osm", help="Path to the input OSM file")
    args = parser.parse_args()
    renumber_osm_ids(args.input_osm)
