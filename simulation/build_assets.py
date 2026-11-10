#!/usr/bin/env python3
"""Generate the isolated x500 LiDAR model and obstacle world."""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

WORLD = "obstacle_course"
MODEL = "x500_lidar"
VEHICLE = f"{MODEL}_0"
SCAN_TOPIC = "/uav/scan"
OBSTACLES = [(2.8, 0, .45), (2.8, 2, .4), (-2, -1.8, .5), (-2, 2.5, .45)]


def child(parent, tag, text=None, **attributes):
    item = ET.SubElement(parent, tag, attributes)
    if text is not None:
        item.text = str(text)
    return item


def fields(element, **values):
    for key, value in values.items():
        child(element, key, value)


def save(element, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(element)
    ET.ElementTree(element).write(path, encoding="utf-8", xml_declaration=True)


def material(visual, color):
    fields(child(visual, "material"), ambient=color, diffuse=color)


def cylinder(parent, name, x, y, radius, height, color, *, solid=True):
    model = child(parent, "model", name=name)
    fields(model, static="true", pose=f"{x} {y} {height / 2} 0 0 0")
    link = child(model, "link", name="link")
    for kind in ("collision", "visual") if solid else ("visual",):
        item = child(link, kind, name=kind)
        shape = child(child(item, "geometry"), "cylinder")
        fields(shape, radius=radius, length=height)
        if kind == "visual":
            material(item, color)


def lidar_model(output):
    sdf = ET.Element("sdf", version="1.9")
    model = child(sdf, "model", name=MODEL)
    child(child(model, "include", merge="true"), "uri", "model://x500")
    link = child(model, "link", name="lidar_link")
    child(link, "pose", "0 0 0.4 0 0 0")
    inertial = child(link, "inertial")
    child(inertial, "mass", 0.02)
    fields(child(inertial, "inertia"), ixx=.00002, iyy=.00002, izz=.00002, ixy=0, ixz=0, iyz=0)
    housing = child(link, "visual", name="lidar_housing")
    fields(child(child(housing, "geometry"), "cylinder"), radius=.045, length=.06)
    material(housing, "0.1 0.7 0.85 1")

    sensor = child(link, "sensor", name="scan", type="gpu_lidar")
    fields(sensor, topic=SCAN_TOPIC, gz_frame_id="lidar_link", always_on="true",
           update_rate=10, visualize="false")
    lidar = child(sensor, "lidar")
    horizontal = child(child(lidar, "scan"), "horizontal")
    fields(horizontal, samples=720, resolution=1, min_angle=-math.pi, max_angle=math.pi)
    fields(child(lidar, "range"), min=.1, max=6.0, resolution=.01)
    joint = child(model, "joint", name="lidar_joint", type="fixed")
    fields(joint, parent="base_link", child="lidar_link")
    save(sdf, output / f"models/{MODEL}/model.sdf")

    metadata = ET.Element("model")
    fields(metadata, name=MODEL, version="1.0")
    child(metadata, "sdf", "model.sdf", version="1.9")
    child(metadata, "description", "Simulated x500 with a centered horizontal LiDAR.")
    save(metadata, output / f"models/{MODEL}/model.config")


def obstacle_world(output):
    sdf = ET.Element("sdf", version="1.9")
    world = child(sdf, "world", name=WORLD)
    physics = child(world, "physics", name="development", type="ode")
    fields(physics, max_step_size=.004, real_time_factor=1)
    fields(world, gravity="0 0 -9.8", magnetic_field="6e-06 2.3e-05 -4.2e-05")
    systems = {
        "physics": "Physics", "user-commands": "UserCommands", "scene-broadcaster": "SceneBroadcaster",
        "imu": "Imu", "air-pressure": "AirPressure", "magnetometer": "Magnetometer",
        "navsat": "NavSat", "sensors": "Sensors",
    }
    for filename, name in systems.items():
        plugin = child(world, "plugin", filename=f"gz-sim-{filename}-system", name=f"gz::sim::systems::{name}")
        if name == "Sensors":
            child(plugin, "render_engine", "ogre")

    fields(child(world, "scene"), ambient=".65 .65 .65 1", background=".82 .88 .94 1", grid="true")
    sun = child(world, "light", name="sun", type="directional")
    fields(sun, direction="-.3 -.4 -1", diffuse=".9 .9 .9 1", cast_shadows="true")
    ground = child(world, "model", name="ground_plane")
    child(ground, "static", "true")
    ground_link = child(ground, "link", name="link")
    for kind in ("collision", "visual"):
        item = child(ground_link, kind, name=kind)
        fields(child(child(item, "geometry"), "plane"), normal="0 0 1", size="30 30")
        if kind == "visual":
            material(item, ".72 .76 .8 1")

    for i, (x, y, radius) in enumerate(OBSTACLES, 1):
        cylinder(world, f"obstacle_{i}", x, y, radius, 1.8, ".85 .25 .12 1")
    cylinder(world, "goal_marker", 5, 0, .35, .03, ".12 .75 .3 1", solid=False)
    fields(child(world, "include"), uri=f"model://{MODEL}", name=VEHICLE, pose="0 0 .2 0 0 0")
    # PX4's stock simulation origin for the GPS model.
    fields(child(world, "spherical_coordinates"), surface_model="EARTH_WGS84",
           world_frame_orientation="ENU", latitude_deg=47.397971057728974,
           longitude_deg=8.546163739800146, elevation=0)
    save(sdf, output / f"worlds/{WORLD}.sdf")


def build_assets(output):
    lidar_model(output)
    obstacle_world(output)
    reference = {
        "world": WORLD, "spawn_xy_enu": [0, 0], "goal_enu": [5, 0], "obstacles_xy_radius": OBSTACLES,
        "sensor": {
            "samples": 720, "coverage_degrees": 360, "range_min_m": .1, "range_max_m": 6,
            "update_rate_hz": 10, "offset_body_flu_m": [0, 0, .4],
        },
        "scope": "Scene reference geometry is separate from policy observations.",
    }
    (output / "scene_reference.json").write_text(json.dumps(reference, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    build_assets(parser.parse_args().output)
