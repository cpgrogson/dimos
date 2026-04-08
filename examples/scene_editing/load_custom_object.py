#!/usr/bin/env python3
"""Create a custom object from code (no GLB needed)."""

from dimos.robot.sim.scene_client import SceneClient

with SceneClient() as scene:
    scene.add_object(
        "box",
        size=(1, 0.5, 1),
        color=0x8B4513,
        position=(3, 0.25, 2),
        name="crate",
    )
    scene.add_object(
        "sphere",
        size=(0.3,),
        color=0xFF0000,
        position=(2, 2, 2),
        name="ball",
        dynamic=True,
        mass=0.5,
        restitution=0.8,
    )
    print("Custom objects added")
