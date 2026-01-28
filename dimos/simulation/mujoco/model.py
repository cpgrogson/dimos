#!/usr/bin/env python3

# Copyright 2025-2026 Dimensional Inc.
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


import json
from pathlib import Path
import xml.etree.ElementTree as ET

from etils import epath
import mujoco
from mujoco_playground._src import mjx_env
import numpy as np

from dimos.core.global_config import GlobalConfig
from dimos.mapping.occupancy.extrude_occupancy import generate_mujoco_scene
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.simulation.mujoco.input_controller import InputController
from dimos.simulation.mujoco.policy import (
    G1OnnxController,
    Go1OnnxController,
    MjlabVelocityOnnxController,
    OnnxController,
)
from dimos.utils.data import get_data


def _get_data_dir() -> epath.Path:
    return epath.Path(str(get_data("mujoco_sim")))


def _bundle_dir(profile: str) -> epath.Path:
    return _get_data_dir() / profile


def _bundle_model_path(profile: str) -> epath.Path:
    # New bundle layout: data/mujoco_sim/<profile>/model.xml
    return _bundle_dir(profile) / "model.xml"


def _bundle_policy_path(profile: str) -> epath.Path:
    # New bundle layout: data/mujoco_sim/<profile>/policy.onnx
    return _bundle_dir(profile) / "policy.onnx"


def _legacy_profile_xml_path(profile: str) -> epath.Path:
    # Legacy layout (older integration): data/mujoco_sim/<profile>.xml
    return _get_data_dir() / f"{profile}.xml"


def _legacy_profile_policy_path(profile: str) -> epath.Path:
    # Legacy layout (older integration): data/mujoco_sim/<profile>_policy.onnx
    return _get_data_dir() / f"{profile}_policy.onnx"


def load_bundle_json(profile: str) -> dict[str, object] | None:
    """Load optional bundle.json for a MuJoCo profile.

    The MuJoCo subprocess uses this to resolve profile-specific camera names.
    """
    cfg_path = _bundle_dir(profile) / "bundle.json"
    if not cfg_path.exists():
        return None
    with cfg_path.open("r", encoding="utf-8") as f:
        return json.load(f)


def get_assets(*, profile: str | None = None) -> dict[str, bytes]:
    data_dir = _get_data_dir()
    # Assets used from https://sketchfab.com/3d-models/mersus-office-8714be387bcd406898b2615f7dae3a47
    # Created by Ryan Cassidy and Coleman Costello
    assets: dict[str, bytes] = {}

    # Add all top-level XMLs. Keys must match include paths like "unitree_go1.xml".
    mjx_env.update_assets(assets, data_dir, "*.xml")

    # Scene assets are referenced with explicit paths (e.g. "scene_office1/office_split/*.obj")
    # after we rewrite the scene XML in get_model_xml(). Load them with path-prefixed keys.
    fs_root = Path(str(data_dir))
    for p in (fs_root / "scene_office1/textures").glob("*.png"):
        assets[f"scene_office1/textures/{p.name}"] = p.read_bytes()
    for p in (fs_root / "scene_office1/office_split").glob("*.obj"):
        assets[f"scene_office1/office_split/{p.name}"] = p.read_bytes()

    if profile:
        # Bundle-scoped assets: keep the sim fully self-contained when a profile is used.
        #
        # IMPORTANT: MuJoCo's asset VFS rejects duplicate *basenames* (even if we provide
        # different directory prefixes). Some profiles (e.g. Falcon) vendor Pinocchio-only
        # URDF/meshes under "<profile>/falcon_assets/..." that must NOT be loaded into MuJoCo,
        # otherwise we can collide on names like "pelvis.stl".
        allowed_suffixes = {".xml", ".png", ".jpg", ".jpeg", ".stl", ".obj", ".mtl"}
        bundle_root = fs_root / profile
        basename_owner: dict[str, str] = {}

        for fp in bundle_root.rglob("*"):
            if not fp.is_file():
                continue
            if fp.suffix.lower() not in allowed_suffixes:
                continue
            # Skip Pinocchio-only Falcon assets to avoid MuJoCo basename collisions.
            if "falcon_assets" in fp.parts:
                continue

            rel = fp.relative_to(fs_root).as_posix()
            base = fp.name.lower()
            if base in basename_owner:
                raise ValueError(
                    f"Repeated file name in MuJoCo assets dict: {base} "
                    f"(from {basename_owner[base]} and {rel}). "
                    "Remove/rename one of the files, or adjust get_assets() filtering."
                )
            basename_owner[base] = rel
            assets[rel] = fp.read_bytes()
    else:
        mjx_env.update_assets(assets, mjx_env.MENAGERIE_PATH / "unitree_go1" / "assets")
        mjx_env.update_assets(assets, mjx_env.MENAGERIE_PATH / "unitree_g1" / "assets")
    return assets


def load_model(
    input_device: InputController, robot: str, scene_xml: str, *, profile: str | None = None
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    mujoco.set_mjcb_control(None)

    include_name = profile or robot
    xml_string = get_model_xml(robot=robot, scene_xml=scene_xml, profile=profile)
    model = mujoco.MjModel.from_xml_string(xml_string, assets=get_assets(profile=profile))
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, 0)

    match robot:
        case "unitree_g1":
            sim_dt = 0.002
        case _:
            sim_dt = 0.005

    ctrl_dt = 0.02
    n_substeps = round(ctrl_dt / sim_dt)
    model.opt.timestep = sim_dt

    # Resolve policy path. Prefer new bundle layout; fall back to legacy flat files.
    if profile and _bundle_policy_path(profile).exists():
        policy_path = _bundle_policy_path(profile).as_posix()
    else:
        policy_path = (_legacy_profile_policy_path(include_name)).as_posix()

    # Default joint angles used for legacy controllers and as a shape reference.
    # Some MJLab-exported bundles use "init_state" and may not provide "home".
    if model.nkey > 0:
        default_qpos = np.array(model.key_qpos[0, 7:], dtype=np.float32)
    else:
        default_qpos = np.array(data.qpos[7:], dtype=np.float32)
    params = {
        "policy_path": policy_path,
        "default_angles": default_qpos,
        "n_substeps": n_substeps,
        "action_scale": 0.5,
        "input_controller": input_device,
        "ctrl_dt": ctrl_dt,
    }

    match robot:
        case "unitree_go1":
            policy: OnnxController = Go1OnnxController(**params)
        case "unitree_g1":
            # Select controller by profile/bundle name when provided.
            if include_name == "unitree_g1_mjlab":
                policy = MjlabVelocityOnnxController(**params)
            else:
                policy = G1OnnxController(**params, drift_compensation=[-0.18, 0.0, -0.09])
        case _:
            raise ValueError(f"Unknown robot policy: {robot}")

    mujoco.set_mjcb_control(policy.get_control)

    return model, data


def get_model_xml(*, robot: str, scene_xml: str, profile: str | None = None) -> str:
    root = ET.fromstring(scene_xml)
    root.set("model", f"{(profile or robot)}_scene")

    # The office scene config uses a global compiler meshdir/texturedir.
    # When we include a robot MJCF (e.g. Unitree GO1) that references meshes like "trunk.stl"
    # without a directory prefix, MuJoCo incorrectly resolves them relative to the scene meshdir
    # and fails to load. Fix by rewriting scene asset file paths to be explicit and clearing
    # meshdir/texturedir so they can't leak into the included robot model.
    compiler = root.find("compiler")
    if compiler is not None:
        meshdir = compiler.get("meshdir")
        texturedir = compiler.get("texturedir")

        if meshdir:
            for mesh in root.findall("./asset/mesh"):
                f = mesh.get("file")
                if f and "/" not in f and "\\" not in f:
                    mesh.set("file", f"{meshdir}/{f}")
            compiler.attrib.pop("meshdir", None)

        if texturedir:
            for tex in root.findall("./asset/texture"):
                f = tex.get("file")
                if f and "/" not in f and "\\" not in f:
                    tex.set("file", f"{texturedir}/{f}")
            compiler.attrib.pop("texturedir", None)

    # Resolve robot include file path.
    # Prefer new bundle layout: <profile>/model.xml
    if profile and _bundle_model_path(profile).exists():
        include_file = f"{profile}/model.xml"
    else:
        # Legacy behavior: include <name>.xml from data/mujoco_sim root.
        include_file = f"{profile or robot}.xml"
    root.insert(0, ET.Element("include", file=include_file))

    # Ensure visual/map element exists with znear and zfar
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    map_elem = visual.find("map")
    if map_elem is None:
        map_elem = ET.SubElement(visual, "map")
    map_elem.set("znear", "0.01")
    map_elem.set("zfar", "10000")

    return ET.tostring(root, encoding="unicode")


def load_model_sdk2(
    robot: str, scene_xml: str, *, profile: str | None = None
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Load MuJoCo model without ONNX policy for SDK2 bridge mode.

    In SDK2 mode, motor control is handled by the SDK2BridgeController
    which receives commands via DDS and applies PD control directly.
    No policy callback is registered - mj_step uses data.ctrl directly.

    Args:
        robot: Robot name (e.g., "unitree_go2", "unitree_g1")
        scene_xml: Scene XML string
        profile: Optional MuJoCo profile bundle name

    Returns:
        Tuple of (MjModel, MjData)
    """
    mujoco.set_mjcb_control(None)  # Clear any existing callback

    xml_string = get_model_xml(robot=robot, scene_xml=scene_xml, profile=profile)
    model = mujoco.MjModel.from_xml_string(xml_string, assets=get_assets(profile=profile))
    data = mujoco.MjData(model)

    # Apply keyframe if available
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
        print(f"[load_model_sdk2] Applied keyframe 0, qpos[7:13] = {data.qpos[7:13].tolist()}")
    else:
        print(f"[load_model_sdk2] WARNING: No keyframes in model (nkey={model.nkey})")

    # Set timestep based on robot type (matching load_model behavior)
    match robot:
        case "unitree_g1":
            sim_dt = 0.002
        case _:
            sim_dt = 0.005
    model.opt.timestep = sim_dt

    return model, data


def load_scene_xml(config: GlobalConfig) -> str:
    if config.mujoco_room_from_occupancy:
        path = Path(config.mujoco_room_from_occupancy)
        return generate_mujoco_scene(OccupancyGrid.from_path(path))

    mujoco_room = config.mujoco_room or "office1"
    xml_file = (_get_data_dir() / f"scene_{mujoco_room}.xml").as_posix()
    with open(xml_file) as f:
        return f.read()
