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

from __future__ import annotations

from os.path import expanduser
from pathlib import Path
import time

from ..support import prompt_tools as p
from ..support.bundled_data import PROJECT_TOML
from ..support.constants import DISCORD_URL, PLACEHOLDERS
from ..support.dimos_banner import RenderLogo
from ..support.direnv import setup_direnv
from ..support.dotenv import setup_dotenv
from ..support.get_system_analysis import get_system_analysis
from ..support.installer_status import installer_status
from ..support.misc import (
    get_project_directory,
    init_repo_with_gitignore,
    replace_strings_in_directory,
)
from ..support.setup_docker_env import setup_docker_env
from ..support.setup_nix import ensure_flakes_enabled, nix_install, setup_nix_flake
from ..support.shell_tooling import command_exists, run_command

home = Path(expanduser("~"))
dimos_cache = home / ".cache" / "dimos"


def phase0(cli_features: list[str] | None = None) -> tuple[dict[str, object], list[str]]:
    #
    # provide animation while running system analysis
    #
    fps = 14
    logo = RenderLogo(
        glitchyness=0.45,  # relative quantity of visual artifacting
        stickyness=fps * 0.75,  # how many frames to keep an artifact
        fps=fps,  # at 30fps it flickers a lot in the MacOS stock terminal. Ironically its fine at 30fps in the VS Code terminal
        color_wave_amplitude=10,  # bigger = wider range of colors
        wave_speed=0.01,  # bigger = faster
        wave_freq=0.01,  # smaller = longer streaks of color
        scrollable=True,
    )

    #
    # system analysis
    #
    logo.log("- checking system")
    system_analysis = get_system_analysis()
    if not dimos_cache.exists():
        logo.log("- creating dimos cache")
        dimos_cache.mkdir(parents=True, exist_ok=True)
        timeout = 0.5  # wait long enough so users can read what is happening and see logo
    else:
        timeout = 0.2  # don't wait on second run

    # visually we want cuda to be listed last and os to be first
    cuda = system_analysis["cuda"]
    del system_analysis["cuda"]
    ordered_analysis = {
        "os": system_analysis["os"],
        **system_analysis,
        "cuda": cuda,
    }
    ordered_analysis["cuda"] = cuda
    for key, result in ordered_analysis.items():
        name = result.get("name") or key
        exists = result.get("exists", False)
        version = result.get("version", "") or ""
        note = result.get("note", "") or ""
        cross = "\u2718"
        check = "\u2714"
        if not exists:
            logo.log(f"- {p.red(cross)} {name} {note}".strip())
        else:
            logo.log(f"- {p.cyan(check)} {name}: {version} {note}".strip())
        time.sleep(timeout)
    logo.stop()
    p.clear_screen()

    #
    # question 1: in a project directory?
    #
    p.header("First Phase: Feature Selection")
    # ask user project question up front
    project_dir = get_project_directory()
    if installer_status.get("template_repo"):
        project_name = project_dir.name
        # fill out the directory
        replace_strings_in_directory(project_dir, PLACEHOLDERS, project_name)

    #
    # question 2: which dimos features?
    #
    selected_features = []
    if cli_features is None:
        optional = PROJECT_TOML["project"].get("optional-dependencies", {})
        features = [f for f in optional.keys() if f not in ["cpu"]]
        selected_features = p.pick_many(
            "Which features do you want? (Pick any number of features)",
            options=["basics", *features],
        )
        # basics is just a dummy entry to make it more user friendly
        selected_features = [each for each in selected_features if each != "basics"]
        if "sim" in selected_features and "cuda" not in selected_features:
            selected_features.append("cpu")

    #
    # question 3: setup .env and .envrc?
    #
    env_path = f"{project_dir}/.env"
    envrc_path = f"{project_dir}/.envrc"
    has_dotenv = setup_dotenv(project_dir, env_path)
    if not has_dotenv:
        return

    setup_direnv(envrc_path)

    #
    # question 4: what install method?
    #
    options = {
        "system": "Typical system install",
        "docker": "Docker container setup",
        "nix": "Nix flake",
        # "nix_venv": "Fast venv setup (nix shell)",
    }
    os_info = system_analysis.get("os", {})
    native_install_supported = (
        os_info.get("name") == "debian_based" or os_info.get("name") == "macos"
    )
    if not native_install_supported:
        del options["docker"]
        print(
            "NOTE: if you want a native install (on non-debian/macos/NixOS) please see the docs on the manual install method"
        )
        # TODO: after manual install docs are created link them here

    while True:
        choice = p.pick_one(
            "Choose install method",
            options=options,
        )
        if choice == "system":
            # continue as normal
            break
        if choice == "docker":
            if not system_analysis.get("docker", {}).get("exists"):
                p.error("Docker is not installed or not detected.")
                print("Download Docker: https://www.docker.com/products/docker-desktop/")
                # print("Alternatively you can likely run this in a different terminal to get docker installed:")
                # print(
                #     "    # Install Docker\n"+
                #     "    curl -fsSL https://get.docker.com -o get-docker.sh\n"+
                #     "    sudo sh get-docker.sh\n"+
                #     "    \n"+
                #     "    # Post-install steps\n"+
                #     "    sudo groupadd docker\n"+
                #     "    sudo usermod -aG docker $USER\n"+
                #     "    newgrp docker\n"+
                #     ""
                # )
                next_step = p.pick_one(
                    "Docker is required for this option.",
                    options={"back": "Choose a different install method", "exit": "Exit installer"},
                )
                if next_step == "exit":
                    raise SystemExit(1)
                continue
            project_dir = get_project_directory()
            paths = setup_docker_env(project_dir, selected_features)
            p.sub_header("Docker assets created/updated:")
            for key, path in paths.items():
                print(f" - {key}: {path}")
            print(
                f"Use {p.highlight('run/docker_build')} to build the image, and {p.highlight('run/docker_exec')} to start a shell in the container."
            )
            # build command
            while True:
                try:
                    response = input(
                        f"Please type {p.highlight('run/docker_build')} to build the image right now\npress CTRL+C to if you want to run that command yourself (e.g. later)"
                    )
                except KeyboardInterrupt:
                    print(
                        "exiting installer, NOTE: docker is ready whenever you're ready to build+run it"
                    )
                    exit(0)
                if response.strip() == "run/docker_build":
                    run_command([str(paths["build_script"])], check=False)
                    break
            # run command
            while True:
                try:
                    response = input(
                        f"Please type {p.highlight('run/docker_exec')} to run the image right now\npress CTRL+C to if you want to run that command later"
                    )
                except KeyboardInterrupt:
                    print(
                        "exiting installer, NOTE: docker is ready whenever you're ready to run it"
                    )
                    exit(0)
                if response.strip() == "run/docker_exec":
                    run_command([str(paths["exec_script"])], check=False)
                    break
            # run completed successfully (setup mounted venv and pip-installed dimos)
            if Path(".dimos.ignore").exists():
                p.sub_header(
                    "Docker setup complete! NOTE: if you try activating the venv outside of docker you're going to have a bad time (so use docker)"
                )
                print("Note: feel free to edit your Dockerfile as you see fit")
                raise SystemExit(0)
            else:
                p.warning(
                    f"It looks like the docker run wasn't able to completely setup dimos\nNote, every time you run {p.highlight('run/docker_exec')} it will attempt to install dimos.\nSo, if you fix the issue, try running {p.highlight('run/docker_exec')} again\n\nIn the meantime, please don't hesitate to reach out to us on discord:\n    {DISCORD_URL}"
                )
                raise SystemExit(3)
        if choice == "nix":
            project_dir = get_project_directory()
            example_path = setup_nix_flake(project_dir)
            if not example_path:
                continue

            feat_str = "[" + (",".join(selected_features)) + "]" if selected_features else ""
            if not command_exists("git"):
                print("You need to install git for the flake.nix to work")
                print("Should I install it for you? (y/n)")
                nix_install(["git"])  # this will install nix if needed
            
            git_commit_instruction = "\n- git commit the {p.highlight('flake.nix')}"
            if not Path(project_dir / ".git").exists():
                if p.ask_yes_no(
                    "Your project doesn't seem to have a (direct) git repo.\nFlakes require a git repo.\nShould I initialize a new git repo for this flake?"
                ):
                    init_repo_with_gitignore(project_dir)
                    run_command(["git", "add", "flake.nix"], print_command=True)
                    run_command(["git", "commit", "-m", "add flake.nix"], print_command=True)
                    git_commit_instruction = ""
                    print()
                    print()
                else:
                    # if we are to automate this, we must account for there being already-staged changes
                    print(
                        "Okay, but make sure to commit the flake.nix changes otherwise you won't be able to run `nix develop`"
                    )

            ensure_flakes_enabled()
            install_command = f"pip install dimos{feat_str}"
            print(
                f"Once you are ready:{git_commit_instruction}\n- run {p.highlight('nix develop')}\n- then run {p.highlight(install_command)}"
            )
            dev_command = (
                f"pip install 'dimos{feat_str} @ git+ssh://git@github.com/dimensionalOS/dimos.git'"
            )
            # FIXME: change before release
            p.warning(f"because you're on dev run: {p.highlight(dev_command)}")

            raise SystemExit(0)

    return system_analysis, selected_features


if __name__ == "__main__":
    print(phase0())
