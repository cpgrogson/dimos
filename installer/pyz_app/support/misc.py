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

from pathlib import Path
import re
import subprocess
import threading
from typing import TYPE_CHECKING

from . import prompt_tools as p
from .bundled_data import DEP_2_HUMAN_NAME, PIP_DEP_DATABASE, PROJECT_TOML
from .constants import (
    DEFAULT_GITIGNORE_CONTENT,
    DEPENDENCY_APT_PACKAGES_SET_MINIMAL,
    DEPENDENCY_BREW_SET_MINIMAL,
    DEPENDENCY_HUMAN_NAMES_SET,
    DEPENDENCY_NIX_PACKAGES_SET_MINIMAL,
)
from .installer_status import installer_status
from .shell_tooling import command_exists, run_command

if TYPE_CHECKING:
    from collections.abc import Iterable

_project_directory: Path | None = None


def get_system_deps(feature: str | None):
    apt_deps: set[str] = set()
    nix_deps: set[str] = set()
    brew_deps: set[str] = set()

    if feature is None:
        pip_deps = list(PROJECT_TOML["project"]["dependencies"])
    elif isinstance(feature, list | tuple | set):
        pip_deps = []
        for feat in feature:
            pip_deps.extend(PROJECT_TOML["project"]["optional-dependencies"].get(feat, []))
    else:
        pip_deps = list(PROJECT_TOML["project"]["optional-dependencies"].get(feature, []))

    pip_deps = [re.sub(r"[<=>,;].+", "", dep) for dep in pip_deps]
    missing: list[str] = []

    for pip_dep in pip_deps:
        pip_dep = pip_dep.lower()

        pip_dep_no_feature = re.sub(r"\[.+", "", pip_dep)
        system_dep_info = PIP_DEP_DATABASE.get(pip_dep) or PIP_DEP_DATABASE.get(pip_dep_no_feature)
        if not system_dep_info:
            missing.append(pip_dep)
            continue

        for key, value in system_dep_info.items():
            if key == "apt_dependencies":
                apt_deps.update(value)
            elif key == "nix_dependencies":
                nix_deps.update(value)
            elif key == "brew_dependencies":
                brew_deps.update(value)
    apt_deps = apt_deps | DEPENDENCY_APT_PACKAGES_SET_MINIMAL
    nix_deps = nix_deps | DEPENDENCY_NIX_PACKAGES_SET_MINIMAL
    brew_deps = brew_deps | DEPENDENCY_BREW_SET_MINIMAL
    combined_deps = set(apt_deps) | set(nix_deps) | set(brew_deps)
    return {
        "apt_deps": sorted(apt_deps),
        "nix_deps": sorted(nix_deps),
        "brew_deps": sorted(brew_deps),
        "pip_deps": sorted(pip_deps),
        "human_names_all": sorted(
            DEPENDENCY_HUMAN_NAMES_SET | {DEP_2_HUMAN_NAME.get(dep, dep) for dep in combined_deps}
        ),
        "human_names_from_apt": sorted(
            DEPENDENCY_HUMAN_NAMES_SET | {DEP_2_HUMAN_NAME.get(dep, dep) for dep in apt_deps}
        ),
        "human_names_from_brew": sorted(
            DEPENDENCY_HUMAN_NAMES_SET | {DEP_2_HUMAN_NAME.get(dep, dep) for dep in brew_deps}
        ),
        "human_names_from_nix": sorted(
            DEPENDENCY_HUMAN_NAMES_SET | {DEP_2_HUMAN_NAME.get(dep, dep) for dep in nix_deps}
        ),
        "missing": missing,
    }


def parse_version(text: str) -> str | None:
    match = re.search(r"\b(\d+(?:\.\d+)+)\b", text)
    return match.group(1) if match else None


def is_version_at_least(found: str, required: str) -> bool:
    found_parts = [int(x) for x in found.split(".")]
    required_parts = [int(x) for x in required.split(".")]
    length = max(len(found_parts), len(required_parts))
    for i in range(length):
        f = found_parts[i] if i < len(found_parts) else 0
        r = required_parts[i] if i < len(required_parts) else 0
        if f > r:
            return True
        if f < r:
            return False
    return True


def detect_python_command() -> str | None:
    if command_exists("python3"):
        return "python3"
    if command_exists("python"):
        return "python"
    return None


def ensure_git_and_lfs() -> None:
    p.boring_log("- checking git")
    if not command_exists("git"):
        raise RuntimeError("- ❌ git is required. Please install git and rerun.")
    p.boring_log("- checking git-lfs")
    git_lfs_res = run_command(["git", "lfs", "version"])  # intentionally not part of dry_run
    if git_lfs_res.code != 0:
        raise RuntimeError("- ❌ git-lfs is required. Please install git-lfs and rerun.")


def ensure_port_audio() -> None:
    # TODO: only run this check if pyaudio is in the pip dependency list. Its in the pyprojct.toml core right now but it might not be forever
    p.boring_log("- checking if portaudio is available")
    port_audio_res = run_command(  # intentionally not part of dry_run
        ["pkg-config", "--modversion", "portaudio-2.0"], print_command=True, capture_output=True
    )
    if port_audio_res.code != 0:
        raise RuntimeError("- ❌ portaudio is required. Please install portaudio and rerun.")


def ensure_python() -> str:
    p.boring_log("- checking python version")
    python_cmd = detect_python_command()
    if not python_cmd:
        raise RuntimeError("- ❌ Python 3.10+ is required but was not found.")
    version_res = run_command(
        [python_cmd, "--version"], capture_output=True
    )  # intentionally not part of dry_run
    version_text = (version_res.stdout or version_res.stderr or "").strip()
    parsed = parse_version(version_text)
    if not parsed or not is_version_at_least(parsed, "3.10.0"):
        raise RuntimeError(f"- ❌ Python 3.10+ required. Detected: {parsed or 'unknown'}")
    return python_cmd


def get_project_directory() -> Path:
    global _project_directory
    if _project_directory is None:
        if installer_status.get("template_repo"):
            _project_directory = Path.cwd()
        else:
            print("Dimos needs to be installed to a project (not just a global install)")
            if p.ask_yes_no("Are you currently in a project directory?"):
                _project_directory = Path.cwd()
            else:
                raise RuntimeError(
                    "- ❌ Please create a project directory and rerun this command from there."
                )
    return _project_directory


def replace_strings_in_directory(root: Path, needles: Iterable[str], replacement: str) -> None:
    root = Path(root)
    needle_list = [n for n in needles if n]
    if not needle_list:
        return

    def _worker() -> None:
        for path in root.rglob("*"):
            if not path.is_file():
                continue
            try:
                text = path.read_text()
            except (UnicodeDecodeError, OSError):
                continue

            new_text = text
            for needle in needle_list:
                if needle in new_text:
                    new_text = new_text.replace(needle, replacement)

            if new_text != text:
                try:
                    path.write_text(new_text)
                except OSError:
                    pass

    threading.Thread(target=_worker, name="ReplaceStringsInDirectory", daemon=True).start()


def add_git_ignore_patterns(
    project_path: str | Path,
    patterns: list[str],
    opts: dict[str, str] | None = None,
) -> dict[str, object]:
    project_path = Path(project_path)
    gitignore_path = project_path.joinpath(".gitignore")
    opts = opts or {}

    if gitignore_path.exists() and not gitignore_path.is_file():
        return {
            "updated": False,
            "added": [],
            "already_present": [],
            "ignore_did_not_exist": True,
        }
    if not gitignore_path.exists():
        return {
            "updated": False,
            "added": [],
            "already_present": list(patterns),
            "ignore_did_not_exist": False,
        }

    original = gitignore_path.read_text()
    has_trailing_newline = original.endswith("\n") or original.endswith("\r\n")
    text = original.replace("\r\n", "\n")

    existing_lines = text.split("\n")
    existing_set = {l.strip() for l in existing_lines if l.strip()}

    cleaned_patterns = [p.strip() for p in patterns if p.strip()]
    added: list[str] = []
    already_present: list[str] = []

    for pattern in cleaned_patterns:
        if pattern in existing_set:
            already_present.append(pattern)
        else:
            added.append(pattern)

    if not added:
        return {
            "updated": False,
            "added": [],
            "already_present": already_present,
            "ignore_did_not_exist": False,
        }

    new_lines: list[str] = []
    needs_newline = not has_trailing_newline and len(text) > 0
    if needs_newline:
        new_lines.append("")

    ends_with_blank_line = len(existing_lines) > 0 and existing_lines[-1].strip() == ""
    if has_trailing_newline and not ends_with_blank_line:
        new_lines.append("")

    comment = opts.get("comment", "").strip()
    if added and comment:
        header = comment if comment.startswith("#") else f"# {comment}"
        new_lines.append(header)

    new_lines.extend(added)

    updated_text = text + "\n".join(new_lines) + "\n"
    gitignore_path.write_text(updated_text)

    return {
        "updated": True,
        "added": added,
        "already_present": already_present,
        "ignore_did_not_exist": False,
    }


def maybe_write(path: Path, content: str) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        if not p.ask_yes_no(f"{path.name} already exists. Overwrite?"):
            return False
    path.write_text(content)
    return True


def init_repo_with_gitignore(repo_dir: str | Path) -> None:
    repo_dir = Path(repo_dir)
    repo_dir.mkdir(parents=True, exist_ok=True)
    p.boring_log("- running git init")
    run_command(["git", "init"], print_command=True)
    git_ignore = Path(repo_dir / ".gitignore")
    if not git_ignore.exists():
        p.boring_log("- git ignore file not found, creating")
        git_ignore.write_text("", encoding="utf-8")
    else:
        p.boring_log("- git ignore file found")

    add_git_ignore_patterns(git_ignore, DEFAULT_GITIGNORE_CONTENT.split("\n"))
    run_command(["git", "add", ".gitignore"], print_command=True)
    run_command(["git", "commit", "-m", "gitignore"], print_command=True)


def in_git_repo() -> bool:
    try:
        subprocess.run(
            ["git", "rev-parse", "--is-inside-work-tree"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=True,
        )
        return True
    except Exception:
        return False
