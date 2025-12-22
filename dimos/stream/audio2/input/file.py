#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

"""File input source for audio pipeline."""

from pathlib import Path

import gi
from pydantic import Field, validator

gi.require_version("Gst", "1.0")
from gi.repository import Gst

from dimos.stream.audio2.base import GStreamerSourceBase
from dimos.stream.audio2.gstreamer import GStreamerNodeConfig
from dimos.stream.audio2.types import AudioSource
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.input.file")


class FileInputConfig(GStreamerNodeConfig):
    """Configuration for file input."""

    file_path: str = Field(description="Path to audio file")
    loop: bool = Field(default=False, description="Whether to loop the file")
    realtime: bool = Field(
        default=True, description="Emit audio at real-time speed instead of as fast as possible"
    )

    @validator("file_path")
    def validate_file_exists(cls, v):
        """Validate that the file exists."""
        if not Path(v).exists():
            raise ValueError(f"Audio file not found: {v}")
        return v


class FileInputNode(GStreamerSourceBase):
    """GStreamer-based file input that emits AudioEvents."""

    def __init__(self, config: FileInputConfig):
        super().__init__(config)
        self.config = config  # Type hint for better IDE support

    def _get_pipeline_string(self) -> str:
        """Get the file source pipeline string."""
        return f"filesrc location={self.config.file_path} ! decodebin"

    def _get_source_name(self) -> str:
        """Get a descriptive name including the file."""
        return f"FileInput[{Path(self.config.file_path).name}]"

    def _configure_appsink(self):
        """Configure appsink with realtime option."""
        self._appsink.set_property("emit-signals", True)
        self._appsink.set_property("sync", self.config.realtime)  # Sync to clock if realtime

        # Apply any custom properties
        if hasattr(self.config, "properties") and self.config.properties:
            for prop, value in self.config.properties.items():
                if prop in ["emit-signals", "sync"]:  # Skip already set properties
                    continue
                try:
                    self._appsink.set_property(prop, value)
                    logger.debug(f"Set appsink property {prop}={value}")
                except Exception as e:
                    logger.warning(f"Failed to set appsink property {prop}: {e}")

    def _handle_eos(self):
        """Handle end of stream with looping support."""
        if self.config.loop:
            # Seek back to beginning
            logger.info(f"{self._get_source_name()}: Looping")
            self._pipeline.seek_simple(
                Gst.Format.TIME, Gst.SeekFlags.FLUSH | Gst.SeekFlags.KEY_UNIT, 0
            )
        else:
            # Use parent implementation for completion
            super()._handle_eos()


def file_input(file_path: str, **kwargs) -> AudioSource:
    """Create a file input source.

    Args:
        file_path: Path to audio file
        **kwargs: Additional arguments passed to FileInputConfig:
            - loop: Whether to loop the file (default: False)
            - realtime: Emit at real-time speed (default: True)
            - output: Output audio specification (default: Vorbis compressed)
            - properties: GStreamer element properties

    Returns:
        AudioSource function that creates the observable
    """
    config = FileInputConfig(file_path=file_path, **kwargs)

    def create_source():
        node = FileInputNode(config)
        return node.create_observable()

    return create_source
