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

"""Test signal generator for audio pipeline testing."""

from enum import Enum
from typing import Optional

import gi
from pydantic import Field, validator

gi.require_version("Gst", "1.0")

from dimos.stream.audio2.base import GStreamerSourceBase
from dimos.stream.audio2.gstreamer import GStreamerNodeConfig
from dimos.stream.audio2.types import AudioSource
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.input.signal")


class WaveformType(Enum):
    """Supported waveform types for test signal generation."""

    SINE = "sine"  # Sine wave
    SQUARE = "square"  # Square wave
    SAW = "saw"  # Sawtooth wave
    TRIANGLE = "triangle"  # Triangle wave
    SILENCE = "silence"  # Silence
    WHITE_NOISE = "white-noise"  # White noise
    PINK_NOISE = "pink-noise"  # Pink noise
    SINE_TABLE = "sine-table"  # Sine wave using pre-calculated table
    TICKS = "ticks"  # Periodic ticks
    GAUSSIAN_NOISE = "gaussian-noise"  # Gaussian white noise
    RED_NOISE = "red-noise"  # Red (brownian) noise
    BLUE_NOISE = "blue-noise"  # Blue noise
    VIOLET_NOISE = "violet-noise"  # Violet noise


class TestSignalConfig(GStreamerNodeConfig):
    """Configuration for test signal generator."""

    waveform: WaveformType = Field(
        default=WaveformType.SINE, description="Type of waveform to generate"
    )
    frequency: float = Field(
        default=440.0, gt=0, description="Frequency in Hz (for periodic waveforms)"
    )
    volume: float = Field(default=0.8, ge=0.0, le=1.0, description="Volume level (0.0 to 1.0)")
    duration: Optional[float] = Field(
        default=None, gt=0, description="Optional duration in seconds (None = infinite)"
    )
    realtime: bool = Field(
        default=True, description="Emit audio at real-time speed instead of as fast as possible"
    )


class TestSignalNode(GStreamerSourceBase):
    """Test signal generator that emits AudioEvents."""

    def __init__(self, config: TestSignalConfig):
        super().__init__(config)
        self.config = config  # Type hint for better IDE support

    def _get_pipeline_string(self) -> str:
        """Build the test signal generator pipeline."""
        # Start with audiotestsrc
        parts = ["audiotestsrc"]

        # Add waveform type
        wave_value = self._get_gst_wave_value(self.config.waveform)
        parts.append(f"wave={wave_value}")

        # Add frequency for applicable waveforms
        if self.config.waveform in [
            WaveformType.SINE,
            WaveformType.SQUARE,
            WaveformType.SAW,
            WaveformType.TRIANGLE,
            WaveformType.SINE_TABLE,
        ]:
            parts.append(f"freq={self.config.frequency}")

        # Add volume
        parts.append(f"volume={self.config.volume}")

        # Add duration if specified
        if self.config.duration is not None:
            # Calculate total samples needed
            sample_rate = self.config.output.sample_rate or 44100
            total_samples = int(self.config.duration * sample_rate)
            # GStreamer typically uses buffers of 1920 samples for audio
            buffer_size = 1920
            num_buffers = max(1, (total_samples + buffer_size - 1) // buffer_size)
            parts.append(f"num-buffers={num_buffers}")
            logger.debug(
                f"Duration {self.config.duration}s = {total_samples} samples = {num_buffers} buffers"
            )

        # Join all parts
        pipeline = " ".join(parts)
        return pipeline

    def _get_gst_wave_value(self, waveform: WaveformType) -> str:
        """Convert WaveformType to GStreamer wave parameter value."""
        # GStreamer uses numeric values for wave types
        wave_map = {
            WaveformType.SINE: "0",
            WaveformType.SQUARE: "1",
            WaveformType.SAW: "2",
            WaveformType.TRIANGLE: "3",
            WaveformType.SILENCE: "4",
            WaveformType.WHITE_NOISE: "5",
            WaveformType.PINK_NOISE: "6",
            WaveformType.SINE_TABLE: "7",
            WaveformType.TICKS: "8",
            WaveformType.GAUSSIAN_NOISE: "9",
            WaveformType.RED_NOISE: "10",
            WaveformType.BLUE_NOISE: "11",
            WaveformType.VIOLET_NOISE: "12",
        }

        return wave_map.get(waveform, "0")  # Default to sine

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

    def _get_source_name(self) -> str:
        """Get descriptive name for this source."""
        return f"TestSignal[{self.config.waveform.value} @ {self.config.frequency}Hz]"


def test_signal(
    waveform: WaveformType = WaveformType.SINE,
    frequency: float = 440.0,
    volume: float = 0.8,
    duration: Optional[float] = None,
    **kwargs,
) -> AudioSource:
    """Create a test signal generator source.

    Args:
        waveform: Type of waveform to generate
        frequency: Frequency in Hz (for periodic waveforms)
        volume: Volume level (0.0 to 1.0)
        duration: Optional duration in seconds (None = infinite)
        **kwargs: Additional arguments passed to TestSignalConfig:
            - output: Output audio specification (default: OPUS compressed)
            - properties: GStreamer element properties

    Returns:
        AudioSource function that creates the observable

    Examples:
        # Generate a 1kHz sine wave
        source = test_signal(frequency=1000)

        # Generate white noise at 16kHz mono
        source = test_signal(
            waveform=WaveformType.WHITE_NOISE,
            output=AudioSpec(
                format=AudioFormat.PCM_F32LE,
                sample_rate=16000,
                channels=1
            )
        )

        # Generate a 5 second square wave
        source = test_signal(
            waveform=WaveformType.SQUARE,
            frequency=100,
            duration=5.0
        )
    """
    config = TestSignalConfig(
        waveform=waveform, frequency=frequency, volume=volume, duration=duration, **kwargs
    )

    def create_source():
        node = TestSignalNode(config)
        return node.create_observable()

    return create_source
