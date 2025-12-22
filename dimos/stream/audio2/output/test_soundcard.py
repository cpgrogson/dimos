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

"""Tests for soundcard output."""

import threading
import time

import pytest

from dimos.stream.audio2.input.signal import WaveformType, test_signal
from dimos.stream.audio2.output.soundcard import speaker
from dimos.stream.audio2.types import AudioFormat, AudioSpec


def test_speaker_with_test_signal():
    """Test playing a test signal through speakers."""
    # Create a 440Hz sine wave for 2 seconds
    source = test_signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.5,  # Moderate volume for testing
        duration=2.0,
        output=AudioSpec(format=AudioFormat.PCM_F32LE),  # Raw audio
    )

    # Create speaker output
    sink = speaker()

    # Track completion
    completed = threading.Event()
    errors = []

    def on_error(e):
        errors.append(e)
        completed.set()

    def on_completed():
        completed.set()

    # Connect source to sink
    observable = source()

    # Subscribe with error/completion handlers
    # Sink will auto-start on first event
    subscription = observable.subscribe(
        on_next=sink.on_next,
        on_error=lambda e: (sink.on_error(e), on_error(e)),
        on_completed=lambda: (sink.on_completed(), on_completed()),
    )

    try:
        # Wait for completion
        assert completed.wait(timeout=5.0), "Audio playback did not complete"
        assert len(errors) == 0, f"Unexpected errors: {errors}"
        # IMPORTANT: Wait for audio to actually play out
        # The sink needs time to play the buffered audio

    finally:
        # Clean up
        subscription.dispose()
        sink.stop()
        time.sleep(0.1)
