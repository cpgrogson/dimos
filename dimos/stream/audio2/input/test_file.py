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

"""Tests for file input audio node."""

import threading
import time

import numpy as np
import pytest

from dimos.stream.audio2.input.file import file_input
from dimos.stream.audio2.output.soundcard import speaker
from dimos.stream.audio2.types import (
    AudioEvent,
    AudioFormat,
    AudioSpec,
    CompressedAudioEvent,
    RawAudioEvent,
)
from dimos.utils.data import get_data


def test_file_input_raw_audio():
    """Test file input with raw PCM output."""
    # Get test audio file
    file_path = get_data("petty_concerns.wav")

    # Configure for raw output, disable realtime for faster testing
    output = AudioSpec(format=AudioFormat.PCM_F32LE)
    source = file_input(file_path=str(file_path), loop=False, realtime=False, output=output)

    # Track events
    events = []
    errors = []
    complete_event = threading.Event()

    def on_next(event: AudioEvent):
        events.append(event)

    def on_error(e):
        errors.append(e)
        complete_event.set()

    def on_completed():
        complete_event.set()

    source().subscribe(on_next=on_next, on_error=on_error, on_completed=on_completed)

    assert complete_event.wait(timeout=5.0), "Stream did not complete in time"

    # Give time for threads to clean up
    time.sleep(0.1)

    # Verify results
    assert len(errors) == 0, f"Unexpected errors: {errors}"
    assert len(events) > 0, "No audio events received"

    # Check all events are raw audio
    for event in events:
        assert isinstance(event, RawAudioEvent), f"Expected RawAudioEvent, got {type(event)}"

    # Check first event properties
    first_event = events[0]
    assert first_event.sample_rate == 48000, f"Expected 48kHz, got {first_event.sample_rate}"
    assert first_event.channels == 2, f"Expected 2 channels, got {first_event.channels}"
    assert first_event.format == AudioFormat.PCM_F32LE

    # Check we got a reasonable number of events (should be 287 for petty_concerns.wav)
    assert 280 <= len(events) <= 300, f"Unexpected number of events: {len(events)}"

    # Check data shape
    for event in events[:-1]:  # All but last event should have consistent size
        assert event.data.shape == (1920, 2), f"Unexpected data shape: {event.data.shape}"

    # Last event might be smaller
    assert events[-1].data.shape[1] == 2, "Last event should have 2 channels"
    assert events[-1].data.shape[0] <= 1920, "Last event should have <= 1920 samples"


def test_file_input_compressed_audio():
    """Test file input with compressed output."""
    # Get test audio file
    file_path = get_data("petty_concerns.wav")

    # Configure for compressed output (default Vorbis), disable realtime for testing
    source = file_input(file_path=str(file_path), loop=False, realtime=False)

    # Track events
    events = []
    errors = []
    complete_event = threading.Event()

    def on_next(event: AudioEvent):
        events.append(event)

    def on_error(e):
        errors.append(e)
        complete_event.set()

    def on_completed():
        complete_event.set()

    # Subscribe and wait for completion
    subscription = source().subscribe(on_next=on_next, on_error=on_error, on_completed=on_completed)

    # Wait for stream to complete (with timeout)
    assert complete_event.wait(timeout=5.0), "Stream did not complete in time"

    # Clean up
    subscription.dispose()

    # Give time for threads to clean up
    import time

    time.sleep(0.1)

    # Verify results
    assert len(errors) == 0, f"Unexpected errors: {errors}"
    assert len(events) > 0, "No audio events received"

    # Check all events are compressed audio
    for event in events:
        assert isinstance(event, CompressedAudioEvent), (
            f"Expected CompressedAudioEvent, got {type(event)}"
        )

    # Check first event properties
    first_event = events[0]
    assert first_event.sample_rate == 48000, f"Expected 48kHz, got {first_event.sample_rate}"
    assert first_event.channels == 2, f"Expected 2 channels, got {first_event.channels}"
    assert first_event.format == AudioFormat.VORBIS

    # Check we have compressed data
    for event in events:
        assert isinstance(event.data, bytes), "Compressed data should be bytes"
        assert len(event.data) > 0, "Compressed data should not be empty"


def test_file_input_nonexistent_file():
    """Test file input with non-existent file."""
    from pydantic import ValidationError

    with pytest.raises(ValidationError, match="Audio file not found"):
        source = file_input(file_path="/tmp/nonexistent_audio_file.wav")


def test_file_input_custom_config():
    """Test file input with custom configuration."""
    file_path = get_data("petty_concerns.wav")

    # Configure with specific sample rate
    output = AudioSpec(format=AudioFormat.PCM_S16LE, sample_rate=16000, channels=1)
    source = file_input(
        file_path=str(file_path),
        loop=False,
        realtime=False,  # Disable for faster testing
        output=output,
        properties={"max-buffers": 20},  # Custom GStreamer property
    )

    # Track first event only
    first_event = []
    errors = []
    got_event = threading.Event()

    def on_next(event: AudioEvent):
        if not first_event:
            first_event.append(event)
            got_event.set()

    def on_error(e):
        errors.append(e)
        got_event.set()

    # Subscribe
    subscription = source().subscribe(on_next=on_next, on_error=on_error)

    # Wait for first event
    assert got_event.wait(timeout=2.0), "No event received"

    # Clean up
    subscription.dispose()

    # Give time for threads to clean up
    import time

    time.sleep(0.1)

    # Verify
    assert len(errors) == 0, f"Unexpected errors: {errors}"
    assert len(first_event) == 1, "Should have received an event"

    event = first_event[0]
    assert isinstance(event, RawAudioEvent)
    assert event.format == AudioFormat.PCM_S16LE
    # Note: GStreamer will resample to 16kHz as requested
    assert event.sample_rate == 16000, f"Expected 16kHz, got {event.sample_rate}"
    assert event.channels == 1, f"Expected 1 channel, got {event.channels}"
    assert event.dtype == np.int16, f"Expected int16 dtype, got {event.dtype}"


@pytest.mark.tool
def test_file_input_with_soundcard():
    """Test file input playing through soundcard output."""
    # Get test audio file
    file_path = get_data("out_of_date.wav")

    # Configure for raw output to test PCM format
    output = AudioSpec(format=AudioFormat.PCM_F32LE)
    source = file_input(
        file_path=str(file_path),
        loop=False,
        realtime=True,  # Disable realtime for faster testing
        output=output,
    )

    # Create speaker output
    sink = speaker()

    # Track completion and errors
    completed = threading.Event()
    errors = []
    event_count = [0]

    def on_next(event: AudioEvent):
        event_count[0] += 1
        # Verify first event to ensure proper format
        if event_count[0] == 1:
            assert isinstance(event, RawAudioEvent)
            assert event.format == AudioFormat.PCM_F32LE
            assert event.sample_rate == 48000
            assert event.channels == 2
            # Check data is in valid range for audio
            assert -1.0 <= event.data.min() <= 1.0
            assert -1.0 <= event.data.max() <= 1.0
        sink.on_next(event)

    def on_error(e):
        errors.append(e)
        sink.on_error(e)
        completed.set()

    def on_completed():
        sink.on_completed()
        completed.set()

    # Subscribe and play
    observable = source()
    subscription = observable.subscribe(
        on_next=on_next, on_error=on_error, on_completed=on_completed
    )

    try:
        # Wait for the file to complete playing (with timeout for safety)
        assert completed.wait(timeout=20.0), "Audio playback did not complete in time"

        # Short wait for cleanup
        time.sleep(0.1)

        # Verify we received events and no errors
        assert len(errors) == 0, f"Unexpected errors: {errors}"
        assert event_count[0] > 0, "No audio events received"

    finally:
        # Ensure cleanup
        if not subscription.is_disposed:
            subscription.dispose()
        sink.stop()
