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

"""Tests for network audio output."""

import time

import pytest

from dimos.stream.audio2.input.file import file_input
from dimos.stream.audio2.input.signal import WaveformType, signal
from dimos.stream.audio2.output.network import network_output
from dimos.stream.audio2.types import AudioFormat, AudioSpec
from dimos.utils.data import get_data


def test_network_output_to_server():
    """Test streaming to actual server (requires manual setup)."""

    # To run this test:
    # 1. Start the server: ./gstreamer_scripts/gstreamer.sh
    # 2. Run this test with: pytest test_network.py::test_network_output_to_server

    duration = 3.0
    signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.5,
        duration=duration,
        output=AudioSpec(format=AudioFormat.PCM_F32LE),
    ).pipe(network_output(host="127.0.0.1", port=5002, codec="opus", queue_size=25)).run()

    # With sync=True, packets are sent at real-time pace
    # Wait for the audio duration + a bit extra for cleanup
    time.sleep(duration + 0.5)


def test_network_output_to_server_file():
    """Test streaming to actual server (requires manual setup)."""

    # To run this test:
    # 1. Start the server: ./gstreamer_scripts/gstreamer.sh
    # 2. Run this test with: pytest test_network.py::test_network_output_to_server
    file_input(
        file_path=str(get_data("out_of_date.wav")),
        realtime=False,  # Fast playback for testing
    ).pipe(network_output(host="127.0.0.1", port=5002, codec="opus", queue_size=50)).run()

    time.sleep(0.5)
