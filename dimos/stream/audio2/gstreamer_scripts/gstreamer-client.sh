#!/bin/bash
# Replace TARGET_IP with the IP address of the machine running gstreamer.sh
TARGET_IP="${1:-localhost}"  # First argument or default to localhost
ffmpeg -re -i ~/old/src/whisper.cpp/samples/jfk.mp3 -c:a libopus -b:a 128k -f rtp rtp://127.0.0.1:5002
