#!/bin/bash

gst-launch-1.0 -v \
  udpsrc address=0.0.0.0 port=5002 caps="application/x-rtp,media=audio,encoding-name=OPUS,payload=97" \
  ! rtpopusdepay ! opusdec ! audioconvert ! audioresample \
  ! autoaudiosink sync=false