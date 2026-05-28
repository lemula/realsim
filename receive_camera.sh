gst-launch-1.0 -e \
  compositor name=comp background=black \
    sink_0::xpos=0 sink_0::ypos=540 sink_0::width=1920 sink_0::height=540 \
    sink_1::xpos=0 sink_1::ypos=0   sink_1::width=1920 sink_1::height=540 ! \
    videoconvert ! videoscale ! video/x-raw,width=1920,height=1080 ! \
    autovideosink sync=false \
  udpsrc port=5500 buffer-size=524288 \
    caps="application/x-rtp, media=video, encoding-name=H264, payload=96, clock-rate=90000" ! \
    rtpjitterbuffer latency=80 drop-on-latency=true ! \
    rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! \
    queue max-size-buffers=4 max-size-bytes=0 max-size-time=0 leaky=downstream ! \
    comp.sink_0 \
  udpsrc port=5501 buffer-size=524288 \
    caps="application/x-rtp, media=video, encoding-name=H264, payload=97, clock-rate=90000" ! \
    rtpjitterbuffer latency=80 drop-on-latency=true ! \
    rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! \
    queue max-size-buffers=4 max-size-bytes=0 max-size-time=0 leaky=downstream ! \
    comp.sink_1
