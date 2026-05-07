#!/usr/bin/env bash
# Mirror /dev/CAM_FACE{1,2,3} to /dev/video{10,11,12} so multiple
# consumers (ustreamer preview + calibrate_camera) can read in parallel.
#
# Prerequisite (one-shot install):
#   sudo apt install v4l2loopback-dkms ffmpeg
#   sudo modprobe v4l2loopback devices=3 video_nr=10,11,12 \
#       card_label=CAM_FACE1_LOOP,CAM_FACE2_LOOP,CAM_FACE3_LOOP \
#       exclusive_caps=1
#
# Usage: cameras_loopback.sh {start|stop|status}

set -euo pipefail

WIDTH=1920
HEIGHT=1080
PIDFILE=/tmp/cameras_loopback.pids
REAL=(/dev/CAM_FACE1 /dev/CAM_FACE2 /dev/CAM_FACE3)
LOOP=(/dev/video10 /dev/video11 /dev/video12)

case "${1:-status}" in
  start)
    [[ -f $PIDFILE ]] && { echo "already running"; exit 1; }
    : > "$PIDFILE"
    for i in 0 1 2; do
      ffmpeg -loglevel warning -nostdin \
             -f v4l2 -input_format mjpeg \
             -video_size ${WIDTH}x${HEIGHT} -framerate 30 \
             -i "${REAL[$i]}" \
             -pix_fmt yuyv422 -f v4l2 "${LOOP[$i]}" \
             >/tmp/loopback_$i.log 2>&1 &
      echo $! >> "$PIDFILE"
    done
    sleep 1
    echo "loopbacks started:"
    paste <(printf '  %s\n' "${REAL[@]}") <(printf '-> %s\n' "${LOOP[@]}")
    ;;
  stop)
    [[ -f $PIDFILE ]] || { echo "not running"; exit 0; }
    while read -r pid; do kill "$pid" 2>/dev/null || true; done < "$PIDFILE"
    rm -f "$PIDFILE"
    echo "stopped"
    ;;
  status)
    if [[ -f $PIDFILE ]]; then
      echo "running, pids:"; cat "$PIDFILE"
    else
      echo "not running"
    fi
    ;;
  *)
    echo "usage: $0 {start|stop|status}" >&2; exit 2 ;;
esac
