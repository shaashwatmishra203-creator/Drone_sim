#!/bin/bash
# hall_run.sh - fly the v2 factory hall mission.
#
#   camera (Gazebo) -> cloud_link (JPEG, emulated WiFi) -> cloud_detector
#        -> boxes back over the link -> hall_mission (avoid, fly, return, land)
#
# Environment knobs:
#   GUI=1          open the Gazebo window        (default 0, headless)
#   VIEW=1         open the cloud CV window       (default 0)
#   HOLD=1         wait for /tmp/drone_go before take-off (default 0)
#   MODE=mission   or frame_check (positive control: 3 m E, 3 m N, back)
#   OUT=dir        output directory (default out/runs/hall_<mode>)
#   HOLD_S=600     keep the world open this long after landing when GUI=1
#   CONFIG=path    hall config (default config/factory_hall.yaml)
#
# One Gazebo instance, hardware GPU only. Processes are killed by PID, never by
# a pkill pattern, because `pkill -f` also matches the caller's own command line.

PX4=$HOME/PX4-Autopilot; DS=$HOME/drone_sim; WS=$HOME/ws_px4

# One run at a time. Two overlapping harnesses (the launcher started twice)
# shared an output folder and could each start a flight controller; that run
# had to be thrown away. The lock is released when this script exits.
exec 9>/tmp/hall_run.lock
if ! flock -n 9; then
  echo "another hall run is already active - not starting a second one"; exit 3
fi

MODE=${MODE:-mission}
CONFIG=$(readlink -f ${CONFIG:-$DS/config/factory_hall.yaml})
# every run gets its own folder; earlier runs all wrote to one and overwrote
# each other's raw data
OUT=$(readlink -f -m ${OUT:-$DS/out/runs/hall_${MODE}_$(date +%Y%m%d_%H%M%S)})
WORLD=factory_hall
MODEL=n360_quad_9x45_pi5
mkdir -p $OUT
ln -sfn $OUT $DS/out/runs/hall_latest
rm -f /tmp/drone_go
echo "output: $OUT"

ME=$$
kill_all() {
  for pat in "drone_eval/" "bin/px4" "MicroXRCEAgent" "gz sim" "gz-launch" "truth_logger"; do
    for p in $(pgrep -f "$pat"); do
      [ "$p" = "$ME" ] || kill "$p" 2>/dev/null
    done
  done
  sleep 2
  for pat in "drone_eval/" "bin/px4" "MicroXRCEAgent" "gz sim" "gz-launch" "truth_logger"; do
    for p in $(pgrep -f "$pat"); do [ "$p" = "$ME" ] || kill -9 "$p" 2>/dev/null; done
  done
}
trap 'echo; echo "stopping..."; kill_all' EXIT INT TERM
echo "clearing leftovers..."; kill_all

export GZ_SIM_RESOURCE_PATH=$PX4/Tools/simulation/gz/models:$PX4/Tools/simulation/gz/worlds
export LD_LIBRARY_PATH=$HOME/.local/lib:${LD_LIBRARY_PATH:-}
set +u; source /opt/ros/humble/setup.bash; source $WS/install/setup.bash
export PYTHONUNBUFFERED=1
# system numpy 1.21 for OpenCV / cv_bridge (a numpy 2 in ~/.local breaks them)
VPY="env PYTHONNOUSERSITE=1"
cd ~

echo "1/6  world: $WORLD"
# the server renders the drone camera; NVIDIA verified to give real frames
MESA_D3D12_DEFAULT_ADAPTER_NAME=${GZ_SERVER_GPU:-NVIDIA} \
  setsid gz sim -r -s "$PX4/Tools/simulation/gz/worlds/$WORLD.sdf" >$OUT/gz_server.log 2>&1 &
for i in $(seq 1 60); do
  gz service -i --service /world/$WORLD/scene/info 2>&1 | grep -q "Service providers" && break; sleep 1
done
gz service -i --service /world/$WORLD/scene/info 2>&1 | grep -q "Service providers" \
  || { echo "  world failed"; tail -5 $OUT/gz_server.log; exit 1; }

if [ "${GUI:-0}" = "1" ]; then
  echo "2/6  Gazebo window (GPU ${GZ_GUI_GPU:-NVIDIA})"
  for attempt in 1 2 3; do
    MESA_D3D12_DEFAULT_ADAPTER_NAME=${GZ_GUI_GPU:-NVIDIA} setsid gz sim -g >$OUT/gz_gui.log 2>&1 &
    G=$!; sleep 8
    kill -0 $G 2>/dev/null && break
    echo "     window crashed on start (attempt $attempt)"; cp $OUT/gz_gui.log $OUT/gz_gui_crash$attempt.log
  done
else
  echo "2/6  headless (no window)"
fi

echo "3/6  PX4 + DDS agent"
setsid $HOME/.local/bin/MicroXRCEAgent udp4 -p 8888 >$OUT/agent.log 2>&1 &
sleep 1
( cd $PX4 && PX4_SYS_AUTOSTART=22000 PX4_SIM_MODEL=$MODEL PX4_GZ_WORLD=$WORLD \
  PX4_GZ_MODEL_POSE="0,0,0.2,0,0,0" HEADLESS=1 \
  setsid ./build/px4_sitl_default/bin/px4 -d >$OUT/px4.log 2>&1 & )
for i in $(seq 1 90); do grep -q "Ready for takeoff" $OUT/px4.log && break; sleep 1; done
grep -q "Ready for takeoff" $OUT/px4.log || { echo "  PX4 not ready"; tail -8 $OUT/px4.log; exit 1; }
SPEED=$(python3 -c "import yaml;print(yaml.safe_load(open('$CONFIG'))['mission']['speed_ms'])")
for kv in MPC_XY_VEL_MAX=$SPEED MPC_XY_CRUISE=$SPEED MPC_ACC_HOR=1.2 MPC_ACC_HOR_MAX=1.5 \
          MPC_JERK_AUTO=3.0 MPC_Z_VEL_MAX_UP=1.0 MPC_Z_VEL_MAX_DN=0.7 MPC_TKO_SPEED=1.0 \
          MPC_YAWRAUTO_MAX=60; do
  $PX4/build/px4_sitl_default/bin/px4-param set "${kv%%=*}" "${kv##*=}" >/dev/null 2>&1
done
echo "     PX4 ready, drone on the pad, xy speed limit $SPEED m/s"
if [ "${GUI:-0}" = "1" ]; then
  gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
    --timeout 2000 --req "data: \"${MODEL}_0\"" >/dev/null 2>&1
  gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean \
    --timeout 2000 --req "x: -6, y: -4, z: 5" >/dev/null 2>&1
fi

echo "4/6  truth logger, cloud detector, emulated link, thrust logger"
setsid ros2 run drone_eval truth_logger --world $WORLD --model ${MODEL}_0 --out $OUT/truth.csv >$OUT/truth.log 2>&1 &
setsid $VPY ros2 run drone_eval cloud_detector --ros-args -p config:=$CONFIG \
  -p out_jsonl:=$OUT/detections_cloud.jsonl >$OUT/cloud_detector.log 2>&1 &
setsid $VPY ros2 run drone_eval cloud_link --ros-args -p config:=$CONFIG \
  -p out_json:=$OUT/link_stats.json >$OUT/cloud_link.log 2>&1 &
setsid ros2 run drone_eval flight_logger --ros-args -p config:=$DS/config/airframe.yaml \
  -p prop:=9x4.5 -p payload:=pi5 -p scenario:=hall_$MODE \
  -p out_csv:=$OUT/flight_log.csv -p out_json:=$OUT/thrust_power.json >$OUT/logger.log 2>&1 &
if [ "${VIEW:-0}" = "1" ]; then
  setsid $VPY ros2 run drone_eval vision_view --ros-args -p out_mp4:=$OUT/detections.mp4 >$OUT/vision_view.log 2>&1 &
else
  setsid $VPY ros2 run drone_eval vision_view --ros-args -p show:=false -p out_mp4:=$OUT/detections.mp4 >$OUT/vision_view.log 2>&1 &
fi
sleep 4

if [ "${HOLD:-0}" = "1" ]; then
  echo "     HOLDING: the drone is on the pad. Take-off when /tmp/drone_go exists."
  for i in $(seq 1 1800); do [ -e /tmp/drone_go ] && break; sleep 1; done
fi

echo "5/6  flying ($MODE)"
setsid $VPY ros2 run drone_eval hall_mission --ros-args -p config:=$CONFIG \
  -p out_dir:=$OUT -p mode:=$MODE >$OUT/mission.log 2>&1 &
MP=$!
for i in $(seq 1 700); do kill -0 $MP 2>/dev/null || break; sleep 1; done
sleep 3

echo "6/6  results"
grep -aE "LEG |TOTAL|decisions|detection messages|global plans|status:" $OUT/mission.log \
  | sed 's/.*hall_mission\]: //' | sed 's/^/   /'
tail -1 $OUT/cloud_link.log | sed 's/.*cloud_link\]: /   link: /'
python3 $DS/tools/hall_report.py --run $OUT --config $CONFIG 2>&1 | sed 's/^/   /'
echo "   load $(cut -d' ' -f1-3 /proc/loadavg), mem available $(free -m | awk '/Mem:/{print $7}') MB"
if [ "${GUI:-0}" = "1" ]; then
  echo "   world stays open ${HOLD_S:-600} s"; sleep ${HOLD_S:-600}
fi
