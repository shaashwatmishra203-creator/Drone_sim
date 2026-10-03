#!/bin/bash
# watch_mission.sh — fly the factory mission with the Gazebo window visible.
#
# Different order from run_scenario.sh on purpose: the world and the GUI come
# up FIRST and are verified, then PX4 attaches to the already-running world
# (px4-rc.gzsim detects one and skips its own launch). That way the window is
# definitely on screen before anything flies, rather than racing the boot.
#
# Usage:  bash watch_mission.sh [prop] [payload]

PROP=${1:-9x4.5}
PAYLOAD=${2:-pi5}
PX4=$HOME/PX4-Autopilot
DS=$HOME/drone_sim
WS=$HOME/ws_px4
SC=$DS/scenarios/factory_mission.yaml

PROPTAG=$(echo $PROP | tr -d '.')
MODEL=n360_quad_${PROPTAG}_${PAYLOAD}
case "${PROPTAG}_${PAYLOAD}" in
  9x45_pi5) AUTOSTART=22000 ;; 11x45_pi5) AUTOSTART=22001 ;;
  13x44_pi5) AUTOSTART=22002 ;; 9x45_orin_nx) AUTOSTART=22003 ;;
  11x45_orin_nx) AUTOSTART=22004 ;; 13x44_orin_nx) AUTOSTART=22005 ;;
  *) echo "unknown combo"; exit 2 ;;
esac

OUT=$DS/out/runs/watch_${PROPTAG}_${PAYLOAD}
mkdir -p $OUT

cleanup() {
  echo; echo "stopping..."
  pkill -f "drone_eval/" 2>/dev/null
  pkill -f "bin/px4" 2>/dev/null
  pkill -f "gz sim" 2>/dev/null
  pkill -f MicroXRCEAgent 2>/dev/null
  sleep 1
}
trap cleanup EXIT INT TERM

echo "clearing any leftovers..."
cleanup
sleep 3

export GZ_SIM_RESOURCE_PATH=$PX4/Tools/simulation/gz/models:$PX4/Tools/simulation/gz/worlds
export LD_LIBRARY_PATH=$HOME/.local/lib:${LD_LIBRARY_PATH:-}

echo "1/5  starting Gazebo server (world: factory)"
gz sim -r -s "$PX4/Tools/simulation/gz/worlds/factory.sdf" >$OUT/gz_server.log 2>&1 &
for i in $(seq 1 60); do
  gz service -i --service /world/factory/scene/info 2>&1 | grep -q "Service providers" && break
  sleep 1
done
if ! gz service -i --service /world/factory/scene/info 2>&1 | grep -q "Service providers"; then
  echo "  server failed:"; tail -5 $OUT/gz_server.log; exit 1
fi
echo "     world ready"

echo "2/5  opening the Gazebo window"
gz sim -g >$OUT/gz_gui.log 2>&1 &
GUI=$!
for i in $(seq 1 40); do
  sleep 1
  gz service -l 2>/dev/null | grep -q "^/gui/follow$" && break
done
if ! kill -0 $GUI 2>/dev/null; then
  echo "  the GUI process died:"; tail -6 $OUT/gz_gui.log; exit 1
fi
if ! gz service -l 2>/dev/null | grep -q "^/gui/follow$"; then
  echo "  GUI is running but not responding yet — continuing anyway"
else
  # A live pid proves nothing: the GUI can run headless-ish and never map a
# window. Ask the X server whether a window called "Gazebo Sim" is actually
# mapped, then raise it on the Windows side - under WSLg it comes up BEHIND
# the terminal, which is what made earlier runs look like a launch failure.
WIN=""
for i in $(seq 1 20); do
  if xwininfo -name "Gazebo Sim" 2>/dev/null | grep -q "IsViewable"; then WIN=yes; break; fi
  sleep 1
done
if [ -z "$WIN" ]; then
  echo "     NO WINDOW MAPPED after 20 s - the GUI process is alive but drew nothing."
  echo "     check: $OUT/gz_gui.log"
else
  echo "     window is mapped and viewable (pid $GUI)"
  # Raising the window from here is not possible: WSL interop is disabled in
  # this distro (no WSLInterop in /proc/sys/fs/binfmt_misc), so no Windows .exe
  # can be launched from inside WSL. Under WSLg the window opens BEHIND the
  # terminal, which is exactly what made earlier runs look like a failure.
  echo ""
  echo "  >>> The window IS open, titled 'Gazebo Sim'. WSLg puts it BEHIND"
  echo "      this terminal. ALT-TAB to it, or click it in the taskbar."
  echo "      If you see '[WARN:COPY MODE]' in its title that is normal here"
  echo "      - it means WSLg is compositing without GPU passthrough."
  echo ""
fi
fi

echo
echo "  >>> Look for the 'Gazebo Sim' window now. It may be behind this one. <<<"
echo "      Press Enter when you can see it (or just wait 15 s)."
read -t 15 -r _ || true
echo

echo "3/5  starting MicroXRCEAgent"
$HOME/.local/bin/MicroXRCEAgent udp4 -p 8888 >$OUT/agent.log 2>&1 &
sleep 2

echo "4/5  attaching PX4 to the running world"
export PX4_SYS_AUTOSTART=$AUTOSTART PX4_SIM_MODEL=$MODEL PX4_GZ_WORLD=factory
export PX4_GZ_MODEL_POSE="0,0,0.2,0,0,0" HEADLESS=1
cd $PX4
./build/px4_sitl_default/bin/px4 -d >$OUT/px4.log 2>&1 &
for i in $(seq 1 90); do grep -q "Ready for takeoff" $OUT/px4.log && break; sleep 1; done
grep -q "Ready for takeoff" $OUT/px4.log || { echo "  PX4 not ready"; tail -8 $OUT/px4.log; exit 1; }
echo "     PX4 ready"

# point the camera at the vehicle now that both exist
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
  --timeout 2000 --req "data: \"${MODEL}_0\"" >/dev/null 2>&1
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean \
  --timeout 2000 --req "x: -7, y: -5, z: 4" >/dev/null 2>&1

PARAMS=$(python3 -c "
import yaml; p=yaml.safe_load(open('$SC')).get('px4_params',{}) or {}
print(' '.join(f'{k}={v}' for k,v in p.items()))")
for kv in $PARAMS; do
  $PX4/build/px4_sitl_default/bin/px4-param set "${kv%%=*}" "${kv##*=}" >/dev/null 2>&1
done

set +u
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
export PYTHONUNBUFFERED=1
set -u

echo "5/5  starting perception, fusion, logging and the mission"
ros2 run drone_eval camera_mapper --ros-args \
  -p config:=$DS/config/airframe.yaml -p world_tools:=$DS/tools \
  -p out_json:=$OUT/camera_map.json >$OUT/mapper.log 2>&1 &
ros2 run drone_eval state_estimator --ros-args \
  -p world_tools:=$DS/tools \
  -p out_json:=$OUT/state_estimate.json >$OUT/estimator.log 2>&1 &
ros2 run drone_eval flight_logger --ros-args \
  -p config:=$DS/config/airframe.yaml -p prop:=$PROP -p payload:=$PAYLOAD \
  -p scenario:=watch -p out_csv:=$OUT/flight_log.csv \
  -p out_json:=$OUT/thrust_power.json >$OUT/logger.log 2>&1 &
sleep 3
ros2 run drone_eval mission_runner --ros-args -p scenario:=$SC >$OUT/runner.log 2>&1 &
RUNNER=$!

echo
echo "  flying — watch the window. The drone threads four gates, scans the room,"
echo "  then returns to base. Roughly 2-3 minutes."
echo
for i in $(seq 1 420); do
  kill -0 $RUNNER 2>/dev/null || break
  sleep 1
done
sleep 3

echo
echo "=============== RESULT ==============="
grep -E "LEG |TOTAL|camera-guided" $OUT/runner.log | sed 's/.*mission_runner\]: //' | sed 's/^/  /'
echo
grep -E "wrote .*camera_map" $OUT/mapper.log | sed 's/.*camera_mapper\]: //' | sed 's/^/  /'
grep -E "wrote .*state_estimate" $OUT/estimator.log | sed 's/.*state_estimator\]: //' | sed 's/^/  /'
grep -E "wrote .*thrust_power" $OUT/logger.log | sed 's/.*flight_logger\]: //' | sed 's/^/  /'
echo
echo "  outputs in $OUT"
echo
echo "  The window stays open so you can look around. Press Enter to close."
read -r _ || true
