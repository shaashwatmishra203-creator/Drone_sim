#!/bin/bash
# run_scenario.sh — bring up the whole stack for one scenario and tear it down.
#
#   MicroXRCEAgent  <->  PX4 SITL  <->  Gazebo Harmonic
#                            |
#                     drone_eval/flight_logger  (CSV)
#                     drone_eval/mission_runner (offboard profile)
#
# Usage: run_scenario.sh <scenario.yaml> [prop_id] [payload]

# NOTE: no `set -u` — ROS 2's setup.bash references unbound variables and
# would abort the run before anything starts.
SC=${1:?usage: run_scenario.sh <scenario.yaml> [prop] [payload]}
# Resolve to an absolute path: the ROS nodes run with their own working
# directory, so a relative path here makes mission_runner exit with
# "scenario not found" and the aircraft silently never arms.
SC=$(readlink -f "$SC") || { echo "cannot resolve scenario path: $1"; exit 2; }
[ -f "$SC" ] || { echo "scenario not found: $SC"; exit 2; }
PX4_PID=""; LOGGER_PID=""; RUNNER_PID=""; WIND_PID=""; GUI_PID=""; MAPPER_PID=""
PROP=${2:-9x4.5}
PAYLOAD=${3:-pi5}

PX4=~/PX4-Autopilot
DS=~/drone_sim
WS=~/ws_px4

NAME=$(python3 -c "import yaml,sys;print(yaml.safe_load(open('$SC'))['name'])")
WORLD=$(python3 -c "import yaml,sys;print(yaml.safe_load(open('$SC')).get('world','default'))")
WIND=$(python3 -c "
import yaml;w=yaml.safe_load(open('$SC')).get('wind',{});m=w.get('mean_ms',[0,0,0])
print(' '.join(str(float(x)) for x in m), float(w.get('gust_ms',0)))")
read WX WY WZ GUST <<< "$WIND"

PROPTAG=$(echo $PROP | tr -d '.')
MODEL=n360_quad_${PROPTAG}_${PAYLOAD}
case "${PROPTAG}_${PAYLOAD}" in
  9x45_pi5)      AUTOSTART=22000 ;;
  11x45_pi5)     AUTOSTART=22001 ;;
  13x44_pi5)     AUTOSTART=22002 ;;
  9x45_orin_nx)  AUTOSTART=22003 ;;
  11x45_orin_nx) AUTOSTART=22004 ;;
  13x44_orin_nx) AUTOSTART=22005 ;;
  *) echo "unknown combo ${PROPTAG}_${PAYLOAD}"; exit 2 ;;
esac

OUT=$DS/out/runs/${NAME}_${PROPTAG}_${PAYLOAD}
mkdir -p $OUT
CSV=$OUT/flight_log.csv

cleanup() {
  kill $WIND_PID $GUI_PID 2>/dev/null
  kill $LOGGER_PID $RUNNER_PID $MAPPER_PID $PX4_PID 2>/dev/null
  pkill -f "gz sim -g" 2>/dev/null
  pkill -f MicroXRCEAgent 2>/dev/null
  pkill -f "gz sim" 2>/dev/null
  # by name as well — a PID kill misses anything respawned or detached
  pkill -f "drone_eval/flight_logger" 2>/dev/null
  pkill -f "drone_eval/mission_runner" 2>/dev/null
  sleep 1
}
trap cleanup EXIT
# Kill the ROS nodes by name too, not just by PID at exit. A logger left over
# from an interrupted run keeps writing to the SAME csv path as the new run,
# and the two interleave: one run looked like 1295 s airborne and 201 Wh when
# the real mission was 90 s and 16 Wh.
pkill -f "bin/px4" 2>/dev/null; pkill -f "gz sim" 2>/dev/null
pkill -f MicroXRCEAgent 2>/dev/null
pkill -f "drone_eval/flight_logger" 2>/dev/null
pkill -f "drone_eval/mission_runner" 2>/dev/null
sleep 2

echo "=============================================================="
echo " scenario : $NAME"
echo " model    : $MODEL  (autostart $AUTOSTART)"
echo " world    : $WORLD   wind ${WX}/${WY}/${WZ} m/s  gust ${GUST}"
echo " out      : $CSV"
echo "=============================================================="

# The agent's shared libs live in ~/.local/lib and are not on the default
# loader path; without this it dies instantly and PX4 sits "disconnected"
# with no /fmu topics at all.
export LD_LIBRARY_PATH=$HOME/.local/lib:${LD_LIBRARY_PATH:-}
~/.local/bin/MicroXRCEAgent udp4 -p 8888 >$OUT/agent.log 2>&1 &
AGENT_PID=$!
sleep 2
if ! kill -0 $AGENT_PID 2>/dev/null; then
  echo "!!! MicroXRCEAgent failed to start:"; cat $OUT/agent.log; exit 1
fi
echo "agent up (pid $AGENT_PID)"

cd $PX4
export PX4_SYS_AUTOSTART=$AUTOSTART
export PX4_SIM_MODEL=$MODEL
export PX4_GZ_WORLD=$WORLD
export PX4_GZ_MODEL_POSE="0,0,0.2,0,0,0"
# PX4 v1.18 starts the Gazebo server only and ignores HEADLESS for gz, so the
# GUI client has to be launched separately. GUI=1 does that; default is
# headless, which is much faster and is what the batch uses.
export HEADLESS=1
if [ "${GUI:-0}" = "1" ]; then
  echo "GUI mode: a Gazebo window will open once the scene is ready"
  (
    # Wait for the SCENE to exist, not merely for the server to publish topics.
    # Launching the GUI against a half-built scene segfaults inside the WSL
    # D3D12 translation layer. scene/info is the same gate PX4 itself waits on.
    for i in $(seq 1 120); do
      W=$(gz topic -l 2>/dev/null | grep -m1 -e "^/world/.*/clock" \
            | sed 's#/world/##; s#/clock##')
      if [ -n "$W" ] && gz service -i --service "/world/$W/scene/info" 2>&1 \
           | grep -q "Service providers"; then
        break
      fi
      sleep 1
    done
    sleep 3
    gz sim -g >$OUT/gz_gui.log 2>&1 &
    GZGUI=$!

    # PX4 asks the GUI to follow the vehicle during ITS boot, which is before
    # this GUI exists, so that request is lost and the camera stays parked at
    # the world origin looking at nothing. Re-issue it once the GUI is up.
    for i in $(seq 1 40); do
      sleep 1
      gz service -l 2>/dev/null | grep -q "^/gui/follow$" || continue
      gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
        --reptype gz.msgs.Boolean --timeout 2000 \
        --req "data: \"${MODEL}_0\"" >/dev/null 2>&1
      gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d \
        --reptype gz.msgs.Boolean --timeout 2000 \
        --req "x: -6, y: -4, z: 4" >/dev/null 2>&1
      echo "camera following ${MODEL}_0" >>$OUT/gz_gui.log
      break
    done

    wait $GZGUI
    # The renderer can still lose the race on a loaded machine; ogre1 is
    # lighter and has not been seen to fail here.
    if ! pgrep -f "gz sim -g" >/dev/null; then
      echo "GUI died, retrying with ogre1" >>$OUT/gz_gui.log
      gz sim -g --render-engine ogre >>$OUT/gz_gui.log 2>&1
    fi
  ) &
  GUI_PID=$!
fi
export GZ_SIM_RESOURCE_PATH=$PX4/Tools/simulation/gz/models:$PX4/Tools/simulation/gz/worlds
./build/px4_sitl_default/bin/px4 -d >$OUT/px4.log 2>&1 &
PX4_PID=$!

echo "waiting for PX4..."
for i in $(seq 1 90); do
  grep -q "Ready for takeoff" $OUT/px4.log 2>/dev/null && break
  sleep 1
done
if ! grep -q "Ready for takeoff" $OUT/px4.log; then
  echo "PX4 failed to become ready:"; tail -20 $OUT/px4.log; exit 1
fi
echo "PX4 ready."

# Per-scenario PX4 parameters. Without a speed cap the position controller
# accelerates to MPC_XY_VEL_MAX (~12 m/s by default) between waypoints, which
# is far too fast to thread a 2.5 m gate: one run peaked at 11.5 m/s, clipped
# gate 0 and tumbled. Survey flight should be deliberate and repeatable.
PARAMS=$(python3 -c "
import yaml
p = yaml.safe_load(open('$SC')).get('px4_params', {}) or {}
print(' '.join(f'{k}={v}' for k, v in p.items()))")
if [ -n "$PARAMS" ]; then
  echo "applying scenario params: $PARAMS"
  for kv in $PARAMS; do
    $PX4/build/px4_sitl_default/bin/px4-param set "${kv%%=*}" "${kv##*=}" \
      >/dev/null 2>&1
  done
fi

# Wind is applied through gz transport so it is scriptable per scenario.
#
# It must be ramped in AFTER the vehicle is airborne. Stepping a 6 m/s wind
# onto a grounded vehicle whose estimator is still settling is an impulse, and
# PX4 fails it out with "Attitude failure (roll)" before it ever takes off.
# That is an artefact of how the wind is applied, not a property of the
# aircraft — a real drone does not meet a step change in wind at t=0.
wind_ramp() {
  # wait for the mission runner to report it reached altitude
  for i in $(seq 1 180); do
    grep -q "profile start" $OUT/runner.log 2>/dev/null && break
    sleep 1
  done
  sleep 3
  STEPS=6
  for s in $(seq 1 $STEPS); do
    read RX RY RZ <<< "$(python3 -c "
f=$s/$STEPS; print(f'{$WX*f:.3f} {$WY*f:.3f} {$WZ*f:.3f}')")"
    gz topic -t /world/$WORLD/wind -m gz.msgs.Wind \
      -p "linear_velocity: {x: $RX, y: $RY, z: $RZ}, enable_wind: true" \
      >/dev/null 2>&1
    sleep 2
  done
  echo "wind ramped to ${WX}/${WY}/${WZ} m/s" >> $OUT/wind.log
  # Gusts: perturb around the mean.
  if [ "$(python3 -c "print(1 if $GUST > 0 else 0)")" = "1" ]; then
    while true; do
      read GX GY <<< "$(python3 -c "
import random; print(f'{$WX+random.uniform(-$GUST,$GUST):.3f} {$WY+random.uniform(-$GUST,$GUST):.3f}')")"
      gz topic -t /world/$WORLD/wind -m gz.msgs.Wind \
        -p "linear_velocity: {x: $GX, y: $GY, z: $WZ}, enable_wind: true" \
        >/dev/null 2>&1
      sleep 3
    done
  fi
}

if [ "$(python3 -c "print(1 if abs($WX)+abs($WY)+abs($WZ) > 0 else 0)")" = "1" ]; then
  echo "wind ${WX}/${WY}/${WZ} m/s (gust ${GUST}) will ramp in after takeoff"
  wind_ramp &
  WIND_PID=$!
fi

set +u
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
# Without this, Python block-buffers stdout when it is redirected to a file and
# the node logs stay empty until the process exits — which is exactly when a
# killed node never flushes them at all.
export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0

echo "=== confirming the DDS bridge actually delivers ==="
for i in $(seq 1 20); do
  N=$(timeout 5 ros2 topic list 2>/dev/null | grep -c "^/fmu/out/")
  [ "${N:-0}" -gt 0 ] && break
  sleep 1
done
echo "  /fmu/out topics visible: ${N:-0}"
if [ "${N:-0}" -eq 0 ]; then
  echo "!!! no /fmu topics — the uXRCE-DDS bridge is not delivering."
  echo "    check: agent running? LD_LIBRARY_PATH? ROS_DOMAIN_ID?"
  tail -5 $OUT/agent.log; exit 1
fi

# NOTE: deliberately NOT use_sim_time. Nothing publishes /clock here (that
# would need a ros_gz bridge, which is not installed), so use_sim_time would
# freeze both nodes at t=0. They read PX4's message timestamps instead.
echo "starting camera_mapper (Arducam perception)..."
ros2 run drone_eval camera_mapper --ros-args \
  -p config:=$DS/config/airframe.yaml \
  -p world_tools:=$DS/tools \
  -p mode:=${PERCEPTION_MODE:-geometric} \
  -p out_json:=$OUT/camera_map.json >$OUT/mapper.log 2>&1 &
MAPPER_PID=$!
sleep 2

echo "starting flight_logger..."
ros2 run drone_eval flight_logger --ros-args \
  -p config:=$DS/config/airframe.yaml \
  -p out_json:=$OUT/thrust_power.json \
  -p prop:=$PROP -p payload:=$PAYLOAD \
  -p scenario:=$NAME -p out_csv:=$CSV >$OUT/logger.log 2>&1 &
LOGGER_PID=$!
sleep 3

echo "starting mission_runner..."
ros2 run drone_eval mission_runner --ros-args \
  -p scenario:=$SC >$OUT/runner.log 2>&1 &
RUNNER_PID=$!

MAXWALL=$(python3 -c "import yaml;print(int(yaml.safe_load(open('$SC')).get('max_duration_s',600))+120)")
echo "running (wall cap ${MAXWALL}s)..."
for i in $(seq 1 $MAXWALL); do
  kill -0 $RUNNER_PID 2>/dev/null || break
  sleep 1
done

sleep 2
kill $LOGGER_PID 2>/dev/null
sleep 1

echo
kill $MAPPER_PID 2>/dev/null
sleep 1

echo "=== runner ==="; tail -6 $OUT/runner.log
echo "=== mapper ==="; tail -4 $OUT/mapper.log
echo "=== logger ==="; tail -4 $OUT/logger.log
echo "=== json outputs ==="
for f in $OUT/thrust_power.json $OUT/camera_map.json; do
  [ -s "$f" ] && echo "  $(basename $f): $(du -h $f | cut -f1)" \
               || echo "  $(basename $f): MISSING"
done
echo "=== csv ==="
if [ -s "$CSV" ]; then
  echo "rows: $(( $(wc -l < $CSV) - 1 ))"
  head -1 $CSV | tr ',' '\n' | head -3 >/dev/null
  python3 $DS/tools/summarize_run.py $CSV || true
else
  echo "!!! CSV EMPTY — check QoS first (BEST_EFFORT), not the model."
  tail -20 $OUT/logger.log
fi
