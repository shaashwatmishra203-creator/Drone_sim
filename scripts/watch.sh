#!/bin/bash
# watch.sh — open the simulation with the Gazebo window visible and fly it
# yourself from the PX4 console.
#
#   bash ~/drone_sim/scripts/watch.sh                      # 9in, Pi5, default world
#   bash ~/drone_sim/scripts/watch.sh 13x4.4 pi5 windy     # 13in props, windy world
#
# Once the PX4 "pxh>" prompt appears, useful commands:
#   commander takeoff
#   commander land
#   commander mode auto:loiter
#   listener actuator_motors        <- the per-motor commands
#   listener vehicle_local_position <- position and velocity
#   shutdown                        <- quit cleanly
#
# To add wind while it is flying, from a SECOND terminal:
#   gz topic -t /world/windy/wind -m gz.msgs.Wind \
#     -p 'linear_velocity: {x: 6.0, y: 0, z: 0}, enable_wind: true'

PROP=${1:-9x4.5}
PAYLOAD=${2:-pi5}
WORLD=${3:-default}

PX4=~/PX4-Autopilot
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

if pgrep -f "bin/px4" >/dev/null; then
  echo "A PX4 instance is already running (the batch, maybe?)."
  echo "Stop it first:  pkill -f 'bin/px4'; pkill -f 'gz sim'"
  exit 1
fi

cleanup() { pkill -f MicroXRCEAgent 2>/dev/null; pkill -f "gz sim" 2>/dev/null; }
trap cleanup EXIT

export LD_LIBRARY_PATH=$HOME/.local/lib:${LD_LIBRARY_PATH:-}
~/.local/bin/MicroXRCEAgent udp4 -p 8888 >/tmp/watch_agent.log 2>&1 &
sleep 2

echo "=================================================="
echo " model : $MODEL"
echo " world : $WORLD"
echo " GUI   : on (WSLg). First launch can take ~30 s."
echo "=================================================="
echo
echo "At the pxh> prompt, try:  commander takeoff"
echo

cd $PX4
export PX4_SYS_AUTOSTART=$AUTOSTART
export PX4_SIM_MODEL=$MODEL
export PX4_GZ_WORLD=$WORLD
export PX4_GZ_MODEL_POSE="0,0,0.2,0,0,0"
export GZ_SIM_RESOURCE_PATH=$PX4/Tools/simulation/gz/models:$PX4/Tools/simulation/gz/worlds

# PX4 v1.18 starts the Gazebo SERVER only — it does not launch the GUI client,
# and HEADLESS is not wired into its gz launcher at all. So we start the GUI
# ourselves once the server is up. `gz sim -g` attaches to the running server.
(
  # Wait for the SCENE, not just for topics. Launching against a half-built
  # scene segfaults inside the WSL D3D12 layer.
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
  gz sim -g >/tmp/gz_gui.log 2>&1
  if ! pgrep -f "gz sim -g" >/dev/null; then
    gz sim -g --render-engine ogre >>/tmp/gz_gui.log 2>&1
  fi
) &
GUI_PID=$!
cleanup() {
  kill $GUI_PID 2>/dev/null
  pkill -f "gz sim -g" 2>/dev/null
  pkill -f MicroXRCEAgent 2>/dev/null
  pkill -f "gz sim" 2>/dev/null
}
trap cleanup EXIT

# Interactive: you get the pxh> console in this terminal.
./build/px4_sitl_default/bin/px4
