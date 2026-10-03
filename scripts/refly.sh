#!/bin/bash
# Re-fly the factory mission against the ALREADY-RUNNING world and PX4.
# Starts no new Gazebo instance: the point is to watch, not to rebuild.
set -u
DS=$HOME/drone_sim
OUT=$DS/out/runs/factory_mission_9x45_pi5
SC=$(readlink -f $DS/scenarios/factory_mission.yaml)
CSV=$OUT/flight.csv
export PYTHONUNBUFFERED=1
set +u; source /opt/ros/humble/setup.bash; source $HOME/ws_px4/install/setup.bash; set -u

for p in "gz sim -r -s" "gz sim -g" "bin/px4" MicroXRCEAgent; do
  pgrep -f "$p" >/dev/null || { echo "MISSING: $p - cannot replay, run watch_mission.sh instead"; exit 1; }
done
echo "world, GUI, PX4 and agent all alive - reusing them"

pkill -f "drone_eval/" 2>/dev/null; sleep 2

# re-point the GUI camera at the vehicle; PX4's own follow request is long gone
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
  --timeout 2000 --req 'data: "n360_quad_9x45_pi5_0"' >/dev/null 2>&1 && echo "camera following the drone"

ros2 run drone_eval camera_mapper --ros-args \
  -p config:=$DS/config/airframe.yaml -p world_tools:=$DS/tools \
  -p mode:=geometric -p out_json:=$OUT/camera_map.json >$OUT/mapper.log 2>&1 &
sleep 2
ros2 run drone_eval state_estimator --ros-args \
  -p world_tools:=$DS/tools -p out_json:=$OUT/state_estimate.json >$OUT/estimator.log 2>&1 &
sleep 1
ros2 run drone_eval flight_logger --ros-args \
  -p config:=$DS/config/airframe.yaml -p out_json:=$OUT/thrust_power.json \
  -p prop:=9x4.5 -p payload:=pi5 -p scenario:=factory_mission -p out_csv:=$CSV >$OUT/logger.log 2>&1 &
sleep 1
ros2 run drone_eval mission_runner --ros-args -p scenario:=$SC >$OUT/runner.log 2>&1 &
RUNNER=$!
echo "flying - watch the Gazebo window (~2 min)"
wait $RUNNER
echo "=== mission ==="
grep -E "LEG |TOTAL|camera-guided|mission complete" $OUT/runner.log | sed 's/.*mission_runner\]: //' | sed 's/^/  /'
