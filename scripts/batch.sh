#!/bin/bash
# Full scenario matrix. Each entry: scenario prop payload
set +u
S=~/drone_sim/scenarios
R=~/drone_sim/scripts/run_scenario.sh
OUT=~/drone_sim/out/batch_summary.txt
: > $OUT

run() {
  echo "" | tee -a $OUT
  echo "################ $1 / $2 / $3 ################" | tee -a $OUT
  bash $R $S/$1.yaml $2 $3 2>&1 | sed -n '/=== csv ===/,$p' | tee -a $OUT
}

run hover_endurance 9x4.5  pi5
run hover_endurance 13x4.4 pi5
run hover_endurance 9x4.5  orin_nx
run cruise_sweep    9x4.5  pi5
run climb_test      9x4.5  pi5
run forest_mission  9x4.5  pi5

echo "" | tee -a $OUT
echo "BATCH COMPLETE" | tee -a $OUT
