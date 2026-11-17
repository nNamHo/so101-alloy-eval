#!/usr/bin/env bash
# run_trials.sh — repeated hardware trials with the command log and the
# verification output of EACH trial saved together, so every measurement can
# be traced to the exact joint vector that run commanded.
#
# Usage (from anywhere):
#   ~/so101_ws/src/so101_pose_milestone/scripts/run_trials.sh <target_index> [n_trials]
#   HW=mock ...run_trials.sh 0 2        # dry run in simulation, no prompts
#   COMP=false ...run_trials.sh 0 5     # baseline: gravity feed-forward off
#   (feed-forward, gravity_comp.py, is on by default on the real arm and off in
#   simulation, which has no sag to cancel; its folders end in _comp)
#
# Each trial:  arm at rest -> launch (bringup + commander) -> wait for the
# commander's outcome -> settle 3 s -> verify_pose.py -> you note whether
# anything touched the gripper -> torque released -> next trial.
#
# Output: docs/data/runs/<date>_<time>_target<N>/
#   trial<i>_launch.log   commander + driver log (contains "IK solution (rad)")
#   trial<i>_verify.txt   verify_pose.py output for that same trial
#   observations.csv      trial, outcome, contact_observed
#   run_info.txt          date, settings, servo gains; calibration file copied
# Summarise with:  python3 docs/analysis/summarize_runs.py docs/data/runs/<folder>

TARGET=${1:?usage: run_trials.sh <target_index> [n_trials]}
N=${2:-5}
HW=${HW:-real}
COMP=${COMP:-$([ "$HW" = real ] && echo true || echo false)}
SUFFIX=$([ "$COMP" = true ] && echo "_comp" || true)
PKG=$(cd "$(dirname "$0")/.." && pwd)
WS=$(cd "$PKG/../.." && pwd)
CAL="$WS/my_follower_joints.yaml"
OUT="${RUNS_DIR:-$PKG/docs/data/runs}/$(date +%Y%m%d_%H%M%S)_target${TARGET}_${HW}${SUFFIX}"
mkdir -p "$OUT"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -m   # each launch gets its own process group, so we can stop it like Ctrl+C

ask() {  # ask <prompt> -> reply in $REPLY; no prompts in mock mode
  if [ "$HW" = real ]; then read -r -p "$1" REPLY; else REPLY=""; fi
}

stack_running() { pgrep -f "controller_manager/ros2_control_node" >/dev/null; }

{
  echo "date:        $(date -Iseconds)"
  echo "target:      $TARGET"
  echo "trials:      $N"
  echo "hardware:    $HW"
  echo "gravity compensation: $COMP"
  echo "calibration: $CAL (copied here as calibration.yaml)"
  echo "servo P/I/D per joint:"
  python3 - "$CAL" <<'PY'
import sys, yaml
for name, j in yaml.safe_load(open(sys.argv[1]))["joints"].items():
    print(f"  {name:14s} P={j['p_coefficient']:3d}  I={j['i_coefficient']}  D={j['d_coefficient']}")
PY
} > "$OUT/run_info.txt"
cp "$CAL" "$OUT/calibration.yaml"
echo "trial,outcome,contact_observed" > "$OUT/observations.csv"
echo "Writing to $OUT"

for i in $(seq 1 "$N"); do
  echo; echo "=== Trial $i of $N (target $TARGET, $HW) ==="
  if stack_running; then
    echo "A ros2_control stack is already running. Stop it first (it holds the serial port)."
    exit 1
  fi
  ask "Put the arm in its resting pose, hands clear, then press Enter... "

  LOG="$OUT/trial${i}_launch.log"
  ros2 launch so101_pose_milestone pose_commander.launch.py \
    hardware_type:="$HW" target_index:="$TARGET" compensate:="$COMP" > "$LOG" 2>&1 &
  LPID=$!

  # Wait for the commander's verdict (it starts ~10 s after bringup).
  outcome=timeout
  for _ in $(seq 1 120); do
    if   grep -q "Execution returned" "$LOG"; then outcome=executed; break
    elif grep -q "IK FAILED"          "$LOG"; then outcome=ik_failed; break
    elif grep -q "PLANNING FAILED"    "$LOG"; then outcome=planning_failed; break
    elif ! kill -0 "$LPID" 2>/dev/null;    then outcome=launch_died; break
    fi
    sleep 1
  done
  echo "Outcome: $outcome"

  if [ "$outcome" = executed ]; then
    sleep 3   # let the servos settle before sampling
    (cd "$PKG/so101_pose_milestone" && python3 verify_pose.py --target "$TARGET") \
      > "$OUT/trial${i}_verify.txt" 2>&1
    grep -E "position error|orientation error|RESULT" "$OUT/trial${i}_verify.txt"
    ask "Is anything touching or pulling on the gripper or wrist (cable, table, object)? [y/N] "
    contact=${REPLY:-n}
  else
    contact=na
  fi
  [ "$HW" = real ] || contact=na
  echo "$i,$outcome,$contact" >> "$OUT/observations.csv"

  ask "SUPPORT THE ARM - torque is about to be released. Press Enter... "
  kill -INT -- "-$LPID" 2>/dev/null          # same as Ctrl+C on the whole launch
  for _ in $(seq 1 20); do kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
  kill -TERM -- "-$LPID" 2>/dev/null; wait "$LPID" 2>/dev/null
  for _ in $(seq 1 10); do stack_running || break; sleep 1; done
done

echo; echo "Done. Results in $OUT"
echo "Summarise:  python3 $PKG/docs/analysis/summarize_runs.py $OUT"
