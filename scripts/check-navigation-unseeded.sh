#!/usr/bin/env bash
# Explicit destructive-to-localization commissioning; does not reset the world.
# Leaves a fresh unseeded localizer and configured, latched navigation on success.
set -eo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
[[ -n ${TMUX:-} && $ROOT == /home/jensen/.goske/codex/orbital-mechanic ]] || {
  echo 'Run inside Thor tmux orbital-work from the Orbital workspace.' >&2; exit 1;
}
[[ ${1:-} == --restart-localization ]] || {
  echo 'Explicit --restart-localization acknowledgment required.' >&2; exit 1;
}
cd "$ROOT"
source scripts/ros-env.sh
demo_id=$(systemctl --user show orbital-demo.service -p InvocationID --value)
systemctl --user is-active --quiet orbital-demo.service
[[ -n $demo_id ]]
result="$ROOT/runtime/unseeded-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$result"
echo "$result"
success=false
stop_navigation() {
  if [[ $(systemctl --user show orbital-navigation.service -p LoadState --value) != not-found ]]; then
    systemctl --user stop orbital-navigation.service
  fi
}
cleanup() {
  if [[ $success != true ]]; then
    # The independent guard also latches on producer loss. Own only this unit.
    stop_navigation || true
  fi
}
trap cleanup EXIT
wait_absent() {
  local unit=$1
  for attempt in {1..100}; do
    [[ $(systemctl --user show "$unit" -p LoadState --value) == not-found ]] && return 0
    sleep .1
  done
  echo "Unit did not finish cleanup: $unit" >&2
  return 1
}
stop_navigation
wait_absent orbital-navigation.service
# This graph gate also proves fresh latched/stationary feedback before restart.
python3 scripts/check-navigation-graph.py --phase before > "$result/before.json"
systemctl --user stop orbital-localization.service
wait_absent orbital-localization.service
bash scripts/launch-localization.sh
# The existing frame preflight takes several seconds before AMCL starts.
for attempt in {1..150}; do
  if ros2 service list | grep -qx /amcl/get_state; then break; fi
  systemctl --user is-active --quiet orbital-localization.service
  sleep .2
done
python3 scripts/check-localization.py --expect-unseeded > "$result/localization.json"
python3 scripts/launch-navigation.py
for attempt in {1..150}; do
  if ros2 service list | grep -qx /controller_server/get_state; then break; fi
  systemctl --user is-active --quiet orbital-navigation.service
  sleep .2
done
# Global costmap activation needs map->base TF. Exercise the real supervisor's
# admission gate with native action servers configured while AMCL is unseeded.
python3 scripts/check-navigation-ready.py --configure > "$result/readiness.json"
gz model -m maintenance_robot -p > "$result/pose-before.txt"
python3 scripts/check-navigation-unseeded.py > "$result/refusal.json"
gz model -m maintenance_robot -p > "$result/pose-after.txt"
python3 - "$result" <<'PY'
import json, math, re, sys
from pathlib import Path
root = Path(sys.argv[1])
def pose(name):
    text = re.sub(r'\x1b\[[0-9;]*m', '', (root/name).read_text())
    match = re.search(r'Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\[([^\]]+)\]\s*\[([^\]]+)\]', text)
    if not match: raise RuntimeError('Gazebo pose unavailable')
    return [float(x) for x in (match[1]+' '+match[2]).split()]
before, after = pose('pose-before.txt'), pose('pose-after.txt')
translation = math.dist(before[:3], after[:3])
rotation = max(abs(math.remainder(a-b, 2*math.pi)) for a,b in zip(after[3:], before[3:]))
passed = all(math.isfinite(x) for x in before+after) and translation <= .001 and rotation <= .001
(root/'stationarity.json').write_text(json.dumps(dict(passed=passed, translation_m=translation,
    rotation_rad=rotation, scope='Gazebo endpoint comparison, not continuous trajectory evidence'), indent=2))
if not passed: raise RuntimeError('Physical pose changed during refused commands')
PY
[[ $(systemctl --user show orbital-demo.service -p InvocationID --value) == "$demo_id" ]]
printf '%s\n' "$demo_id" > "$result/demo-invocation-id"
cp runtime/navigation/source-manifest.json "$result/source-manifest.json"
python3 scripts/navigation-manifest.py --verify "$result/source-manifest.json" > "$result/source-verification.json"
success=true
printf 'passed\n' > "$result/result"
