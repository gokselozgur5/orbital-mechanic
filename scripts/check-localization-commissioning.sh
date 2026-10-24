#!/usr/bin/env bash
# Thor-only simulator reset, deliberately offset initial pose and service lifecycle.
# This is commissioning evidence, not autonomous-navigation acceptance.
set -eo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
[[ -n ${TMUX:-} && $ROOT == /home/jensen/.goske/codex/orbital-mechanic ]] || {
  echo 'Run inside Thor tmux orbital-work in the simulation workspace.' >&2; exit 1;
}
cd "$ROOT"
source scripts/ros-env.sh
report="$ROOT/runtime/localization-acceptance-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir "$report"
/usr/bin/python3 - "$report" <<'PY'
import hashlib,json,sys
from pathlib import Path
root=Path.cwd()
files=sorted([*root.glob('scripts/*'),*root.glob('config/*'),*root.glob('orbital/*.py'),*root.glob('worlds/*.sdf')])
Path(sys.argv[1],'source-sha256.json').write_text(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()},indent=2)+'\n')
PY
cleanup() {
  result=$?
  if (( result != 0 )); then
    timeout 8 /usr/bin/python3 scripts/orbital-operator.py stop || bash scripts/stop-thor.sh || true
  fi
}
trap cleanup EXIT
if [[ $(systemctl --user show orbital-demo.service --property=LoadState --value) != not-found ]]; then
  bash scripts/stop-thor.sh
fi
bash scripts/launch-thor.sh
sleep 12
bash scripts/launch-localization.sh
sleep 15
systemctl --user is-active orbital-localization.service
timeout 20 /usr/bin/python3 scripts/check-localization.py --expect-unseeded > "$report/unseeded.json"
# Never send evaluator ground truth to AMCL; seed known origin with a deliberate error.
timeout 20 /usr/bin/python3 scripts/seed-localization.py --fresh-world-origin \
  --seed-offset-x .2 --seed-offset-y -.15 --seed-offset-yaw .15 > "$report/offset-seed.json"
/usr/bin/python3 scripts/orbital-operator.py arm > "$report/arm-forward.json"
/usr/bin/python3 scripts/orbital-operator.py jog forward --duration 1 > "$report/forward.json"
/usr/bin/python3 scripts/orbital-operator.py arm > "$report/arm-left.json"
/usr/bin/python3 scripts/orbital-operator.py jog left --duration .7 > "$report/left.json"
timeout 25 /usr/bin/python3 scripts/check-localization.py --compare-gazebo > "$report/offset-convergence.json"
timeout 100 /usr/bin/python3 scripts/check-localization-lifecycle.py > "$report/lifecycle.json"
printf 'Localization commissioning passed. Evidence: %s\nLive target/reset/loss supervision remains a separate gate.\n' "$report"
