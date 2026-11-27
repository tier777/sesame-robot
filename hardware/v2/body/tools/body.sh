#!/usr/bin/env bash
# Рама v2 целиком: записи поз прошивки → топооптимизация стенки → рама и крышка отсека (с проверками) →
# двойник на v2 → ворота «нет новых упоров против v1» → картинки и плита.
#   bash hardware/v2/body/tools/body.sh
set -euo pipefail
cd "$(dirname "$0")/../.."
step() {
  echo "== $1"
  shift
  if ! "$@"; then echo "сломался шаг: $*" >&2; exit 1; fi
}
step "записи поз прошивки" .venv/bin/python -c "from twin.firmware_trace.trace import build_traces; from twin import params as P; r = build_traces(P.HERE.parents[2] / 'firmware', P.HERE / 'traces'); print(len(r.traces), 'поз', ('не записались: ' + ', '.join(r.failed)) if r.failed else '')"
step "топооптимизация стенки отсека" .venv/bin/python -c "from body.topopt import wall_density; wall_density()"
step "рама и крышка отсека" .venv/bin/python -m body.frame_v2
step "двойник на v2" bash tools/twin.sh v2
step "новых упоров против v1 нет" .venv/bin/python -m body.gate
step "картинки и плита" .venv/bin/python -m body.render
