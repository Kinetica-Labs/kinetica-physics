#!/bin/bash
# Попеременный замер двух сборок: validation/compare_cycles.sh <было> <стало> [пар]
#
# Каждая пара — прогон «было», затем «стало», на закреплённом ядре; печатается наименьшее число тактов
# пользователя, число инструкций и время кадра у каждой сборки. Наименьшее, а не среднее: помеха от
# соседних программ, частоты и нагрева такты только прибавляет (один и тот же исполняемый подряд даёт
# от 10,2 до 12,2 млрд). Сборка против самой себя даёт отношение 1,000 — различимо около 1 %.
# Окружение: BENCH (сторона пирамиды, 14), KP_THREADS (1), ARGS (аргументы программы, «realtime 8»),
# CORE (номер ядра, 3). Нужен perf: sudo sysctl kernel.perf_event_paranoid=1.
set -u
before=$1; after=$2; pairs=${3:-6}
one() {
  KP_THREADS=${KP_THREADS:-1} BENCH=${BENCH:-14} taskset -c "${CORE:-3}" perf stat -x, -e cycles:u,instructions:u "$1" ${ARGS:-realtime 8} 2>&1 |
    awk -F, '/мс на кадр/ {match($0, /[0-9.]+ мс на кадр/); ms = substr($0, RSTART, RLENGTH) + 0}
             /cycles/ {c = $1} /instructions/ {i = $1} END {printf "%.0f %.0f %.2f\n", c, i, ms}'
}
for round in $(seq "$pairs"); do echo "A $(one "$before")"; echo "B $(one "$after")"; done | awk '
  {cycles[$1][++count[$1]] = $2; instructions[$1] = $3
   if (!($1 in least) || $2 < least[$1]) least[$1] = $2
   if (!($1 in frame) || $4 < frame[$1]) frame[$1] = $4}
  END {for (r = 1; r <= count["A"]; ++r) ratios = ratios sprintf(" %.3f", cycles["B"][r] / cycles["A"][r])
       printf "было:  тактов %.3f млрд (наименьшее), инструкций %.3f млрд, кадр %.2f мс\n", least["A"] / 1e9, instructions["A"] / 1e9, frame["A"]
       printf "стало: тактов %.3f млрд (наименьшее), инструкций %.3f млрд, кадр %.2f мс\n", least["B"] / 1e9, instructions["B"] / 1e9, frame["B"]
       printf "отношение стало/было: по наименьшим %.3f; по парам:%s\n", least["B"] / least["A"], ratios}'
