#!/bin/sh
# Проверяет леммы решателем Z3.
#
# Лемма (*.smt2) — посылки и отрицание её вывода последним утверждением (assert). Лемма доказана,
# когда вместе они невыполнимы (unsat). Невыполнимость была бы пустой, если бы противоречили друг
# другу сами посылки, поэтому та же задача без последнего утверждения обязана быть выполнимой (sat).
#
# Дыра (counterexamples/*.smt2) — утверждение, что гарантии нет: задача описывает нарушение и
# обязана быть выполнимой (sat). Дыра остаётся в каталоге, пока алгоритм её не закрыл.
cd "$(dirname "$0")" || exit 2
failed=0
for lemma in *.smt2; do
    answers=$(z3 -T:60 "$lemma" 2>&1) || failed=1
    proof=$(printf '%s\n' "$answers" | head -1)
    # Именованный запрос печатает «имя expected sat/unsat» перед ответом.
    printf '%s\n' "$answers" | awk '
        pending { if ($0 != expected) { print label ": " $0; bad = 1 }; pending = 0; next }
        / expected (sat|unsat)$/ { label = $0; expected = $NF; pending = 1 }
        END { exit (bad || pending) }
    ' || failed=1
    last=$(grep -n '^(assert' "$lemma" | tail -1 | cut -d: -f1)
    premises=$(sed "${last}d" "$lemma" | z3 -T:60 -in 2>&1 | head -1)
    printf 'лемма  %-44s вывод: %-6s посылки: %s\n' "$lemma" "$proof" "$premises"
    [ "$proof" = unsat ] && [ "$premises" = sat ] || failed=1
done
for hole in counterexamples/*.smt2; do
    answer=$(z3 -T:60 "$hole" 2>&1 | head -1)
    printf 'дыра   %-44s нарушение существует: %s\n' "$(basename "$hole")" "$answer"
    [ "$answer" = sat ] || failed=1
done
exit $failed
