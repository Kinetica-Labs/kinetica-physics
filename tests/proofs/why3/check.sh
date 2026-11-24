#!/bin/sh
# Проверяет модели Why3 (*.mlw) решателями Z3 и Alt-Ergo. Модель — программа на WhyML, повторяющая
# функцию движка, с договором (requires / ensures) и инвариантами циклов; Why3 превращает их в
# условия проверки. Условие считается доказанным, если его доказал хотя бы один решатель.
cd "$(dirname "$0")" || exit 2
command -v why3 > /dev/null || eval "$(opam env --switch=default 2> /dev/null)"
command -v why3 > /dev/null || { echo "why3 не найден (opam, переключатель default)"; exit 2; }
work=$(mktemp -d) || exit 2
for model in *.mlw; do # все модели и оба решателя — разом: время проверки равно самому долгому прогону
    for prover in Z3 Alt-Ergo; do
        why3 prove -P "$prover,," -a split_vc -t 3 "$model" 2> /dev/null | grep "Prover result is" | sed 's/ (.*//' > "$work/$model.$prover" &
    done
done
wait
failed=0
for model in *.mlw; do
    total=$(wc -l < "$work/$model.Z3")
    proved=$(paste "$work/$model.Z3" "$work/$model.Alt-Ergo" | grep -c "Valid")
    printf 'модель %-28s условий проверки: %3d  доказано: %3d\n' "$model" "$total" "$proved"
    [ "$total" -gt 0 ] && [ "$proved" = "$total" ] || failed=1
done
rm -rf "$work"
exit $failed
