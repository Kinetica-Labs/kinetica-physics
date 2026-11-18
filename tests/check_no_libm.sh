#!/bin/sh
# Правило детерминизма: ядро не вызывает трансцендентные функции стандартной библиотеки
# (math/math_functions.h). Проверяется по неразрешённым символам собранной библиотеки.
#   check_no_libm.sh путь/к/libkinetica-physics.a
found=$(nm -u "$1" | grep -wE '(sin|cos|tan|asin|acos|atan|atan2|sinh|cosh|tanh|exp|exp2|log|log2|log10|pow|cbrt|hypot)f?l?' || true)
if [ -n "$found" ]; then
    echo "в ядре есть вызовы libm:"; echo "$found"; exit 1
fi
echo "вызовов libm в ядре нет"
