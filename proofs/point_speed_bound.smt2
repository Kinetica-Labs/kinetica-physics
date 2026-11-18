; Лемма 6. Граница скорости точки тела: |v + ω × r| <= |v| + |ω|·|r|. На ней стоит поиск времени
; удара (maxPointSpeed, timeOfImpact).
; Обозначения: V = |v|, W = |ω|, R = |r|, C = |ω × r|, dot = v·(ω × r). Посылки — следствия леммы 5:
; C² <= W²·R² (для пары ω, r) и dot² <= V²·C² (для пары v, ω × r). Вывод: |v + ω × r|² =
; V² + 2·dot + C² <= (V + W·R)². Ответ unsat.
(declare-const V Real) (declare-const W Real) (declare-const R Real) (declare-const C Real) (declare-const dot Real)
(assert (and (>= V 0.0) (>= W 0.0) (>= R 0.0) (>= C 0.0)))
(assert (<= (* C C) (* W W R R)))
(assert (<= (* dot dot) (* V V C C)))
(assert (> (+ (* V V) (* 2.0 dot) (* C C)) (* (+ V (* W R)) (+ V (* W R)))))
(check-sat)
