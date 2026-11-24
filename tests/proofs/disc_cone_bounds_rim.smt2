; Лемма 9. Точное условие для круга. Скорость сближения на круге радиуса R линейна:
; f = α + β·x + γ·y. Если α >= R·√(β² + γ²), то f >= 0 в каждой точке круга: вместо точек на
; окружности достаточно одного условия-конуса в центре. Ответ unsat.
(declare-const alpha Real) (declare-const beta Real) (declare-const gamma Real)
(declare-const R Real) (declare-const g Real) (declare-const x Real) (declare-const y Real)
(assert (> R 0.0))
(assert (and (>= g 0.0) (= (* g g) (+ (* beta beta) (* gamma gamma)))))
(assert (>= alpha (* R g)))
(assert (<= (+ (* x x) (* y y)) (* R R)))
(assert (< (+ alpha (* beta x) (* gamma y)) 0.0))
(check-sat)
