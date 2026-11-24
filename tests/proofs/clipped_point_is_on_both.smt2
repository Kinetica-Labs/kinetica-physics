; Лемма 10. Точка отсечения. Отрезок from—to пересекает сторону грани там, где расстояние до
; стороны меняет знак: t = d0 / (d0 − d1), где d0 и d1 — расстояния концов (crossings). Расстояние
; до стороны линейно вдоль отрезка, поэтому в точке from + t·(to − from) оно равно нулю, а сама
; точка лежит на отрезке. Ответ unsat.
(declare-const d0 Real) (declare-const d1 Real) (declare-const t Real)
(assert (> d0 0.0)) (assert (< d1 0.0))
(assert (= (* t (- d0 d1)) d0))
(assert (or (not (= (+ d0 (* t (- d1 d0))) 0.0)) (< t 0.0) (> t 1.0)))
(check-sat)
