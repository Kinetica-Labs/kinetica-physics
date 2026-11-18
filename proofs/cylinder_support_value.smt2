; Лемма 2а. Опорная функция цилиндра. Цилиндр — точки с x² + z² <= R², |y| <= H. В направлении
; d = (dx, dy, dz) ни одна его точка не дальше чем R·s + H·|dy|, где s = √(dx² + dz²).
; Ответ unsat.
(declare-const R Real) (declare-const H Real)
(declare-const dx Real) (declare-const dy Real) (declare-const dz Real) (declare-const s Real)
(declare-const x Real) (declare-const y Real) (declare-const z Real)
(assert (> R 0.0)) (assert (> H 0.0))
(assert (>= s 0.0)) (assert (= (* s s) (+ (* dx dx) (* dz dz))))
(assert (<= (+ (* x x) (* z z)) (* R R))) (assert (<= y H)) (assert (>= y (- H)))
(assert (> (+ (* dx x) (* dy y) (* dz z)) (+ (* R s) (* H (ite (>= dy 0.0) dy (- dy))))))
(check-sat)
