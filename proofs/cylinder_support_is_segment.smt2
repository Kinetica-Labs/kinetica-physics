; Лемма 2в. Поперёк оси (dy = 0, s > 0) цилиндр упирается образующей: у каждой опорной точки
; (x, z) = R·(dx, dz)/s, а y любое. Опорная точка вне этой образующей невозможна. Ответ unsat.
(declare-const R Real) (declare-const H Real)
(declare-const dx Real) (declare-const dz Real) (declare-const s Real)
(declare-const x Real) (declare-const y Real) (declare-const z Real)
(assert (> R 0.0)) (assert (> H 0.0))
(assert (> s 0.0)) (assert (= (* s s) (+ (* dx dx) (* dz dz))))
(assert (<= (+ (* x x) (* z z)) (* R R))) (assert (<= y H)) (assert (>= y (- H)))
(assert (= (+ (* dx x) (* dz z)) (* R s)))
(assert (or (not (= (* x s) (* R dx))) (not (= (* z s) (* R dz)))))
(check-sat)
