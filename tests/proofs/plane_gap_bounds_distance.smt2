; Лемма 1. Зазор плоскости не больше расстояния между телами.
; Тело A лежит за плоскостью (n, o), каждая точка тела B — не ниже g над ней. Тогда любые две
; точки a из A и b из B разделены не меньше чем g. На этом стоит выбор плоского элемента в
; patchManifold (engine/geometry/ContactManifold.cpp): g <= d, равенство — касание на самом элементе.
; Отрицание утверждения обязано быть невыполнимым: ответ unsat.
(declare-const ax Real) (declare-const ay Real) (declare-const az Real)
(declare-const bx Real) (declare-const by Real) (declare-const bz Real)
(declare-const nx Real) (declare-const ny Real) (declare-const nz Real)
(declare-const ox Real) (declare-const oy Real) (declare-const oz Real)
(declare-const g Real)
(assert (= (+ (* nx nx) (* ny ny) (* nz nz)) 1.0))
(assert (>= g 0.0))
(assert (<= (+ (* nx (- ax ox)) (* ny (- ay oy)) (* nz (- az oz))) 0.0))
(assert (>= (+ (* nx (- bx ox)) (* ny (- by oy)) (* nz (- bz oz))) g))
(assert (< (+ (* (- bx ax) (- bx ax)) (* (- by ay) (- by ay)) (* (- bz az) (- bz az))) (* g g)))
(check-sat)
