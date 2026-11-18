; Лемма 18. Вторая производная угла шарнира. Скорость угла — q' = e·(ω_b − ω_a) (лемма 17), ось e
; вращается с телом a: de/dt = ω_a × e. Поэтому q'' = (ω_a × e)·(ω_b − ω_a) + e·(α_b − α_a), и
; первое слагаемое равно (ω_a × e)·ω_b: вектор ω_a × e перпендикулярен ω_a. Отсюда граница,
; по которой World::earliestLimit ищет момент предела хода: |q''| <= |ω_a| |ω_b| + |α_a| + |α_b|
; (длина векторного произведения — лемма 4). Проверяется равенство двух записей первого
; слагаемого. Ответ unsat.
(declare-const ex Real) (declare-const ey Real) (declare-const ez Real)
(declare-const ax Real) (declare-const ay Real) (declare-const az Real)
(declare-const bx Real) (declare-const by Real) (declare-const bz Real)
(define-fun cx () Real (- (* ay ez) (* az ey)))
(define-fun cy () Real (- (* az ex) (* ax ez)))
(define-fun cz () Real (- (* ax ey) (* ay ex)))
(assert (not (= (+ (* cx (- bx ax)) (* cy (- by ay)) (* cz (- bz az))) (+ (* cx bx) (* cy by) (* cz bz)))))
(check-sat)
