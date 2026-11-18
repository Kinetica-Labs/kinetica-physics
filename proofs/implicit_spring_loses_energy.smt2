; Лемма 21. Пружина с демпфером в сочленении, посчитанные неявно, не добавляют энергии ни при какой
; жёсткости и ни при каком шаге. Груз массы m на пружине k с демпфером c; шаг h: сила берётся по
; положению и скорости в конце шага (ContactSolver.cpp, податливая линия):
;   m (v1 − v0) = −h (k x1 + c v1),   x1 = x0 + h v1.
; Тогда энергия E = m v² / 2 + k x² / 2 убывает ровно на
;   m (v0 − v1)² / 2 + k (x1 − x0)² / 2 + h c v1²  >= 0.
; Проверяется само равенство; d — изменение скорости v0 − v1. Ответ unsat.
(declare-const m Real) (declare-const k Real) (declare-const c Real) (declare-const h Real)
(declare-const v1 Real) (declare-const x1 Real) (declare-const d Real)
(define-fun v0 () Real (+ v1 d))
(define-fun x0 () Real (- x1 (* h v1)))
(define-fun energy ((v Real) (x Real)) Real (+ (/ (* m v v) 2.0) (/ (* k x x) 2.0)))
(assert (and (> m 0.0) (> h 0.0) (>= k 0.0) (>= c 0.0) (= (* m d) (* h (+ (* k x1) (* c v1))))))
(assert (not (= (- (energy v0 x0) (energy v1 x1)) (+ (/ (* m d d) 2.0) (/ (* k h h v1 v1) 2.0) (* h c v1 v1)))))
(check-sat)
