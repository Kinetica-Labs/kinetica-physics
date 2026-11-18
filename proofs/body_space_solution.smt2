; Лемма 19. Решение в пространстве тел (ContactSolver::solveInBodySpace) — решение исходной
; системы (P + J W Jᵀ) x = b. Тождество Вудбери проверяется на наименьшем случае, где линий больше,
; чем свобод: две линии j1, j2 на одну свободу с обратной массой w > 0, P = diag(p1, p2) > 0.
; Скорость тела y находится из K y = Jᵀ P⁻¹ b, K = 1/w + j1²/p1 + j2²/p2 (здесь обе части умножены
; на w p1 p2), импульсы — x_i = (b_i − j_i y) / p_i. Утверждение: p_i x_i + j_i w (j1 x1 + j2 x2) = b_i
; для обеих линий. Общий случай — та же выкладка с матрицами. Ответ unsat.
(declare-const p1 Real) (declare-const p2 Real) (declare-const w Real)
(declare-const j1 Real) (declare-const j2 Real) (declare-const b1 Real) (declare-const b2 Real)
(declare-const y Real) (declare-const x1 Real) (declare-const x2 Real)
(assert (and (> p1 0.0) (> p2 0.0) (> w 0.0)))
(assert (= (* (+ (* p1 p2) (* w j1 j1 p2) (* w j2 j2 p1)) y) (* w (+ (* j1 b1 p2) (* j2 b2 p1)))))
(assert (and (= (* p1 x1) (- b1 (* j1 y))) (= (* p2 x2) (- b2 (* j2 y)))))
(assert (not (and (= (+ (* p1 x1) (* j1 w (+ (* j1 x1) (* j2 x2)))) b1) (= (+ (* p2 x2) (* j2 w (+ (* j1 x1) (* j2 x2)))) b2))))
(check-sat)
