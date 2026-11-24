; Лемма 14. Линия связи шарнира — производная её нарушения. Нарушение C = x_b · u: главная ось
; тела b обязана оставаться перпендикулярной оси u тела a. Оси поворачиваются вместе с телами:
; dx_b/dt = ω_b × x_b, du/dt = ω_a × u, поэтому dC/dt = (ω_b × x_b) · u + x_b · (ω_a × u). Линия связи
; в коде: (ω_b − ω_a) · (x_b × u). Эти выражения равны тождественно. Ответ unsat.
(declare-const ax Real) (declare-const ay Real) (declare-const az Real)
(declare-const ux Real) (declare-const uy Real) (declare-const uz Real)
(declare-const wax Real) (declare-const way Real) (declare-const waz Real)
(declare-const wbx Real) (declare-const wby Real) (declare-const wbz Real)
(define-fun cx ((px Real) (py Real) (pz Real) (qx Real) (qy Real) (qz Real)) Real (- (* py qz) (* pz qy)))
(define-fun cy ((px Real) (py Real) (pz Real) (qx Real) (qy Real) (qz Real)) Real (- (* pz qx) (* px qz)))
(define-fun cz ((px Real) (py Real) (pz Real) (qx Real) (qy Real) (qz Real)) Real (- (* px qy) (* py qx)))
(define-fun dot ((px Real) (py Real) (pz Real) (qx Real) (qy Real) (qz Real)) Real (+ (* px qx) (* py qy) (* pz qz)))
(define-fun derivative () Real (+ (dot (cx wbx wby wbz ax ay az) (cy wbx wby wbz ax ay az) (cz wbx wby wbz ax ay az) ux uy uz) (dot ax ay az (cx wax way waz ux uy uz) (cy wax way waz ux uy uz) (cz wax way waz ux uy uz))))
(define-fun line () Real (dot (- wbx wax) (- wby way) (- wbz waz) (cx ax ay az ux uy uz) (cy ax ay az ux uy uz) (cz ax ay az ux uy uz)))
(assert (not (= derivative line)))
(check-sat)
