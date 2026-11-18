; Лемма 5. Тождество Лагранжа: (a·b)² + |a × b|² = |a|²·|b|². Из него неравенство Коши —
; Буняковского (a·b)² <= |a|²·|b|² и граница |a × b|² <= |a|²·|b|²: оба слагаемых слева
; неотрицательны. Леммы 6–8 пользуются этими двумя следствиями как посылками. Ответ unsat.
(declare-const ax Real) (declare-const ay Real) (declare-const az Real)
(declare-const bx Real) (declare-const by Real) (declare-const bz Real)
(define-fun dot () Real (+ (* ax bx) (* ay by) (* az bz)))
(define-fun cx () Real (- (* ay bz) (* az by)))
(define-fun cy () Real (- (* az bx) (* ax bz)))
(define-fun cz () Real (- (* ax by) (* ay bx)))
(assert (not (= (+ (* dot dot) (* cx cx) (* cy cy) (* cz cz)) (* (+ (* ax ax) (* ay ay) (* az az)) (+ (* bx bx) (* by by) (* bz bz))))))
(check-sat)
