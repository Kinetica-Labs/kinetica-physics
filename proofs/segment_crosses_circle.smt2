; Лемма 3. Пересечение отрезка с окружностью торца (crossings). В плоскости круга точка отрезка —
; q + t·d. При a = d·d > 0, b = q·d, c = q·q − R² и t = (−b ± √(b² − a·c)) / a точка лежит на
; окружности: |q + t·d|² = R². Ответ unsat.
(declare-const qx Real) (declare-const qy Real) (declare-const ddx Real) (declare-const ddy Real)
(declare-const R Real) (declare-const root Real) (declare-const t Real) (declare-const sign Real)
(define-fun a () Real (+ (* ddx ddx) (* ddy ddy)))
(define-fun b () Real (+ (* qx ddx) (* qy ddy)))
(define-fun c () Real (- (+ (* qx qx) (* qy qy)) (* R R)))
(assert (> a 0.0))
(assert (>= root 0.0)) (assert (= (* root root) (- (* b b) (* a c))))
(assert (or (= sign 1.0) (= sign (- 1.0))))
(assert (= (* t a) (+ (- b) (* sign root))))
(assert (not (= (+ (* (+ qx (* t ddx)) (+ qx (* t ddx))) (* (+ qy (* t ddy)) (+ qy (* t ddy)))) (* R R))))
(check-sat)
