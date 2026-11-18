; Лемма 4. Пересечение окружности торца со стороной грани (patchPoints). Точка окружности —
; c + r·(u cos φ + v sin φ); плоскость стороны даёт уравнение A cos φ + B sin φ = C. Формулы
;   cos φ = (A·C − σ·B·w) / (A² + B²),  sin φ = (B·C + σ·A·w) / (A² + B²),  w = √(A² + B² − C²)
; дают настоящий угол (cos² + sin² = 1), и он удовлетворяет уравнению. Ответ unsat.
(declare-const A Real) (declare-const B Real) (declare-const C Real) (declare-const w Real)
(declare-const sign Real) (declare-const cosine Real) (declare-const sine Real)
(define-fun reach () Real (+ (* A A) (* B B)))
(assert (> reach 0.0))
(assert (>= w 0.0)) (assert (= (* w w) (- reach (* C C))))
(assert (or (= sign 1.0) (= sign (- 1.0))))
(assert (= (* cosine reach) (- (* A C) (* sign B w))))
(assert (= (* sine reach) (+ (* B C) (* sign A w))))
(assert (or (not (= (+ (* cosine cosine) (* sine sine)) 1.0)) (not (= (+ (* A cosine) (* B sine)) C))))
(check-sat)
