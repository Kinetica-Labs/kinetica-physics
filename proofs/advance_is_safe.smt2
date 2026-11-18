; Лемма 7. Продвижение безопасно. Две точки разделены вектором u, |u| = U >= D; их взаимное
; смещение w по величине N <= r, и r <= D. Тогда после смещения они разделены не меньше чем на
; D − r. Поэтому тела на расстоянии d, сближающиеся не быстрее c, не коснутся раньше чем через
; d / c — на этом стоит консервативное продвижение (timeOfImpact).
; Посылка dot² <= U²·N² для dot = u·w — следствие леммы 5. Вывод: |u + w|² = U² + 2·dot + N² >=
; (D − r)². Ответ unsat.
(declare-const U Real) (declare-const N Real) (declare-const D Real) (declare-const r Real) (declare-const dot Real)
(assert (and (>= N 0.0) (<= N r) (<= r D) (<= D U)))
(assert (<= (* dot dot) (* U U N N)))
(assert (< (+ (* U U) (* 2.0 dot) (* N N)) (* (- D r) (- D r))))
(check-sat)
