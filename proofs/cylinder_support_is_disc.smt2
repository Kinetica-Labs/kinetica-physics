; Лемма 2г. Вдоль оси (s = 0, dy > 0) цилиндр упирается всем кругом торца: опорные точки — ровно
; точки с y = H. Опорная точка с другим y или точка торца, не являющаяся опорной, невозможны.
; Ответ unsat.
(declare-const R Real) (declare-const H Real) (declare-const dy Real)
(declare-const x Real) (declare-const y Real) (declare-const z Real)
(assert (> R 0.0)) (assert (> H 0.0)) (assert (> dy 0.0))
(assert (<= (+ (* x x) (* z z)) (* R R))) (assert (<= y H)) (assert (>= y (- H)))
(assert (not (= (= (* dy y) (* H dy)) (= y H))))
(check-sat)
