; Лемма 16. Трение не зависит от выбора касательных. По теореме о причёсывании ежа на сфере нет
; непрерывного поля ненулевых касательных векторов: какое бы правило ни выбирало два касательных
; направления по нормали n (ContactSolver.cpp, tangents), где-то на сфере нормалей оно делает
; скачок. Значит, ответ решателя обязан не зависеть от этого выбора — иначе при плавном повороте
; нормали сила трения прыгала бы.
; Скольжение в касательной плоскости — вектор s, отклик пары — симметричная матрица K, импульс
; трения x гасит скольжение: K x = −s. В касательных, повёрнутых на угол φ (c = cos φ, s = sin φ),
; те же величины — s' = R s, K' = R K Rᵀ. Тогда x' = R x удовлетворяет K' x' = −s', то есть это тот
; же вектор в мире, и его длина, которую ограничивает закон Кулона, та же: |x'|² = |x|².
; Ответ unsat.
(declare-const k11 Real) (declare-const k12 Real) (declare-const k22 Real)
(declare-const x1 Real) (declare-const x2 Real) (declare-const s1 Real) (declare-const s2 Real)
(declare-const c Real) (declare-const s Real)
; повёрнутые величины
(define-fun rx1 () Real (+ (* c x1) (* s x2)))
(define-fun rx2 () Real (- (* c x2) (* s x1)))
(define-fun rs1 () Real (+ (* c s1) (* s s2)))
(define-fun rs2 () Real (- (* c s2) (* s s1)))
; K' = R K Rᵀ
(define-fun q11 () Real (+ (* c c k11) (* 2.0 c s k12) (* s s k22)))
(define-fun q12 () Real (+ (* c s (- k22 k11)) (* (- (* c c) (* s s)) k12)))
(define-fun q22 () Real (+ (* s s k11) (* (- 2.0) c s k12) (* c c k22)))
(assert (= (+ (* c c) (* s s)) 1.0))
(assert (= (+ (* k11 x1) (* k12 x2)) (- s1)))
(assert (= (+ (* k12 x1) (* k22 x2)) (- s2)))
(assert (or (not (= (+ (* q11 rx1) (* q12 rx2)) (- rs1))) (not (= (+ (* q12 rx1) (* q22 rx2)) (- rs2))) (not (= (+ (* rx1 rx1) (* rx2 rx2)) (+ (* x1 x1) (* x2 x2))))))
(check-sat)
