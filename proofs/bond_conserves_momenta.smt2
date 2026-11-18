; Лемма 12. Связь сочленения не меняет импульс и момент импульса пары. Импульс λ вдоль связи,
; запирающей перемещение по оси e в точке P: тело b получает импульс λ·e и момент (P − x_b) × λe
; относительно своего центра, тело a — импульс −λ·e и момент −(P − x_a) × λe (Joint.cpp, bondsOf).
; Сумма импульсов нулевая; сумма моментов импульса относительно начала координат,
; x_b × λe + (P − x_b) × λe − x_a × λe − (P − x_a) × λe, тоже нулевая. Проверяется первая
; составляющая суммы моментов; две другие получаются перестановкой осей. Ответ unsat.
(declare-const ex Real) (declare-const ey Real) (declare-const ez Real) (declare-const l Real)
(declare-const px Real) (declare-const py Real) (declare-const pz Real)
(declare-const ax Real) (declare-const ay Real) (declare-const az Real)
(declare-const bx Real) (declare-const by Real) (declare-const bz Real)
(define-fun crossx ((uy Real) (uz Real) (vy Real) (vz Real)) Real (- (* uy vz) (* uz vy)))
(assert (not (= (+ (crossx by bz (* l ey) (* l ez)) (crossx (- py by) (- pz bz) (* l ey) (* l ez)) (- (crossx ay az (* l ey) (* l ez))) (- (crossx (- py ay) (- pz az) (* l ey) (* l ez)))) 0.0)))
(check-sat)
