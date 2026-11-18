; Лемма 17. Скорость угла шарнира — разность угловых скоростей тел вдоль его оси. Угол θ отсчитан
; от оси y тела a до оси y тела b вокруг общей оси e (Joint.cpp, coordinateOf): cos θ ~ c = y_b·y_a,
; sin θ ~ s = y_b·z_a, и dθ/dt = (c·ds/dt − s·dc/dt) / (c² + s²). Оси тел вращаются с телами:
; dy_b/dt = ω_b × y_b, dy_a/dt = ω_a × y_a, dz_a/dt = ω_a × z_a. Утверждение:
; c·ds/dt − s·dc/dt = (c² + s²)·e·(ω_b − ω_a), то есть привод, держащий скорость вдоль линии связи
; (0, e, e), держит именно скорость координаты, которую отдаёт coordinateOf. Без потери общности
; e, y_a, z_a — координатные оси, а y_b = (0, c, s): шарнир держит ось тела b вдоль e, и y_b ей
; перпендикулярна. Ответ unsat.
(declare-const c Real) (declare-const s Real)
(declare-const wax Real) (declare-const way Real) (declare-const waz Real)
(declare-const wbx Real) (declare-const wby Real) (declare-const wbz Real)
(define-fun crossx ((ux Real) (uy Real) (uz Real) (vx Real) (vy Real) (vz Real)) Real (- (* uy vz) (* uz vy)))
(define-fun crossy ((ux Real) (uy Real) (uz Real) (vx Real) (vy Real) (vz Real)) Real (- (* uz vx) (* ux vz)))
(define-fun crossz ((ux Real) (uy Real) (uz Real) (vx Real) (vy Real) (vz Real)) Real (- (* ux vy) (* uy vx)))
(define-fun dot ((ux Real) (uy Real) (uz Real) (vx Real) (vy Real) (vz Real)) Real (+ (* ux vx) (* uy vy) (* uz vz)))
; dc/dt = (ω_b × y_b)·y_a + y_b·(ω_a × y_a),  ds/dt = (ω_b × y_b)·z_a + y_b·(ω_a × z_a)
(define-fun cdot () Real (+ (dot (crossx wbx wby wbz 0.0 c s) (crossy wbx wby wbz 0.0 c s) (crossz wbx wby wbz 0.0 c s) 0.0 1.0 0.0) (dot 0.0 c s (crossx wax way waz 0.0 1.0 0.0) (crossy wax way waz 0.0 1.0 0.0) (crossz wax way waz 0.0 1.0 0.0))))
(define-fun sdot () Real (+ (dot (crossx wbx wby wbz 0.0 c s) (crossy wbx wby wbz 0.0 c s) (crossz wbx wby wbz 0.0 c s) 0.0 0.0 1.0) (dot 0.0 c s (crossx wax way waz 0.0 0.0 1.0) (crossy wax way waz 0.0 0.0 1.0) (crossz wax way waz 0.0 0.0 1.0))))
(assert (not (= (- (* c sdot) (* s cdot)) (* (+ (* c c) (* s s)) (- wbx wax)))))
(check-sat)
