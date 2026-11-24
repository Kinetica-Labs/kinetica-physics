; Лемма 13. Линия связи перемещения — производная её нарушения. Нарушение C = e · (p_b − p_a),
; где p_a, p_b — точки сочленения двух тел, e — ось тела a. Тела движутся как твёрдые:
; dp_a/dt = v_a + ω_a × (p_a − x_a), dp_b/dt = v_b + ω_b × (p_b − x_b), de/dt = ω_a × e. По правилу
; производной произведения dC/dt = e · (dp_b/dt − dp_a/dt) + (de/dt) · (p_b − p_a). Линия связи в
; коде: e · (v_b − v_a) + ((p_b − x_b) × e) · ω_b − ((p_b − x_a) × e) · ω_a — у обоих тел плечо до
; одной и той же точки p_b. Эти два выражения равны тождественно. Ответ unsat.
(declare-const ex Real) (declare-const ey Real) (declare-const ez Real)
(declare-const pax Real) (declare-const pay Real) (declare-const paz Real)
(declare-const pbx Real) (declare-const pby Real) (declare-const pbz Real)
(declare-const xax Real) (declare-const xay Real) (declare-const xaz Real)
(declare-const xbx Real) (declare-const xby Real) (declare-const xbz Real)
(declare-const vax Real) (declare-const vay Real) (declare-const vaz Real)
(declare-const vbx Real) (declare-const vby Real) (declare-const vbz Real)
(declare-const wax Real) (declare-const way Real) (declare-const waz Real)
(declare-const wbx Real) (declare-const wby Real) (declare-const wbz Real)
(define-fun cx ((ux Real) (uy Real) (uz Real) (vx Real) (vy Real) (vz Real)) Real (- (* uy vz) (* uz vy)))
(define-fun cy ((ux Real) (uy Real) (uz Real) (vx Real) (vy Real) (vz Real)) Real (- (* uz vx) (* ux vz)))
(define-fun cz ((ux Real) (uy Real) (uz Real) (vx Real) (vy Real) (vz Real)) Real (- (* ux vy) (* uy vx)))
(define-fun dot ((ux Real) (uy Real) (uz Real) (vx Real) (vy Real) (vz Real)) Real (+ (* ux vx) (* uy vy) (* uz vz)))
(define-fun dpax () Real (+ vax (cx wax way waz (- pax xax) (- pay xay) (- paz xaz))))
(define-fun dpay () Real (+ vay (cy wax way waz (- pax xax) (- pay xay) (- paz xaz))))
(define-fun dpaz () Real (+ vaz (cz wax way waz (- pax xax) (- pay xay) (- paz xaz))))
(define-fun dpbx () Real (+ vbx (cx wbx wby wbz (- pbx xbx) (- pby xby) (- pbz xbz))))
(define-fun dpby () Real (+ vby (cy wbx wby wbz (- pbx xbx) (- pby xby) (- pbz xbz))))
(define-fun dpbz () Real (+ vbz (cz wbx wby wbz (- pbx xbx) (- pby xby) (- pbz xbz))))
(define-fun derivative () Real (+ (dot ex ey ez (- dpbx dpax) (- dpby dpay) (- dpbz dpaz)) (dot (cx wax way waz ex ey ez) (cy wax way waz ex ey ez) (cz wax way waz ex ey ez) (- pbx pax) (- pby pay) (- pbz paz))))
(define-fun line () Real (+ (dot ex ey ez (- vbx vax) (- vby vay) (- vbz vaz)) (dot (cx (- pbx xbx) (- pby xby) (- pbz xbz) ex ey ez) (cy (- pbx xbx) (- pby xby) (- pbz xbz) ex ey ez) (cz (- pbx xbx) (- pby xby) (- pbz xbz) ex ey ez) wbx wby wbz) (- (dot (cx (- pbx xax) (- pby xay) (- pbz xaz) ex ey ez) (cy (- pbx xax) (- pby xay) (- pbz xaz) ex ey ez) (cz (- pbx xax) (- pby xay) (- pbz xaz) ex ey ez) wax way waz))))
(assert (not (= derivative line)))
(check-sat)
