; Лемма 23. Вторая половина толчка на отрезке опоры не меняет момент импульса пары
; (ExactStep::move). Сила F действует на тело b, −F на тело a, обе — в общей точке P; моменты сил
; посчитаны в начале отрезка: τ_a = −(P − c_a) × F, τ_b = (P − c_b) × F. За отрезок центры масс
; сместились на g_a и g_b, а общая точка — на g (в коде carried: вместе с телом, лежащим на грани).
; К моментам добавлена поправка плеча: τ_a − (g − g_a) × F и τ_b + (g − g_b) × F. Тогда за половину
; толчка t = h/2 момент импульса пары относительно начала координат,
; Σ (ΔL + c' × Δp) с c' = c + g, не меняется — при любом смещении общей точки g. Ответ unsat.
(declare-const t Real) (declare-const fx Real) (declare-const fy Real) (declare-const fz Real) (declare-const px Real) (declare-const py Real) (declare-const pz Real) (declare-const cax Real) (declare-const cay Real) (declare-const caz Real) (declare-const cbx Real) (declare-const cby Real) (declare-const cbz Real) (declare-const gax Real) (declare-const gay Real) (declare-const gaz Real) (declare-const gbx Real) (declare-const gby Real) (declare-const gbz Real) (declare-const gx Real) (declare-const gy Real) (declare-const gz Real)
(define-fun tax () Real (- (- (* (- py cay) fz) (* (- pz caz) fy))))
(define-fun tay () Real (- (- (* (- pz caz) fx) (* (- px cax) fz))))
(define-fun taz () Real (- (- (* (- px cax) fy) (* (- py cay) fx))))
(define-fun tbx () Real (- (* (- py cby) fz) (* (- pz cbz) fy)))
(define-fun tby () Real (- (* (- pz cbz) fx) (* (- px cbx) fz)))
(define-fun tbz () Real (- (* (- px cbx) fy) (* (- py cby) fx)))
; поправки плеча
(define-fun eax () Real (- (- (* (- gy gay) fz) (* (- gz gaz) fy))))
(define-fun eay () Real (- (- (* (- gz gaz) fx) (* (- gx gax) fz))))
(define-fun eaz () Real (- (- (* (- gx gax) fy) (* (- gy gay) fx))))
(define-fun ebx () Real (- (* (- gy gby) fz) (* (- gz gbz) fy)))
(define-fun eby () Real (- (* (- gz gbz) fx) (* (- gx gbx) fz)))
(define-fun ebz () Real (- (* (- gx gbx) fy) (* (- gy gby) fx)))
(assert (not (and (= (+ (* t (+ tax eax)) (* t (+ tbx ebx)) (- (* (+ cay gay) (- (* t fz))) (* (+ caz gaz) (- (* t fy)))) (- (* (+ cby gby) (* t fz)) (* (+ cbz gbz) (* t fy)))) 0.0) (= (+ (* t (+ tay eay)) (* t (+ tby eby)) (- (* (+ caz gaz) (- (* t fx))) (* (+ cax gax) (- (* t fz)))) (- (* (+ cbz gbz) (* t fx)) (* (+ cbx gbx) (* t fz)))) 0.0) (= (+ (* t (+ taz eaz)) (* t (+ tbz ebz)) (- (* (+ cax gax) (- (* t fy))) (* (+ cay gay) (- (* t fx)))) (- (* (+ cbx gbx) (* t fy)) (* (+ cby gby) (* t fx)))) 0.0))))
(check-sat)
