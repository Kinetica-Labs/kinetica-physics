; Лемма 25. Импульсы строки контакта — это λ на градиент зазора (принцип виртуальных работ).
; Решатель прикладывает импульс λ вдоль нормали n в общей точке P: телу b — силу λn и момент
; (P − x_b) × λn относительно центра масс x_b, телу a — противоположные (SolverBody::push).
; Пусть тела смещаются независимо: тело a — на t_a с поворотом ω_a, тело b — на t_b с поворотом ω_b.
; Работа этих сил и моментов F_a · t_a + τ_a · ω_a + F_b · t_b + τ_b · ω_b равна λ δg, где
; δg = n · (δP_b − δP_a) — изменение зазора в точке касания, δP = t + ω × (P − x). Ответ unsat.
(declare-const l Real)
(declare-const nx Real) (declare-const ny Real) (declare-const nz Real)
(declare-const px Real) (declare-const py Real) (declare-const pz Real)
(declare-const xax Real) (declare-const xay Real) (declare-const xaz Real)
(declare-const xbx Real) (declare-const xby Real) (declare-const xbz Real)
(declare-const tax Real) (declare-const tay Real) (declare-const taz Real)
(declare-const wax Real) (declare-const way Real) (declare-const waz Real)
(declare-const tbx Real) (declare-const tby Real) (declare-const tbz Real)
(declare-const wbx Real) (declare-const wby Real) (declare-const wbz Real)
(assert (not (= (+ (+ (* (- (* l nx)) tax) (* (- (* l ny)) tay) (* (- (* l nz)) taz)) (+ (* (- (- (* (- py xay) (* l nz)) (* (- pz xaz) (* l ny)))) wax) (* (- (- (* (- pz xaz) (* l nx)) (* (- px xax) (* l nz)))) way) (* (- (- (* (- px xax) (* l ny)) (* (- py xay) (* l nx)))) waz)) (+ (* (* l nx) tbx) (* (* l ny) tby) (* (* l nz) tbz)) (+ (* (- (* (- py xby) (* l nz)) (* (- pz xbz) (* l ny))) wbx) (* (- (* (- pz xbz) (* l nx)) (* (- px xbx) (* l nz))) wby) (* (- (* (- px xbx) (* l ny)) (* (- py xby) (* l nx))) wbz))) (* l (+ (* nx (- (+ tbx (- (* wby (- pz xbz)) (* wbz (- py xby)))) (+ tax (- (* way (- pz xaz)) (* waz (- py xay)))))) (* ny (- (+ tby (- (* wbz (- px xbx)) (* wbx (- pz xbz)))) (+ tay (- (* waz (- px xax)) (* wax (- pz xaz)))))) (* nz (- (+ tbz (- (* wbx (- py xby)) (* wby (- px xbx)))) (+ taz (- (* wax (- py xay)) (* way (- px xax)))))))))))
(check-sat)
