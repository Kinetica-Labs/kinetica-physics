; Лемма 24. Симметрия строки контакта: зазор не меняется, когда пара тел движется как целое.
; Зазор g = n · (P_b − P_a), P_a и P_b — точки контакта двух тел, n — нормаль. При бесконечно малом
; движении всего как одного твёрдого тела точка x смещается на t + ω × x, направление — на ω × n.
; Первая вариация δg = (ω × n) · (P_b − P_a) + n · (δP_b − δP_a) равна нулю при любых t и ω:
; сдвиг t сокращается в разности, а поворот даёт (ω × n) · d + n · (ω × d) = 0. Это однородность и
; изотропность пространства для контакта — посылка теоремы Нётер. Ответ unsat.
(declare-const nx Real) (declare-const ny Real) (declare-const nz Real)
(declare-const pax Real) (declare-const pay Real) (declare-const paz Real)
(declare-const pbx Real) (declare-const pby Real) (declare-const pbz Real)
(declare-const tx Real) (declare-const ty Real) (declare-const tz Real)
(declare-const wx Real) (declare-const wy Real) (declare-const wz Real)
(assert (not (= (+ (+ (* (- (* wy nz) (* wz ny)) (- pbx pax)) (* (- (* wz nx) (* wx nz)) (- pby pay)) (* (- (* wx ny) (* wy nx)) (- pbz paz))) (+ (* nx (- (+ tx (- (* wy pbz) (* wz pby))) (+ tx (- (* wy paz) (* wz pay))))) (* ny (- (+ ty (- (* wz pbx) (* wx pbz))) (+ ty (- (* wz pax) (* wx paz))))) (* nz (- (+ tz (- (* wx pby) (* wy pbx))) (+ tz (- (* wx pay) (* wy pax))))))) 0.0)))
(check-sat)
