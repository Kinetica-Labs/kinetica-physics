; Лемма 26. Вывод теоремы Нётер для пары тел: если силы и моменты не совершают работы на движениях
; пары как целого, то импульс и момент импульса пары не меняются.
; Телу a приложены сила F_a и момент τ_a относительно его центра масс x_a, телу b — F_b и τ_b. При
; движении пары как целого (сдвиг t, поворот ω вокруг начала координат) центр масс тела смещается на
; t + ω × x, само тело поворачивается на ω; работа F_a · (t + ω × x_a) + τ_a · ω + F_b · (t + ω × x_b) + τ_b · ω.
; Посылки — работа равна нулю на шести движениях: трёх сдвигах и трёх поворотах вдоль осей. Вывод:
; F_a + F_b = 0 и x_a × F_a + τ_a + x_b × F_b + τ_b = 0 — сохраняются импульс и момент импульса
; относительно начала координат. С леммами 24 и 25 это даёт: зазор симметричен ⇒ работа λ δg нулевая
; ⇒ строка контакта сохраняет импульс и момент импульса (лемма 12 проверяет то же прямым счётом).
; Ответ unsat.
(declare-const fax Real) (declare-const fay Real) (declare-const faz Real)
(declare-const fbx Real) (declare-const fby Real) (declare-const fbz Real)
(declare-const tauax Real) (declare-const tauay Real) (declare-const tauaz Real)
(declare-const taubx Real) (declare-const tauby Real) (declare-const taubz Real)
(declare-const xax Real) (declare-const xay Real) (declare-const xaz Real)
(declare-const xbx Real) (declare-const xby Real) (declare-const xbz Real)
(assert (= (+ (+ (* fax (+ 1.0 (- (* 0.0 xaz) (* 0.0 xay)))) (* fay (+ 0.0 (- (* 0.0 xax) (* 0.0 xaz)))) (* faz (+ 0.0 (- (* 0.0 xay) (* 0.0 xax))))) (+ (* tauax 0.0) (* tauay 0.0) (* tauaz 0.0)) (+ (* fbx (+ 1.0 (- (* 0.0 xbz) (* 0.0 xby)))) (* fby (+ 0.0 (- (* 0.0 xbx) (* 0.0 xbz)))) (* fbz (+ 0.0 (- (* 0.0 xby) (* 0.0 xbx))))) (+ (* taubx 0.0) (* tauby 0.0) (* taubz 0.0))) 0.0))
(assert (= (+ (+ (* fax (+ 0.0 (- (* 0.0 xaz) (* 0.0 xay)))) (* fay (+ 1.0 (- (* 0.0 xax) (* 0.0 xaz)))) (* faz (+ 0.0 (- (* 0.0 xay) (* 0.0 xax))))) (+ (* tauax 0.0) (* tauay 0.0) (* tauaz 0.0)) (+ (* fbx (+ 0.0 (- (* 0.0 xbz) (* 0.0 xby)))) (* fby (+ 1.0 (- (* 0.0 xbx) (* 0.0 xbz)))) (* fbz (+ 0.0 (- (* 0.0 xby) (* 0.0 xbx))))) (+ (* taubx 0.0) (* tauby 0.0) (* taubz 0.0))) 0.0))
(assert (= (+ (+ (* fax (+ 0.0 (- (* 0.0 xaz) (* 0.0 xay)))) (* fay (+ 0.0 (- (* 0.0 xax) (* 0.0 xaz)))) (* faz (+ 1.0 (- (* 0.0 xay) (* 0.0 xax))))) (+ (* tauax 0.0) (* tauay 0.0) (* tauaz 0.0)) (+ (* fbx (+ 0.0 (- (* 0.0 xbz) (* 0.0 xby)))) (* fby (+ 0.0 (- (* 0.0 xbx) (* 0.0 xbz)))) (* fbz (+ 1.0 (- (* 0.0 xby) (* 0.0 xbx))))) (+ (* taubx 0.0) (* tauby 0.0) (* taubz 0.0))) 0.0))
(assert (= (+ (+ (* fax (+ 0.0 (- (* 0.0 xaz) (* 0.0 xay)))) (* fay (+ 0.0 (- (* 0.0 xax) (* 1.0 xaz)))) (* faz (+ 0.0 (- (* 1.0 xay) (* 0.0 xax))))) (+ (* tauax 1.0) (* tauay 0.0) (* tauaz 0.0)) (+ (* fbx (+ 0.0 (- (* 0.0 xbz) (* 0.0 xby)))) (* fby (+ 0.0 (- (* 0.0 xbx) (* 1.0 xbz)))) (* fbz (+ 0.0 (- (* 1.0 xby) (* 0.0 xbx))))) (+ (* taubx 1.0) (* tauby 0.0) (* taubz 0.0))) 0.0))
(assert (= (+ (+ (* fax (+ 0.0 (- (* 1.0 xaz) (* 0.0 xay)))) (* fay (+ 0.0 (- (* 0.0 xax) (* 0.0 xaz)))) (* faz (+ 0.0 (- (* 0.0 xay) (* 1.0 xax))))) (+ (* tauax 0.0) (* tauay 1.0) (* tauaz 0.0)) (+ (* fbx (+ 0.0 (- (* 1.0 xbz) (* 0.0 xby)))) (* fby (+ 0.0 (- (* 0.0 xbx) (* 0.0 xbz)))) (* fbz (+ 0.0 (- (* 0.0 xby) (* 1.0 xbx))))) (+ (* taubx 0.0) (* tauby 1.0) (* taubz 0.0))) 0.0))
(assert (= (+ (+ (* fax (+ 0.0 (- (* 0.0 xaz) (* 1.0 xay)))) (* fay (+ 0.0 (- (* 1.0 xax) (* 0.0 xaz)))) (* faz (+ 0.0 (- (* 0.0 xay) (* 0.0 xax))))) (+ (* tauax 0.0) (* tauay 0.0) (* tauaz 1.0)) (+ (* fbx (+ 0.0 (- (* 0.0 xbz) (* 1.0 xby)))) (* fby (+ 0.0 (- (* 1.0 xbx) (* 0.0 xbz)))) (* fbz (+ 0.0 (- (* 0.0 xby) (* 0.0 xbx))))) (+ (* taubx 0.0) (* tauby 0.0) (* taubz 1.0))) 0.0))
(assert (not (and (= (+ fax fbx) 0.0) (= (+ fay fby) 0.0) (= (+ faz fbz) 0.0) (= (+ (+ (- (* xay faz) (* xaz fay)) tauax) (+ (- (* xby fbz) (* xbz fby)) taubx)) 0.0) (= (+ (+ (- (* xaz fax) (* xax faz)) tauay) (+ (- (* xbz fbx) (* xbx fbz)) tauby)) 0.0) (= (+ (+ (- (* xax fay) (* xay fax)) tauaz) (+ (- (* xbx fby) (* xby fbx)) taubz)) 0.0))))
(check-sat)
