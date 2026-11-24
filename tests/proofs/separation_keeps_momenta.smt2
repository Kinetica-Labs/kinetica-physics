; Лемма 22. Раздвигание перекрытий не меняет импульс и момент импульса свободной группы тел
; (OverlapSeparation::keepMomenta). Позы тел сдвинуты на sᵢ при прежних импульсах pᵢ: момент
; импульса относительно начала координат изменился на Σ sᵢ × pᵢ. Группа получает твёрдую поправку
; скорости — поворот b вокруг своего центра масс R: Δpᵢ = mᵢ b × dᵢ, ΔLᵢ = Iᵢ b, где dᵢ = rᵢ − R, а
; b найдено из I b = −Σ sᵢ × pᵢ с тензором инерции группы I b = Σ (mᵢ dᵢ × (b × dᵢ) + Iᵢ b).
; Тогда Σ Δpᵢ = 0 и Σ (sᵢ × pᵢ + rᵢ × Δpᵢ + ΔLᵢ) = 0.
; Три тела; R — центр масс, то есть Σ mᵢ dᵢ = 0: d₁ = m₃ u₁, d₂ = m₃ u₂, d₃ = −(m₁ u₁ + m₂ u₂) при
; любых u₁, u₂ — так записаны все расположения трёх тел. Тензоры Iᵢ — симметричные, в мировых осях.
; Для другого числа тел вывод тот же: в него входит только Σ mᵢ dᵢ = 0. Ответ unsat.
(declare-const m1 Real) (declare-const m2 Real) (declare-const m3 Real) (declare-const Rx Real) (declare-const Ry Real) (declare-const Rz Real) (declare-const u1x Real) (declare-const u1y Real) (declare-const u1z Real) (declare-const u2x Real) (declare-const u2y Real) (declare-const u2z Real) (declare-const bx Real) (declare-const by Real) (declare-const bz Real)
(declare-const s1x Real) (declare-const s1y Real) (declare-const s1z Real) (declare-const p1x Real) (declare-const p1y Real) (declare-const p1z Real) (declare-const i1xx Real) (declare-const i1xy Real) (declare-const i1xz Real) (declare-const i1yy Real) (declare-const i1yz Real) (declare-const i1zz Real)
(declare-const s2x Real) (declare-const s2y Real) (declare-const s2z Real) (declare-const p2x Real) (declare-const p2y Real) (declare-const p2z Real) (declare-const i2xx Real) (declare-const i2xy Real) (declare-const i2xz Real) (declare-const i2yy Real) (declare-const i2yz Real) (declare-const i2zz Real)
(declare-const s3x Real) (declare-const s3y Real) (declare-const s3z Real) (declare-const p3x Real) (declare-const p3y Real) (declare-const p3z Real) (declare-const i3xx Real) (declare-const i3xy Real) (declare-const i3xz Real) (declare-const i3yy Real) (declare-const i3yz Real) (declare-const i3zz Real)
(define-fun d1x () Real (* m3 u1x))
(define-fun d1y () Real (* m3 u1y))
(define-fun d1z () Real (* m3 u1z))
(define-fun d2x () Real (* m3 u2x))
(define-fun d2y () Real (* m3 u2y))
(define-fun d2z () Real (* m3 u2z))
(define-fun d3x () Real (- (+ (* m1 u1x) (* m2 u2x))))
(define-fun d3y () Real (- (+ (* m1 u1y) (* m2 u2y))))
(define-fun d3z () Real (- (+ (* m1 u1z) (* m2 u2z))))
; тело 1: поправка скорости b × d, поправка собственного момента I b
(define-fun w1x () Real (- (* by d1z) (* bz d1y)))
(define-fun w1y () Real (- (* bz d1x) (* bx d1z)))
(define-fun w1z () Real (- (* bx d1y) (* by d1x)))
(define-fun l1x () Real (+ (* i1xx bx) (* i1xy by) (* i1xz bz)))
(define-fun l1y () Real (+ (* i1xy bx) (* i1yy by) (* i1yz bz)))
(define-fun l1z () Real (+ (* i1xz bx) (* i1yz by) (* i1zz bz)))
; тело 2: поправка скорости b × d, поправка собственного момента I b
(define-fun w2x () Real (- (* by d2z) (* bz d2y)))
(define-fun w2y () Real (- (* bz d2x) (* bx d2z)))
(define-fun w2z () Real (- (* bx d2y) (* by d2x)))
(define-fun l2x () Real (+ (* i2xx bx) (* i2xy by) (* i2xz bz)))
(define-fun l2y () Real (+ (* i2xy bx) (* i2yy by) (* i2yz bz)))
(define-fun l2z () Real (+ (* i2xz bx) (* i2yz by) (* i2zz bz)))
; тело 3: поправка скорости b × d, поправка собственного момента I b
(define-fun w3x () Real (- (* by d3z) (* bz d3y)))
(define-fun w3y () Real (- (* bz d3x) (* bx d3z)))
(define-fun w3z () Real (- (* bx d3y) (* by d3x)))
(define-fun l3x () Real (+ (* i3xx bx) (* i3xy by) (* i3xz bz)))
(define-fun l3y () Real (+ (* i3xy bx) (* i3yy by) (* i3yz bz)))
(define-fun l3z () Real (+ (* i3xz bx) (* i3yz by) (* i3zz bz)))
; ушедший момент импульса Σ s × p и тензор инерции группы, применённый к b
(define-fun lostx () Real (+ (- (* s1y p1z) (* s1z p1y)) (- (* s2y p2z) (* s2z p2y)) (- (* s3y p3z) (* s3z p3y))))
(define-fun losty () Real (+ (- (* s1z p1x) (* s1x p1z)) (- (* s2z p2x) (* s2x p2z)) (- (* s3z p3x) (* s3x p3z))))
(define-fun lostz () Real (+ (- (* s1x p1y) (* s1y p1x)) (- (* s2x p2y) (* s2y p2x)) (- (* s3x p3y) (* s3y p3x))))
(define-fun groupx () Real (+ (* m1 (- (* d1y w1z) (* d1z w1y))) (* m2 (- (* d2y w2z) (* d2z w2y))) (* m3 (- (* d3y w3z) (* d3z w3y))) l1x l2x l3x))
(define-fun groupy () Real (+ (* m1 (- (* d1z w1x) (* d1x w1z))) (* m2 (- (* d2z w2x) (* d2x w2z))) (* m3 (- (* d3z w3x) (* d3x w3z))) l1y l2y l3y))
(define-fun groupz () Real (+ (* m1 (- (* d1x w1y) (* d1y w1x))) (* m2 (- (* d2x w2y) (* d2y w2x))) (* m3 (- (* d3x w3y) (* d3y w3x))) l1z l2z l3z))
(assert (and (= (+ groupx lostx) 0.0) (= (+ groupy losty) 0.0) (= (+ groupz lostz) 0.0)))
(assert (not (and (and (= (+ (* m1 w1x) (* m2 w2x) (* m3 w3x)) 0.0) (= (+ (* m1 w1y) (* m2 w2y) (* m3 w3y)) 0.0) (= (+ (* m1 w1z) (* m2 w2z) (* m3 w3z)) 0.0)) (and (= (+ lostx (- (* (+ Ry d1y) (* m1 w1z)) (* (+ Rz d1z) (* m1 w1y))) (- (* (+ Ry d2y) (* m2 w2z)) (* (+ Rz d2z) (* m2 w2y))) (- (* (+ Ry d3y) (* m3 w3z)) (* (+ Rz d3z) (* m3 w3y))) l1x l2x l3x) 0.0) (= (+ losty (- (* (+ Rz d1z) (* m1 w1x)) (* (+ Rx d1x) (* m1 w1z))) (- (* (+ Rz d2z) (* m2 w2x)) (* (+ Rx d2x) (* m2 w2z))) (- (* (+ Rz d3z) (* m3 w3x)) (* (+ Rx d3x) (* m3 w3z))) l1y l2y l3y) 0.0) (= (+ lostz (- (* (+ Rx d1x) (* m1 w1y)) (* (+ Ry d1y) (* m1 w1x))) (- (* (+ Rx d2x) (* m2 w2y)) (* (+ Ry d2y) (* m2 w2x))) (- (* (+ Rx d3x) (* m3 w3y)) (* (+ Ry d3y) (* m3 w3x))) l1z l2z l3z) 0.0)))))
(check-sat)
