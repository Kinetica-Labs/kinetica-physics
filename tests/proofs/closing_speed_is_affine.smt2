; Лемма 8а. Скорость сближения линейна по положению точки. Точка твёрдого тела движется со
; скоростью v + ω × (x − c); её составляющая вдоль нормали n в точке λa·a + λb·b + (1 − λa − λb)·e
; равна той же смеси значений в точках a, b, e. Ответ unsat.
(declare-const nx Real) (declare-const ny Real) (declare-const nz Real)
(declare-const vx Real) (declare-const vy Real) (declare-const vz Real)
(declare-const wx Real) (declare-const wy Real) (declare-const wz Real)
(declare-const cx Real) (declare-const cy Real) (declare-const cz Real)
(declare-const ax Real) (declare-const ay Real) (declare-const az Real)
(declare-const bx Real) (declare-const by Real) (declare-const bz Real)
(declare-const ex Real) (declare-const ey Real) (declare-const ez Real)
(declare-const la Real) (declare-const lb Real)
(define-fun le () Real (- 1.0 la lb))
(define-fun closing ((x Real) (y Real) (z Real)) Real
  (+ (* nx (+ vx (- (* wy (- z cz)) (* wz (- y cy)))))
     (* ny (+ vy (- (* wz (- x cx)) (* wx (- z cz)))))
     (* nz (+ vz (- (* wx (- y cy)) (* wy (- x cx)))))))
(assert (not (= (closing (+ (* la ax) (* lb bx) (* le ex)) (+ (* la ay) (* lb by) (* le ey)) (+ (* la az) (* lb bz) (* le ez))) (+ (* la (closing ax ay az)) (* lb (closing bx by bz)) (* le (closing ex ey ez))))))
(check-sat)
