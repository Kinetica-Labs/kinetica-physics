; Непрерывный SAT для поступательного движения с постоянной скоростью.
; Вход: замкнутые проекции выпуклых тел на фиксированную ось, координаты в м,
; относительная скорость B в м/с, время в с. Касание считается пересечением.
; Предпосылка применения к многогранникам: задан полный набор неподвижных осей SAT.
; Выбор/полнота осей, вращение, вычисление опорных точек и IEEE-754 вне модели.
; Свободные константы позволяют Z3 искать контрпример среди ВСЕХ вещественных
; значений. UNSAT для отрицания равносильности доказывает оба направления.
(set-logic QF_NRA)
(declare-const alo Real)
(declare-const ahi Real)
(declare-const blo Real)
(declare-const bhi Real)
(declare-const a Real)
(declare-const b Real)
(declare-const v Real)
(declare-const t Real)
(declare-const T Real)
(declare-const l Real)
(declare-const u Real)
(declare-const p Real)
(declare-const q Real)

(define-fun inside ((low Real) (high Real) (x Real)) Bool
  (and (<= low x) (<= x high)))
(define-fun meet ((a0 Real) (a1 Real) (b0 Real) (b1 Real)) Bool
  (and (<= a0 b1) (<= b0 a1)))
(define-fun larger ((x Real) (y Real)) Real (ite (>= x y) x y))
(define-fun smaller ((x Real) (y Real)) Real (ite (<= x y) x y))

; A=[alo,ahi], B(t)=[blo+vt,bhi+vt], поэтому a=alo-bhi, b=ahi-blo.
(define-fun projection-ok () Bool
  (=> (and (<= alo ahi) (<= blo bhi))
      (= (meet alo ahi (+ blo (* v t)) (+ bhi (* v t)))
         (inside (- alo bhi) (- ahi blo) (* v t)))))

; После умножения на отрицательную скорость границы меняются местами.
; Нулевую скорость обрабатываем без деления: за весь горизонт либо вся ось
; допускает контакт, либо ни один момент его не допускает.
(define-fun positive-ok () Bool
  (=> (> v 0) (= (inside a b (* v t)) (inside (/ a v) (/ b v) t))))
(define-fun negative-ok () Bool
  (=> (< v 0) (= (inside a b (* v t)) (inside (/ b v) (/ a v) t))))
(define-fun stationary-ok () Bool
  (=> (= v 0) (= (and (inside 0 T t) (inside a b (* v t)))
                 (and (inside 0 T t) (inside a b 0)))))

; Одно звено накопления пересечения. Включает уже пустой накопитель l>u
; или пустую новую ось p>q. База для нуля осей — [0,T]. По индукции любое
; конечное число осей имеет ровно те t, которые входят во все их интервалы.
(define-fun fold-ok () Bool
  (= (inside (larger l p) (smaller u q) t)
     (and (inside l u t) (inside p q t))))

; Ограничение общим закрытым горизонтом T>=0. При L<=U само L — свидетель
; и самый ранний контакт. При L>U ни один t не проходит; равенство не пусто.
(define-fun entry () Real (larger 0 l))
(define-fun leave () Real (smaller T u))
(define-fun eligible ((x Real)) Bool (and (inside 0 T x) (inside l u x)))
(define-fun horizon-ok () Bool
  (=> (>= T 0)
      (and (= (<= entry leave) (eligible entry))
           (=> (eligible t) (inside entry leave t))
           (=> (> entry leave) (not (eligible t)))
           (=> (= entry leave) (= (eligible t) (= t entry)))) ))

; Два единичных квадрата (или куба с одинаковой высотой). A=[0,1]^2,
; B(t)=[1+t-m,2+t-m] x [1+m-t,2+m-t]. Скорость B равна (+1,-1) м/с.
; Они соприкасаются углами только в момент m. Для любой пары соседних
; временных выборок p<q берём m=(p+q)/2. Любая выборка вне (p,q) пропустит
; это касание, включая оба конца. Между любыми двумя соседними узлами
; конечной сетки лежит такой m, вне зависимости от размера интервала.
(define-fun corner ((x Real) (m Real)) Bool
  (and (meet 0 1 (+ 1 (- x m)) (+ 2 (- x m)))
       (meet 0 1 (+ 1 (- m x)) (+ 2 (- m x)))))
(define-fun mid () Real (/ (+ p q) 2))
(define-fun missed-singleton-ok () Bool
  (=> (< p q)
      (and (< p mid) (< mid q) (corner mid mid)
           (= (corner t mid) (= t mid))
           (=> (or (<= t p) (>= t q)) (not (corner t mid))))))

; Первый ответ обязан быть UNSAT, либо SAT при удалении последнего assert.
; Это соглашение proofs/check.sh. Последующие запросы помечены по отдельности.
(push 1)
(assert (not (and projection-ok positive-ok negative-ok stationary-ok fold-ok horizon-ok missed-singleton-ok)))
(check-sat)
(pop 1)
(echo "projection_equivalence expected unsat")
(check-sat-assuming ((not projection-ok)))
(echo "positive_velocity expected unsat")
(check-sat-assuming ((not positive-ok)))
(echo "negative_velocity expected unsat")
(check-sat-assuming ((not negative-ok)))
(echo "zero_velocity expected unsat")
(check-sat-assuming ((not stationary-ok)))
(echo "intersection_fold expected unsat")
(check-sat-assuming ((not fold-ok)))
(echo "closed_horizon_and_first_contact expected unsat")
(check-sat-assuming ((not horizon-ok)))
(echo "singleton_in_any_gap expected unsat")
(check-sat-assuming ((not missed-singleton-ok)))

; Отдельный конкретный контрпример: пять снимков в [0,1] не показывают
; столкновения, но квадрат касается второго при t=3/8 с.
(define-fun missed-by-five () Bool
  (let ((m (/ 3 8)))
    (and (corner m m) (not (corner 0 m)) (not (corner (/ 1 4) m))
         (not (corner (/ 1 2) m)) (not (corner (/ 3 4) m)) (not (corner 1 m)))))
(echo "five_samples_miss_contact_at_3_over_8 expected sat")
(check-sat-assuming (missed-by-five))
