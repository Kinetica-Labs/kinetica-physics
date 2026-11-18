# Сводная матрица «движок × ось»

Сгенерировано `research/facts.py tables` из `research/data/` — руками не править. Подстрочная метка — вид свидетельства; `·` — факт не установлен.

## Покрытие (заполнено ключей / всего)

| Ось | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| passport | 0/4 | 4/4 | 0/4 | 0/4 | 4/4 | 0/4 | 0/4 |
| state | 0/3 | 3/3 | 0/3 | 0/3 | 3/3 | 0/3 | 0/3 |
| pipeline | 0/1 | 1/1 | 0/1 | 0/1 | 1/1 | 0/1 | 0/1 |
| broadphase | 3/7 | 7/7 | 1/7 | 1/7 | 7/7 | 1/7 | 2/7 |
| midphase | 0/4 | 4/4 | 0/4 | 0/4 | 4/4 | 0/4 | 0/4 |
| narrowphase | 0/7 | 7/7 | 0/7 | 0/7 | 7/7 | 0/7 | 0/7 |
| manifold | 0/6 | 6/6 | 0/6 | 0/6 | 6/6 | 0/6 | 0/6 |
| contact | 0/6 | 6/6 | 0/6 | 0/6 | 6/6 | 0/6 | 0/6 |
| solver | 0/8 | 8/8 | 0/8 | 1/8 | 8/8 | 0/8 | 0/8 |
| joints | 0/6 | 6/6 | 0/6 | 0/6 | 6/6 | 0/6 | 0/6 |
| integrator | 0/3 | 3/3 | 0/3 | 0/3 | 3/3 | 0/3 | 0/3 |
| ccd | 0/2 | 2/2 | 0/2 | 0/2 | 2/2 | 0/2 | 0/2 |
| islands | 0/2 | 2/2 | 0/2 | 0/2 | 2/2 | 0/2 | 0/2 |
| perf | 0/4 | 4/4 | 0/4 | 0/4 | 4/4 | 0/4 | 0/4 |
| determinism | 0/3 | 3/3 | 0/3 | 0/3 | 3/3 | 0/3 | 0/3 |
| diff | 0/2 | 2/2 | 0/2 | 0/2 | 2/2 | 0/2 | 0/2 |
| queries | 0/1 | 1/1 | 0/1 | 0/1 | 1/1 | 0/1 | 0/1 |
| api | 0/2 | 2/2 | 0/2 | 0/2 | 2/2 | 0/2 | 0/2 |
| weak | 0/1 | 1/1 | 0/1 | 0/1 | 1/1 | 0/1 | 0/1 |
| borrow | 0/1 | 1/1 | 0/1 | 0/1 | 1/1 | 0/1 | 0/1 |

## passport

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `language` | · | C17 <sub>src</sub> | · | · | C11 (ядро src/engine) + C++20 (компилятор моделей, XML, пул потоков) <sub>src</sub> | · | · |
| `license` | · | MIT <sub>src</sub> | · | · | Apache-2.0 <sub>src</sub> | · | · |
| `niche` | · | games <sub>src</sub> | · | · | robotics, research <sub>src</sub> | · | · |
| `precision` | · | float-double-pos <sub>src</sub> | · | · | both <sub>src</sub> | · | · |

## state

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `coords` | · | maximal <sub>src</sub> | · | · | generalized <sub>src</sub> | · | · |
| `layout` | · | mixed <sub>src</sub> | · | · | soa <sub>src</sub> | · | · |
| `handles` | · | generational-id <sub>src</sub> | · | · | name-id <sub>src</sub> | · | · |

## pipeline

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `order` | · | b3UpdateBroadPhasePairs (пары по флагам moved прошлого шага + b3CreateContact; b3DynamicTree_Rebuild уходит в фоновую задачу), b3Collide: b3CollideTask → переиспользование контакта либо b3UpdateContact; затем последовательно b3LinkContact / b3AddContactToGraph, b3Solve: b3PrepareJointsTask, b3PrepareContacts_Convex, b3PrepareContacts_MeshWide, b3PrepareContacts_Overflow, подшаг × subStepCount: b3IntegrateVelocitiesTask → warm start → solve (b3PushContacts_* + b3SolveJoint useBias=true) → b3IntegratePositionsTask → relax (b3SolveContacts_* + b3SolveJoint useBias=false), b3ApplyRestitution_* × restitutionIterations, b3StoreImpulses_*, b3FinalizeBodiesTask (поза, сон, fat AABB, b3SolveContinuous для быстрых тел), b3DynamicTree_Refit (kinematic, dynamic), b3BulletBodyTask, b3TrySleepIsland, b3OverlapSensors <sub>src</sub> | · | · | mj_checkPos, mj_checkVel, mj_fwdPosition, mj_kinematics, mj_comPos, mj_tendon, mj_makeM, mj_factorM, mj_collision, mj_makeConstraint, mj_island, mj_projectConstraint, mj_transmission, mj_sensorPos, mj_fwdVelocity, mj_comVel, mj_passive, mj_referenceConstraint, mj_rne, mj_sensorVel, mjcb_control, mj_fwdActuation, mj_fwdAcceleration, mj_fwdConstraint, mj_sensorAcc, mj_checkAcc, mj_Euler\|mj_RungeKutta\|mj_implicit\|mj_discrete, mj_advance, mj_sleep <sub>src</sub> | · | · |

## broadphase

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `structure` | dbvt <sub>src</sub> | dbvt <sub>src</sub> | dbvt <sub>src</sub> | quad-bvh <sub>src</sub> | sap-1axis <sub>src</sub> | mbp <sub>src</sub> | dbvt <sub>src</sub> |
| `static_split` | Отдельные деревья для активных и для статических/спящих объектов. <sub>src</sub> | три дерева по типу тела: static, kinematic, dynamic; пары ищутся только dynamic×dynamic, dynamic×static, dynamic×kinematic <sub>src</sub> | · | · | мировое тело (id 0) и неподвижные тела с плоскостью не входят в SAP: они сразу спарены со всеми телами, дальше работает BVH тела <sub>src</sub> | · | · |
| `margin` | · | fat AABB = точный AABB + B3_SPECULATIVE_DISTANCE (20 мм) + запас формы min(50 мм, 0.125 × радиус формы); у статических — только 20 мм <sub>src</sub> | · | · | не fat-AABB: бокс тела каждый шаг строится заново и расширяется на geom_margin + geom_gap (под override — 0.5*o_margin) <sub>src</sub> | · | · |
| `update` | partial-rebuild <sub>src</sub> | mixed <sub>src</sub> | · | · | rebuild <sub>src</sub> | · | mixed <sub>src</sub> |
| `pair_cache` | · | списка пар нет: хэш-множество pairSet ключей (shapeA, shapeB, child) отсекает уже существующие контакты; контакт удаляется в узкой фазе, когда fat AABB перестали пересекаться <sub>src</sub> | · | · | нет: пары и контакты создаются заново каждый шаг на арене mjData <sub>absence/src</sub> | · | · |
| `filter` | · | categoryBits/maskBits по 64 бита + groupIndex; исключение пары тел суставом с collideConnected=false; пользовательский b3CustomFilterFcn; пары без динамического тела, формы одного тела и сенсоры отбрасываются <sub>src</sub> | · | · | битовые маски contype/conaffinity (на теле — OR по geom, затем на паре geom); общее weld-тело; родитель–потомок (filterparent); оба спят; оба без dof; явные <pair> и <exclude> по сигнатурам; обратный вызов mjcb_contactfilter; ограничивающие сферы <sub>src</sub> | · | · |
| `parallel` | · | обход дерево-на-дерево: по dynamic-дереву собираются пары братьев с флагом moved и раздаются b3ParallelFor (b3SelfPairsTask); для static и kinematic серийный BFS до 64 затравок, затем b3CrossPairsTask; ключи пар копятся по рабочим, сливаются, сортируются и превращаются в контакты последовательно <sub>src</sub> | · | · | не распараллелен: mj_broadphase и обход BVH идут в основном потоке; по потокам делится только narrowphase <sub>src</sub> | · | · |

## midphase

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `mesh` | · | бинарный BVH b3MeshNode (сборка по SAH либо по медиане — опция useMedianSplit, до 4 треугольников на лист); у контакта кэш треугольников с расширенной областью запроса — BVH не опрашивается, пока AABB формы не вышел за неё <sub>src</sub> | · | · | в контактах сетки нет: mesh заменён выпуклой оболочкой (qhull при компиляции); опорная функция — hill-climbing по графу вершин оболочки; BVH граней mesh используется только лучами и парой mesh–SDF <sub>src</sub> | · | · |
| `heightfield` | · | b3_heightShape: регулярная сетка, запрос идёт прямо по диапазону ячеек (b3QueryHeightField), треугольники попадают в тот же конвейер, что и у сетки; только статические тела <sub>src</sub> | · | · | есть (mjGEOM_HFIELD): AABB выпуклого geom в системе поля выбирает подрешётку, каждая треугольная призма после теста по высоте сталкивается с geom общим выпуклым алгоритмом; до 50 контактов на пару <sub>src</sub> | · | · |
| `compound` | · | запечённый неизменяемый compound только для статических тел: внутреннее AABB-дерево в одном блобе; широкая фаза разворачивает пару в отдельный b3Contact на каждого пересечённого ребёнка (childIndex входит в ключ пары) <sub>src</sub> | · | · | составная форма = тело с несколькими geom; у каждого тела статическое бинарное BVH из AABB его geom в инерциальной системе тела, два дерева обходятся совместно с OBB-тестом <sub>src</sub> | · | · |
| `sdf` | · | none <sub>absence/src</sub> | · | · | mjGEOM_SDF (плагин или октодерево mesh): SDF–SDF — градиентный спуск из sdf_initpoints точек Халтона в пересечении AABB (по умолчанию 40 точек, 10 итераций); mesh–SDF — обход BVH граней и отбор farthest-point до 50 точек <sub>src</sub> | · | · |

## narrowphase

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `shape_types` | · | sphere, capsule, hull, mesh, height-field, compound <sub>absence/src</sub> | · | · | plane, hfield, sphere, capsule, ellipsoid, cylinder, box, mesh (convex hull), sdf <sub>src</sub> | · | · |
| `dispatch` | · | таблица s_registers[typeA][typeB] хранит только флаги supported/primary (primary задаёт порядок A,B); сам алгоритм выбирается цепочкой if по типам в b3ComputeConvexManifold, указатель на функцию закомментирован <sub>src</sub> | · | · | статическая верхнетреугольная таблица указателей mjCOLLISIONFUNC[type1][type2] (9x9), пара упорядочивается по типу; таблица доступна пользователю для замены <sub>src</sub> | · | · |
| `convex_convex` | · | sat-gauss, gjk-distance, analytic <sub>src</sub> | · | · | gjk-epa, mpr, sat-clip, analytic <sub>src</sub> | · | · |
| `gjk_variant` | · | b3ShapeDistance: до 32 итераций, симплекс решается барицентрически (b3SolveSimplex2/3/4), стартует из b3SimplexCache прошлого шага (кэш сбрасывается, если метрика симплекса изменилась вдвое); останов — нет убывания расстояния, вырожденное направление поиска или полный тетраэдр (перекрытие) <sub>src</sub> | · | · | подалгоритм расстояния signed volumes (Montanari et al., ToG 2017); останов по зазору двойственности Франка–Вульфа <x_k, x_k - s_k> < 0.5*tol^2 (для пар многогранников tol = 0); старт из центров geom, между шагами симплекс не хранится; 35 итераций, tol 1e-6 <sub>src</sub> | · | · |
| `penetration` | · | sat, radius, analytic <sub>absence/src</sub> | · | · | radius, epa, mpr, sat, analytic <sub>src</sub> | · | · |
| `convex_radius` | · | нет: радиус есть только у сферы и капсулы, hull входит в GJK с радиусом 0 и сталкивается точной геометрией через SAT <sub>src</sub> | · | · | отдельного параметра нет; ядро + радиус применяется внутри: сфера = точка + r, капсула = отрезок + r, любой geom с margin = форма + margin/2 <sub>src</sub> | · | · |
| `mesh_edges` | · | при сборке сетки запекаются флаги рёбер (concave / inverse concave / flat), 8-битные углы соседей и кольца направлений рёбер у вершин; нормаль контакта на ребре или вершине проверяется по области Вороного соседей: admit (плоское ребро), skip (выпуклое — оставить соседу), snap (вогнутое — заменить нормалью треугольника), drop (обрыв); контакт с обратной стороны отбрасывается; многообразия треугольников склеиваются в кластеры по нормали <sub>src</sub> | · | · | специальной обработки нет: mesh сталкивается как выпуклая оболочка (внутренних рёбер нет), призмы hfield обрабатываются независимо <sub>src</sub> | · | · |

## manifold

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `generation` | · | face-clip, multi-analytic <sub>src</sub> | · | · | face-clip, multi-analytic, perturbation, single <sub>src</sub> | · | · |
| `max_points` | · | 4 на многообразие (B3_MAX_MANIFOLD_POINTS); у выпуклой пары одно многообразие, у пары с сеткой — по одному на кластер нормалей <sub>src</sub> | · | · | 50 на пару geom (mjMAXCONPAIR); фактически: box–box 8, грань–грань (mesh, box, cylinder) 4, с капсулой 2, сфера/эллипсоид 1, plane–box/mesh 4, hfield и SDF до 50 <sub>src</sub> | · | · |
| `reduction` | · | жадно, 4 шага: (1) самая глубокая и крайняя вдоль b3ArbitraryPerp(normal); (2) самая дальняя в плоскости, с добавкой 4·глубина²; (3) наибольшая площадь треугольника; (4) наибольшая добавочная площадь вне треугольника. Каждое сравнение идёт с bias 0.95 — гистерезис против мерцания набора точек <sub>src</sub> | · | · | отсечённый многоугольник грань–грань сокращается до четырёхугольника наибольшей площади (hull4); box–box намеренно не сокращается (до 8 точек); группы flex и mesh–SDF — жадный farthest-point от самой глубокой точки до 50 <sub>src</sub> | · | · |
| `matching` | · | feature-id <sub>prior/src</sub> | · | · | none <sub>absence/src</sub> | · | · |
| `speculative` | · | есть: точки принимаются до разделения B3_SPECULATIVE_DISTANCE = 4·slop = 20 мм; в решателе при s > 0 смещение s·inv_h без мягкости, то есть точка тормозит сближение ровно до касания за подшаг <sub>src</sub> | · | · | два механизма: (1) margin — контакт создаётся при dist < margin + gap и входит в решатель при dist < margin, т.е. до касания (по умолчанию margin = 0); (2) флаг speculative (выключен) — консервативное продвижение даёт контакт с эффективной dist <sub>src</sub> | · | · |
| `offsets` | · | B3_LINEAR_SLOP 5 мм (допуск геометрии и TOI, в формулу смещения контакта не входит); B3_SPECULATIVE_DISTANCE 20 мм; B3_MESH_REST_OFFSET 5 мм вычитается из разделения контактов с сеткой; B3_OVERLAP_SLOP 0.5 мм; скорость выталкивания ограничена contactSpeed = 3 м/с <sub>src</sub> | · | · | margin (сумма по двум geom; порог обнаружения margin + gap, порог силы margin), gap, ширина solimp[2]; slop / допустимого проникновения нет; точка контакта — середина между поверхностями <sub>src</sub> | · | · |

## contact

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `formulation` | · | soft-si <sub>src</sub> | · | · | soft-convex <sub>src</sub> | · | · |
| `softness` | · | b3MakeSoft(hertz, zeta, h): omega = 2π·hertz, a1 = 2·zeta + h·omega, a2 = h·omega·a1, a3 = 1/(1+a2); biasRate = omega/a1, massScale = a2·a3, impulseScale = a3. Контакт: contactHertz = min(30, 0.125·inv_h), zeta = 10; контакт со статикой вдвое жёстче (2·hertz, 0.5·zeta). Сустав: constraintHertz = min(60, 0.25·inv_h), zeta = 2 <sub>prior/src</sub> | · | · | на каждую строку: solref = (timeconst, dampratio) либо (-stiffness, -damping) и solimp = (dmin, dmax, width, midpoint, power); импеданс d(r) в [0.0001, 0.9999]; R = (1-d)/d * diagA; aref = -B*vel - K*d*(pos - margin), K = 1/(dmax^2*timeconst^2*dampratio^2), B = 2/(dmax*timeconst) <sub>src</sub> | · | · |
| `cone` | · | disk <sub>src</sub> | · | · | pyramid, elliptic <sub>src</sub> | · | · |
| `friction_mix` | · | трение sqrt(fA·fB), восстановление max(rA, rB); оба заменяются пользовательскими обратными вызовами с userMaterialId; у сетки с материалами по треугольникам значения усредняются по точкам контакта <sub>src</sub> | · | · | при разном priority берутся параметры geom с большим priority; при равном — трение: максимум по каждой компоненте, condim: максимум, solref/solimp: взвешенное среднее по solmix, adhesion: сумма <sub>src</sub> | · | · |
| `restitution` | · | отдельная стадия после всех подшагов, restitutionIterations = 2 прохода: точка «взводится», если скорость сближения до решения превышала restitutionThreshold (1 м/с) и за шаг был импульс сжатия; целевая скорость restitution·relativeVelocity, импульс ограничен по Пуассону: approachImpulse + max(restitution·(compressionImpulse + approachImpulse) − restitutionImpulse, 0) <sub>src</sub> | · | · | коэффициента нет: отскок получается из пружины–демпфера строки (dampratio < 1 или прямой формат solref = (-stiffness, -damping) с нулевым демпфированием); по умолчанию критическое демпфирование <sub>src</sub> | · | · |
| `rolling_torsion` | · | качение: угловой импульс с массой inv(invIA + invIB), зажат по длине rollingResistance·ΣnormalImpulse (rollingResistance = коэффициент материала × радиус формы); кручение: скалярный импульс вокруг нормали, предел friction·Σ(leverArm·normalImpulse), где leverArm — расстояние точки от центра трения <sub>src</sub> | · | · | condim 4 добавляет трение кручения вокруг нормали, condim 6 — ещё два направления качения; коэффициенты friction[5] = (tangent1, tangent2, spin, roll1, roll2) <sub>src</sub> | · | · |

## solver

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `family` | · | soft-step <sub>prior/src</sub> | · | · | newton, cg, pgs-si <sub>src</sub> | · | · |
| `stabilization` | · | soft, relax <sub>absence/prior/src</sub> | · | · | soft, implicit <sub>absence/src</sub> | · | · |
| `warm_start` | · | Между шагами: normalImpulse каждой точки по featureId; frictionImpulse, twistImpulse, rollingImpulse — на многообразие, как мировые векторы, перепроецируются на новые касательные и переносятся независимо от совпадения точек (у сетки — при совпадении нормали многообразия). Внутри шага warm start применяется в каждом подшаге импульсами предыдущего подшага; суставы хранят импульсы в b3JointSim. При переиспользовании контакта сохраняются якоря и импульсы, пересчитывается только separation. <sub>src</sub> | · | · | переносится один вектор qacc_warmstart (ускорение прошлого шага, nv чисел), а не импульсы контактов; перед итерациями сравнивается стоимость в qacc_warmstart и в qacc_smooth, берётся лучшая; для PGS силы восстанавливаются из qacc_warmstart и обнуляются, если двойственная стоимость > 0 <sub>src</sub> | · | · |
| `iterations` | · | subStepCount — аргумент b3World_Step (минимум 1, умолчания в API нет, тесты и примеры берут 4); на подшаг ровно 1 проход push и 1 проход relax (ITERATIONS, RELAX_ITERATIONS — макросы); после подшагов 2 прохода восстановления <sub>src</sub> | · | 10 скоростных + 2 позиционных итерации на один шаг столкновений; число шагов столкновений inCollisionSteps задаёт вызывающий в Update() <sub>src</sub> | iterations = 100 (максимум), ls_iterations = 50, tolerance = 1e-8, ls_tolerance = 0.01, noslip_iterations = 0; подшагов нет, timestep = 0.002 <sub>absence/src</sub> | · | · |
| `order` | · | В каждой стадии сначала overflow-ограничения (последовательно: суставы, затем контакты), затем цвета по возрастанию индекса; внутри цвета блоки суставов, широких выпуклых и сеточных контактов решаются одновременно. Контакты динамика×динамика занимают цвета с 0 (до 20), контакты со статикой — с конца (22 → 1), поэтому решаются последними и «побеждают». Внутри многообразия в relax: нормали → качение → кручение → трение. <sub>src</sub> | · | · | строки идут в порядке equality → сухое трение → пределы → контакты; PGS обходит блоки в случайном порядке, перемешивая их перед каждым проходом (PCG32 с фиксированным зерном); noslip — подряд; Newton/CG обновляют все dof разом <sub>src</sub> | · | · |
| `block` | · | блочного решения нормалей многообразия нет — точки решаются по одной; блоки есть только у трения (2×2 tangentMass в центре трения), у точечной связи сустава (3×3, b3SolvePointConstraint, матрица пересчитывается каждый подшаг) и у соосности revolute (2×2) <sub>absence/src</sub> | · | · | PGS: эллиптический контакт — блок размерности 3..6: шаг вдоль луча из вершины конуса, затем QCQP по трению при фиксированной нормали; noslip для пирамиды — пары встречных рёбер 2x2; Newton — все dof совместно через Cholesky гессиана <sub>src</sub> | · | · |
| `parallel` | · | graph-color <sub>src</sub> | · | · | islands <sub>absence/src</sub> | · | · |
| `mass_ratio` | · | специальных приёмов нет: только подшаги + мягкие ограничения + relax; контакты со статикой вдвое жёстче и решаются последними (приоритет цвета), чтобы тело не продавливалось сквозь пол; длинные цепи и экстремальные отношения масс признаны ограничением <sub>absence/src</sub> | · | · | (1) суставы в обобщённых координатах — цепь не содержит ограничений, которые надо сводить; (2) Newton с точным гессианом H = M + J'*D*J и Cholesky (разреженным, с rank-1 обновлениями) не зависит от длины цепи и отношения масс; (3) регуляризация R каждой строки пропорциональна её обратной инерции diagA; (4) CG предобусловлен M^-1; (5) PGS — Нестеров с рестартом <sub>src</sub> | · | · |

## joints

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `coords` | · | maximal <sub>src</sub> | · | · | reduced <sub>src</sub> | · | · |
| `algorithm` | · | none <sub>absence</sub> | · | · | CRBA (mj_crb) в разреженную M со структурой дерева → разложение L'*D*L в обратном порядке строк без заполнения (mj_factorI) → смещение через RNE без ускорения → qacc_smooth = M^-1*qfrc_smooth обратной подстановкой; ABA не используется <sub>src</sub> | · | · |
| `types` | · | parallel, distance, filter, motor, prismatic, revolute, spherical, weld, wheel, mover, pogo <sub>src</sub> | · | · | free, ball, slide, hinge <sub>src</sub> | · | · |
| `drives` | · | пружина — мягкое ограничение b3MakeSoft(hertz, dampingRatio, h); мотор — скоростная строка с зажимом maxMotorTorque·h; предел — односторонняя строка: до предела спекулятивное смещение c·inv_h, за пределом мягкое смещение (только в проходе useBias); якоря поворачиваются на deltaRotation каждый подшаг <sub>src</sub> | · | · | приводы — обобщённые силы qfrc_actuator = moment'*force (gain/bias/dynamics; серво позиции и скорости — аффинный bias), не ограничения; пружины и демпферы суставов — пассивные силы; пределы — односторонние мягкие строки, включаемые при dist < margin; сухое трение — строка с силой в пределах ±frictionloss; armature — добавка к диагонали M <sub>src</sub> | · | · |
| `loops` | · | отдельной обработки нет: замкнутая цепь — обычный набор парных ограничений в графе раскраски и в острове <sub>src</sub> | · | · | equality-ограничения: connect (3 строки), weld (6), joint и tendon (связь значений кубическим полиномом), flex; это мягкие строки общего решателя с теми же solref/solimp и поправкой Jdot*v для connect/weld <sub>src</sub> | · | · |
| `tendons` | · | none <sub>absence/src</sub> | · | · | fixed (линейная комбинация координат суставов) и spatial (путь через site с огибанием сферы и цилиндра, блоки pulley); длина и разреженный якобиан ten_J; у сухожилия свои пределы, сухое трение, пружина, демпфер и armature (M += armature * ten_J' * ten_J) <sub>src</sub> | · | · |

## integrator

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `scheme` | · | semi-implicit-euler <sub>src</sub> | · | · | semi-implicit-euler, rk4, implicit-velocity, linearly-implicit-metric <sub>src</sub> | · | · |
| `gyro` | · | неявно: I·(w2 − w1) + h·cross(w2, I·w2) = 0 решается Ньютоном–Рафсоном в локальных осях, B3_GYROSCOPIC_ITERATIONS = 1; включается флагом b3_enableGyroscopicTorque только у тел, чей тензор инерции отклоняется от изотропного больше isotropicInertiaTolerance (0.2) <sub>src</sub> | · | · | явно: член cvel x (cinert*cvel) в RNE входит в qfrc_bias; implicit добавляет производную RNE по скорости в M - h*qDeriv (LU); implicitfast её отбрасывает, но для отдельных свободных тел решает локальную несимметричную систему 6x6; RK4 — поправка Рунге–Кутты–Мунте-Кааса для кватернионов <sub>src</sub> | · | · |
| `damping` | · | демпфирование Паде 1/(1 + h·c) для линейной и угловой скорости (по умолчанию 0); линейная скорость ограничена maximumLinearSpeed = 400 м/с, поворот за шаг — B3_MAX_ROTATION = π/4 (снимается флагом allowFastRotation); блокировки осей применяются при интегрировании поз <sub>src</sub> | · | · | глобального демпфирования и ограничения скоростей нет; демпфирование — параметр сустава/сухожилия и вязкость среды; при NaN или \|значение\| > mjMAXVAL — предупреждение и автосброс состояния <sub>absence/src</sub> | · | · |

## ccd

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `method` | · | speculative, toi-clamp <sub>src</sub> | · | · | speculative, conservative-advancement <sub>src</sub> | · | · |
| `scope` | · | динамическое тело против статических форм, если maxMotion > safetyFactor·minExtent (safetyFactor 0.5); тело с isBullet дополнительно против kinematic и dynamic, но не против других bullet; формы-сетки на быстром теле и сенсоры пропускаются; общего dynamic×dynamic нет, через суставы не распространяется <sub>src</sub> | · | · | пары geom типов plane/sphere/capsule/ellipsoid/cylinder/box/mesh без активного контакта, где смещение geom за шаг не меньше минимальной полутолщины AABB пары; hfield, SDF, flex — нет <sub>src</sub> | · | · |

## islands

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `structure` | · | persistent <sub>src</sub> | · | · | union-find <sub>src</sub> | · | · |
| `sleep` | · | тело «сонное», если sleepVelocity ≤ sleepThreshold (0.05 м/с), где sleepVelocity = max(\|v\| + скорость дальней точки от вращения, 0.5·inv_dt·перемещение за шаг); остров засыпает, когда у всех тел sleepTime ≥ B3_TIME_TO_SLEEP (0.5 с) и нет отложенного расщепления; спящий остров переезжает в свой b3SolverSet; будит начало касания с бодрствующим телом (b3LinkContact → b3WakeSolverSet) <sub>src</sub> | · | · | выключен по умолчанию (mjENBL_SLEEP); единица — кинематическое дерево, усыпляется остров целиком: у всех деревьев max\|dof_length*qvel\| < sleep_tolerance (1e-3) в течение mjMINAWAKE = 10 шагов и нет приложенных сил; будит контакт со бодрствующим деревом, сухожилие, equality <sub>src</sub> | · | · |

## perf

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `threading` | · | Задачи отдаются наружу через enqueueTask/finishTask либо встроенному планировщику (потоки + семафор), иначе выполняются на месте. Параллельны: поиск пар, перестройка дерева (фоновая задача на время узкой фазы), узкая фаза, все стадии решателя, финализация тел с CCD, bullet, сенсоры, расщепление острова. Последовательны: создание контактов, смена состояния контактов и слияние островов, overflow-ограничения, подгонка дерева, сон. Решатель: блоки по ≥4 широких ограничений или ≥32 тел, до 4 блоков на рабочего, захват блока CAS по syncIndex; рабочие ждут смены стадии активным ожиданием. <sub>src</sub> | · | · | свой пул (std::thread, общий атомарный счётчик задач, основной поток участвует и ждёт спином), включается mju_threadpool; параллельны чанки narrowphase, решение островов, тактильные сенсоры; по умолчанию один поток. Отдельно — mjBatch: много симуляций одной модели на пуле <sub>src</sub> | · | · |
| `simd` | · | Ширина 4 (SSE2, NEON на AArch64, скалярная эмуляция) или 8 (AVX2, выбирается в рантайме по cpuid; один и тот же .inl компилируется дважды). Векторизованы: подготовка, warm start, push, relax, восстановление выпуклых и сеточных контактов (один контакт на дорожку, AoSoA), SAT hull×hull, отсев треугольников сетки, пересечение и подгонка узлов дерева. Скалярны: суставы, overflow-контакты, интегрирование тел. <sub>src</sub> | · | · | только AVX (256 бит, 4 double) в разреженных скалярных произведениях, под mjUSEPLATFORMSIMD и только для double; остальное — скалярный C <sub>src</sub> | · | · |
| `gpu` | · | none <sub>absence/src</sub> | · | · | в C-ядре нет; отдельные реализации: MJX (JAX, каталог mjx/) и mujoco_warp (Warp, пакет миров nworld). Обе — только CG/Newton без PGS; MJX: выпуклые пары через SAT с картой Гаусса и отсечение, плотный Cholesky на каждой итерации; Warp: порт GJK/EPA, broadphase NXN или SAP, буферы фиксированного размера <sub>src</sub> | · | · |
| `memory` | · | b3Stack — LIFO-арена шага (широкие ограничения, блоки, стадии; при нехватке идёт в кучу и растёт к следующему шагу); по b3Arena на рабочего для узкой фазы — передаётся по значению, поэтому указатель сам откатывается при выходе из функции; b3BlockAllocator на каждое число многообразий (под мьютексом); пулы id со свободными списками; выравнивание 32 байта <sub>src</sub> | · | · | один буфер на mjData плюс одна арена: массивы шага (контакты, efc, острова) растут снизу, стек временных массивов (mj_markStack / mj_freeStack) — сверху; malloc в шаге нет; под пулом потоков стек резервируется атомарным сложением; переполнение арены — предупреждение, стека — ошибка <sub>src</sub> | · | · |

## determinism

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `level` | · | cross-platform <sub>src</sub> | · | · | same-binary <sub>src</sub> | · | · |
| `means` | · | -ffp-contract=off; свои b3Atan2 и b3ComputeCosSin вместо libm; в SIMD нет rsqrt и настоящего FMA (b3MulAdd = сложение + умножение на всех бэкендах); скалярный SAT обязан повторять векторный; порядок: ключи пар сортируются, смены состояния контактов собираются в битовые множества по id и применяются последовательно, ничьи при выборе острова разрешаются по id; раскраска постоянная и меняется только в последовательном коде <sub>src</sub> | · | · | состояния-кэша нет, кроме qacc_warmstart (входит в состояние интегрирования); пары тел сортируются по сигнатуре; потоки narrowphase пишут в заранее рассчитанные слоты, контакты добавляются строго по префиксу; номера островов — по возрастанию корня; перемешивание PGS — PCG32 с фиксированным зерном <sub>absence/src</sub> | · | · |
| `snapshot` | · | снимок мира есть как часть записи: b3SerializeWorld / b3DeserializeIntoShell пишут сырой образ структур с хэшем раскладки; наружу он выходит через b3World_StartRecording и ключевые кадры b3RecPlayer, отдельного публичного save/restore нет <sub>src</sub> | · | · | mj_getState / mj_setState с битовой маской mjtState; mjSTATE_INTEGRATION = время, qpos, qvel, act, история, пластика, плагины, пользовательские входы и warmstart; целиком — mj_copyData <sub>src</sub> | · | · |

## diff

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `derivatives` | · | none <sub>absence</sub> | · | · | конечные разности: mjd_transitionFD (матрицы перехода A, B, C, D; односторонние или центральные), mjd_inverseFD; аналитически — только производные гладких сил по скорости (mjd_smooth_vel → qDeriv) для неявных интеграторов; autodiff в C нет (есть в MJX через JAX) <sub>src</sub> | · | · |
| `inverse_dynamics` | · | none <sub>absence/src</sub> | · | · | есть: mj_inverse — аналитическая; силы ограничений получаются формулами по строкам без итераций (обратная задача диагональна по R); флаг fwdinv сравнивает прямую и обратную динамику каждый шаг <sub>src</sub> | · | · |

## queries

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `kinds` | · | overlap AABB, overlap shape (выпуклый прокси), ray cast с обратным вызовом, ray cast closest, shape cast, варианты *_Static только по статическому дереву, mover cast / collide mover (капсула персонажа), ray/shape cast по одному телу, ray cast и ближайшая точка по одной форме, сенсоры (события перекрытия) <sub>src</sub> | · | · | raycast, multi-raycast, geom-distance, distance/contact sensors <sub>absence/src</sub> | · | · |

## api

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `formats` | · | нет форматов моделей (URDF/MJCF/SDF не читаются), b3rec — запись и воспроизведение сессии, запечённые байтовые блобы: compound, hull, mesh, height field <sub>absence/src</sub> | · | · | MJCF (XML), URDF, MJB (двоичный mjModel), mjSpec (программная сборка модели), OpenUSD (experimental) <sub>src</sub> | · | · |
| `style` | · | плоский C-API: непрозрачные id по значению, структуры-описания с b3Default*Def() и проверочным cookie, b3World_Step(worldId, timeStep, subStepCount), события массивами после шага, обратные вызовы для фильтра и pre-solve, внешний планировщик через enqueueTask/finishTask; снаружи не потокобезопасен, кроме запросов на чтение <sub>src</sub> | · | · | C API вокруг двух структур: mjModel (описание, const для всех функций шага) и mjData (состояние и рабочая память); mj_step(m, d) или по стадиям mj_forward / mj_step1 / mj_step2; глобальные обратные вызовы mjcb_*; таблица mjCOLLISIONFUNC заменяема; плагины <sub>src</sub> | · | · |

## weak

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `known` | · | экстремальные отношения масс между связанными телами растягивают суставы и дают перекрытие контактов, мягкие ограничения допускают прогиб суставов и контактов; длинные цепочки weld гнутся, нет общего CCD динамика×динамика; CCD не проходит через суставы; при ударе теряется остаток шага, mesh, height field и compound — только на статических телах, нет цилиндра; hull ограничен 128 вершинами/гранями/рёбрами, ограничения сверх 24 цветов (overflow) решаются одним потоком скалярным кодом, рабочие потоки решателя ждут активным ожиданием — автор сам пишет о заметной потере времени, стек обхода дерева фиксирован (512), мягкого отказа в release нет, pairSet после первого шага может остаться большим, character mover экспериментален; движок целиком — v0.1 «still maturing» <sub>src</sub> | · | · | мягкая выпуклая модель нарушает комплементарность: проскальзывание начинается только вместе с движением по нормали, остаточное скольжение контактов присуще мягкой модели; noslip — ad-hoc пост-проход, может вызывать неустойчивость, столкновения только выпуклых geom: mesh заменяется выпуклой оболочкой, невыпуклое — ручная декомпозиция, broadphase резервирует буфер на все n(n-1)/2 пар тел и отказывает при >= 65536 объектах (16-битные id пар), CCD выключен по умолчанию; после 16 шагов — неконсервативная линейная экстраполяция, у медленного тела ошибка плеча (TODO), diagApprox (основа регуляризации R) заморожена в qpos0 и изотропна: ошибается на длинных цепях, в тяжёлых случаях расходимость, PGS — сходимость первого порядка, признан худшим выбором, чем Newton, под Euler/implicit refsafe ограничивает жёсткость строки: timeconst >= 2*timestep, пирамидальный конус (по умолчанию) даёт иную мягкую динамику, чем эллиптический; мешал тонкому захвату, член Jdot*v для контактов опущен, точная воспроизводимость только в пределах одной версии и архитектуры, интегратор discrete новый и меняется; PGS и noslip с flex под ним не поддержаны <sub>src</sub> | · | · |

## borrow

| Ключ | bepu | box3d | bullet3 | jolt | mujoco | physx | rapier |
|---|---|---|---|---|---|---|---|
| `items` | · | Soft Step с разделением push/relax: трение, кручение и качение решаются только в relax — джиттер и рост энергии от split-impulse, мягкость через (hertz, zeta) с ограничением частоты по длине подшага и более жёсткой статикой — джиттер, продавливание пола, переиспользование контакта: при малом относительном сдвиге узкая фаза пропускается, якоря и импульсы сохраняются, separation пересчитывается — устаревшая реакция кэша и цена покоя, feature id как пара рёбер, устойчивая к смене опорной грани + трение на многообразие мировым вектором — устаревшая реакция кэша, редукция до 4 точек с bias 0.95 — мерцание набора точек, постоянная раскраска графа с приоритетом статических контактов и overflow-цветом — параллельный решатель, AoSoA-ограничения шириной 4/8 из одного .inl, gather/scatter 64-байтных b3BodyState — SIMD, стадии решателя с CAS по блокам и монотонным syncIndex, домашние диапазоны блоков у рабочих — накладные расходы планирования на лёгкой сцене, широкая фаза дерево-на-дерево по флагам moved + фоновая частичная перестройка + линейная подгонка — цена поиска пар, наборы решателя: спящий остров уезжает в свой массив, бодрствующие тела плотные — лёгкая сцена, стадия восстановления с ограничением по Пуассону — рост энергии при отскоке, правила детерминизма: без FMA и rsqrt, свои тригонометрические функции, сортировка пар, битовые множества событий <sub>src</sub> | · | · | обобщённые координаты + CRBA + L'DL без заполнения: цепь точна без итераций — длинные цепи, Newton по выпуклой задаче: H = M + J'DJ, Cholesky, rank-1 обновления при смене состояния строк, точный одномерный поиск — длинные цепи, отношения масс, сертификат сходимости на тёплом старте (0.5 g'M^-1 g < tol → ноль итераций, гессиан не строится) — лёгкие сцены, мягкая строка solref/solimp: aref = -B*v - K*d*(r - margin), R = (1-d)/d*diagA вместо позиционной коррекции — джиттер, неявная строка (discrete): пружина строки в конце шага, R/f и aref/f, f = 1 + hB + h^2Kd — джиттер, устойчивость при любой жёсткости, эллиптический конус: зоны top/middle/bottom с аналитической стоимостью и гессианом; в PGS — луч + QCQP — квадратное трение, согласование R по осям трения (R[j]*mu[j]^2 = const) и impratio — анизотропное трение, проскальзывание, одношаговый многоточечный контакт: совпадающие грани из симплекса GJK/EPA → polygonClip → hull4, без кэша — джиттер стопок, цена narrowphase, ядро + радиус в GJK (сфера = точка, капсула = отрезок, margin/2): мелкое проникновение без EPA — цена narrowphase, box–box: SAT по 15 осям с приоритетом граней и отсечение без сокращения до 4 точек — джиттер стопок, SAP по главной оси ковариации + статическое BVH на тело — поиск пар на лёгких сценах без состояния, острова (DSU по деревьям) + сон островов + раздача островов по потокам — лёгкие сцены, арена и стек в одном буфере, вся аллокация шага без malloc — производительность, PGS с ускорением Нестерова, градиентным рестартом и перемешиванием блоков — сходимость PGS, аналитическая обратная динамика как встроенная проверка сходимости решателя (fwdinv) — диагностика <sub>src</sub> | · | · |
