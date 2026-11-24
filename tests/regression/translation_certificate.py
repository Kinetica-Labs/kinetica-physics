"""Точное время контакта выпуклых оболочек при равномерном относительном переносе.

A и B — выпуклые оболочки конечных множеств вершин в трёх измерениях, обе имеют
ненулевой объём. На закрытом [0,T] ориентации постоянны, B(t)=B(0)+v*t в системе A.
Для каждой оси n разделяющих плоскостей необходимы и достаточны два неравенства
min(A·n)−max(B·n) ≤ (v·n)t ≤ max(A·n)−min(B·n). Нормали граней A и B и произведения
направлений всех пар рёбер включают нормали всех граней разности Минковского A−B.
Следовательно, пересечение одномерных интервалов даёт все t, включая касания.
Строгие неравенства дают все t пересечения открытых внутренних объёмов.

Расчёт выполняется с Fraction и целыми числами. Десятичная запись из входа задаёт
точное рациональное число. Теорема верна и для вещественных координат; численная
реализация сертифицирует конкретный рациональный вход. Проекции в сертификате
взяты по ненормированным целочисленным осям, время в секундах, исходные точки в метрах.
"""
import argparse
import csv
import json
import math
from dataclasses import dataclass
from fractions import Fraction as Q
from itertools import combinations, product
from pathlib import Path


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def mul(k, a):
    return tuple(k * x for x in a)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    x, y, z = a
    u, v, w = b
    return y*w - z*v, z*u - x*w, x*v - y*u


def direction(a):
    """Единственный примитивный целочисленный представитель одной неориентированной оси."""
    d = math.gcd(*a)
    if not d:
        return None
    first = next(x for x in a if x)
    return tuple(x // (d if first > 0 else -d) for x in a)


def rotate(p, quat):
    # q p q* / (q·q) задаёт правильный поворот для любого ненулевого рационального q.
    w, *u = quat
    size = dot(quat, quat)
    if not size:
        raise ValueError("zero quaternion")
    t = mul(Q(2, size), cross(u, p))
    return add(p, add(mul(w, t), cross(u, t)))


def topology(points):
    """Восстанавливаем опорные плоскости оболочки и направления её настоящих рёбер."""
    if len(points) < 4:
        raise ValueError("four non-coplanar points required")
    faces = {}
    for i, j, k in combinations(range(len(points)), 3):
        n = direction(cross(sub(points[j], points[i]), sub(points[k], points[i])))
        if n is None:
            continue
        height = dot(n, points[i])
        if (n, height) in faces:
            continue
        g = [dot(n, p) - height for p in points]
        if (min(g) == 0 < max(g)) or (min(g) < 0 == max(g)):
            faces[n, height] = frozenset(index for index, gap in enumerate(g) if gap == 0)
    if not faces:
        raise ValueError("convex hull has zero volume")
    edges = {}
    for ((n, _), left), ((m, _), right) in combinations(faces.items(), 2):
        shared = sorted(left & right)
        if n != m and len(shared) >= 2:
            i, j = shared[:2]
            axis = direction(sub(points[j], points[i]))
            edges.setdefault(axis, (i, j))
    normals = {}
    for (n, _), indices in faces.items():
        normals.setdefault(n, sorted(indices))
    return normals, edges


@dataclass(frozen=True)
class Slab:
    axis: tuple
    source: tuple
    low: Q
    high: Q
    rate: Q

    def proof(self):
        return dict(axis=self.axis, source=self.source, low=self.low, high=self.high, rate=self.rate)

    def strict_at(self, t):
        return self.low < self.rate*t < self.high


def slabs(a, b, velocity):
    points = [tuple(map(Q, p)) for p in [*a, *b]]
    if any(len(p) != 3 for p in points) or len(velocity) != 3:
        raise ValueError("3-D points and velocity required")
    d = math.lcm(*(x.denominator for p in points for x in p))
    packed = [tuple(int(x*d) for x in p) for p in points]
    # Сортировка даёт устойчивые номера свидетелей независимо от порядка входных строк.
    a, b = sorted(set(packed[:len(a)])), sorted(set(packed[len(a):]))
    fa, ea = topology(a)
    fb, eb = topology(b)
    axes = {}
    for tag, faces in (("face_A", fa), ("face_B", fb)):
        for n, vertices in faces.items():
            axes.setdefault(n, (tag, vertices))
    for u, edge_a in ea.items():
        for v, edge_b in eb.items():
            n = direction(cross(u, v))
            if n is not None:
                axes.setdefault(n, ("edge_cross", edge_a, edge_b))
    result = []
    for n, source in sorted(axes.items()):
        aa = [dot(n, p) for p in a]
        bb = [dot(n, p) for p in b]
        result.append(Slab(n, source, Q(min(aa)-max(bb), d), Q(max(aa)-min(bb), d), Q(dot(velocity, n))))
    return result


def interval(lo, hi, include_lo=True, include_hi=True):
    return dict(lo=lo, hi=hi, include_lo=include_lo, include_hi=include_hi)


def solve(a, b, velocity, duration):
    duration = Q(duration)
    if duration < 0:
        raise ValueError("negative duration")
    rows = slabs(a, b, tuple(map(Q, velocity)))
    lo, hi = Q(0), duration
    lower, upper = {"domain": "start"}, {"domain": "end"}
    stationary_touch = stationary_miss = None
    for row in rows:
        if row.rate == 0:
            if not row.low <= 0 <= row.high:
                stationary_miss = row
            elif not row.strict_at(0):
                stationary_touch = row
            continue
        enter, leave = sorted((row.low / row.rate, row.high / row.rate))
        if enter > lo:
            lo, lower = enter, row.proof()
        if leave < hi:
            hi, upper = leave, row.proof()
    answer = dict(axes=len(rows), duration=duration, contact=None, interior=None)
    if stationary_miss:
        return dict(answer, kind="miss", proof={"constant_separating_axis": stationary_miss.proof()})
    if lo > hi:
        # Вся область покрыта t<lo и t>hi. Достаточно двух одномерных ограничений вместе с [0,T].
        return dict(answer, kind="miss", proof={"lower_time": lo, "lower_source": lower,
                                                "upper_time": hi, "upper_source": upper})

    def endpoint(t, bound):
        tight = [row for row in rows if not row.strict_at(t)]
        witness = next((row for row in tight if row.rate), tight[0] if tight else None)
        return dict(at=t, domain_start=t == 0, domain_end=t == duration, tight_axes=len(tight),
                    time_bound_source=bound, witness=witness.proof() if witness else None)

    left, right = endpoint(lo, lower), endpoint(hi, upper)
    inside_lo, inside_hi = not left["tight_axes"], not right["tight_axes"]
    answer.update(contact=interval(lo, hi), endpoints=[left, right])
    if stationary_touch:
        return dict(answer, kind="boundary_only", proof={"constant_touching_axis": stationary_touch.proof()})
    if lo == hi and not inside_lo:
        return dict(answer, kind="boundary_only", proof={"singleton": lo, "lower_source": lower, "upper_source": upper})
    # Нестационарная линейная форма строго внутри между двумя различными допустимыми концами.
    mid = (lo + hi) / 2
    slack = min(min(row.rate*mid-row.low, row.high-row.rate*mid) for row in rows)
    if slack <= 0:
        raise ArithmeticError("inconsistent strict-interior certificate")
    return dict(answer, kind="interior_overlap", interior=interval(lo, hi, inside_lo, inside_hi),
                proof={"interior_time": mid, "min_unnormalized_projection_slack": slack})


def load_geometry(path):
    shape = {"A": {}, "B": {}}
    with Path(path).open(encoding="utf-8-sig", newline="") as source:
        for row in csv.DictReader(source):
            vertices = shape[row["shape"]]
            p = tuple(Q(row[k]) for k in ("x_m", "y_m", "z_m"))
            i = int(row["vertex"])
            if vertices.setdefault(i, p) != p:
                raise ValueError(f"inconsistent coordinates for vertex {i}")
    return tuple(list(vertices.values()) for vertices in shape.values())


def fmt(obj):
    """JSON без float. Кортежи индексов и произвольно большие целые оси хранятся без округления."""
    return json.dumps(obj, default=lambda value: str(value) if isinstance(value, Q) else None,
                      sort_keys=True, ensure_ascii=False, indent=2, allow_nan=False)


def self_test(geometry=None):
    cube = list(product((-1, 1), repeat=3))
    results = {}

    def case(name, a, b, v, t, expected, strict):
        answer = solve(a, b, v, t)
        if answer["contact"] != expected or answer["interior"] != strict:
            raise AssertionError(f"{name}: {fmt(answer)}")
        results[name] = answer
        return answer

    def boxes(name, offset, v, t, expected, strict):
        return case(name, cube, [add(p, offset) for p in cube], v, t, expected, strict)

    hit = boxes("crossing", (4,0,0), (-1,0,0), 8, interval(2,6), interval(2,6,False,False))
    boxes("sliding_face", (4,2,0), (-1,0,0), 8, interval(2,6), None)
    boxes("sliding_edge", (4,2,2), (-1,0,0), 8, interval(2,6), None)
    boxes("singleton_grazing", (4,0,0), (-1,1,0), 8, interval(2,2), None)
    boxes("initial_boundary", (2,0,0), (1,0,0), 1, interval(0,0), None)
    boxes("final_boundary", (3,0,0), (-1,0,0), 1, interval(1,1), None)
    boxes("start_inside", (0,0,0), (1,0,0), 3, interval(0,2), interval(0,2,True,False))
    boxes("end_inside", (3,0,0), (-1,0,0), 2, interval(1,2), interval(1,2,False,True))
    boxes("zero_velocity_inside", (0,0,0), (0,0,0), 3, interval(0,3), interval(0,3))
    boxes("zero_velocity_face", (2,0,0), (0,0,0), 3, interval(0,3), None)
    boxes("zero_velocity_miss", (3,0,0), (0,0,0), 3, None, None)
    boxes("time_windows_disjoint", (4,1,0), (-1,1,0), 6, None, None)
    boxes("instant_inside", (0,0,0), (3,1,2), 0, interval(0,0), interval(0,0))
    boxes("instant_boundary", (2,0,0), (-1,0,0), 0, interval(0,0), None)
    boxes("between_60Hz_samples", (10,0,0), (-1000,0,0), 1,
          interval(Q(8,1000),Q(12,1000)), interval(Q(8,1000),Q(12,1000),False,False))
    case("permuted_bodies", [add(p,(4,0,0)) for p in reversed(cube)], cube, (1,0,0), 8, hit["contact"], hit["interior"])
    for scale in (Q(1,10**6), Q(10**6)):
        case(f"scale_{scale}", [mul(scale,p) for p in cube], [mul(scale,add(p,(4,0,0))) for p in cube],
             (-scale,0,0), 8, hit["contact"], hit["interior"])

    # Узкие непараллельные рёбра. Все нормали граней пропускают пару; разделяет только edge×edge.
    b = [add((0,Q(1003,1000),Q(1004,1000)), add(mul(k,(0,Q(2,5),Q(-3,10))),
         add(mul(i,(1,Q(3,10),Q(2,5))),mul(j,(-1,Q(3,10),Q(2,5))))))
         for k,i,j in product((-1,1),(0,1),(0,1))]
    skew = case("edge_cross_required", cube, b, (0,0,0), 1, None, None)
    if skew["proof"]["constant_separating_axis"]["source"][0] != "edge_cross" or any(
            not row.low <= 0 <= row.high for row in slabs(cube,b,(0,0,0)) if row.source[0] != "edge_cross"):
        raise AssertionError("edge-cross negative control lost")
    for bad in ([], [(0,0,0),(1,0,0),(0,1,0),(1,1,0)]):
        try:
            solve(bad,cube,(0,0,0),1)
        except ValueError:
            continue
        raise AssertionError("flat hull accepted")
    if geometry:
        a,b = load_geometry(geometry)
        b = [add((2,0,0), rotate(p,(1,2,3,4))) for p in b]
        trial = solve(a,b,(-1,0,0),4)
        if not trial["interior"] or not 0 < trial["interior"]["lo"] < 2 < trial["interior"]["hi"] < 4:
            raise AssertionError("CSV traversal missing")
        case("windows_geometry_swapped", b, a, (1,0,0), 4, trial["contact"], trial["interior"])
        results["windows_geometry"] = trial
    return dict(passed=True, cases=len(results), rejected_degenerate=2, results=results)


def components(text, count):
    values = tuple(Q(x) for x in text.split(","))
    if len(values) != count:
        raise ValueError(f"expected {count} components")
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--geometry", type=Path, help="CSV shape,vertex,x_m,y_m,z_m; shape names A/B")
    parser.add_argument("--velocity", default="0,0,0", help="relative B−A velocity, m/s")
    parser.add_argument("--offset", default="0,0,0", help="B origin relative to A at t=0, m")
    parser.add_argument("--quat-a", default="1,0,0,0", help="w,x,y,z, nonzero rational tuple")
    parser.add_argument("--quat-b", default="1,0,0,0")
    parser.add_argument("--duration", default="1", help="seconds, integer/decimal/fraction")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--verify", type=Path, help="recompute all slabs and compare a saved JSON certificate")
    args = parser.parse_args()
    if not args.geometry and not args.self_test:
        parser.error("--geometry or --self-test required")
    if args.self_test:
        answer = self_test(args.geometry)
    else:
        a,b = load_geometry(args.geometry)
        qa,qb = components(args.quat_a,4),components(args.quat_b,4)
        offset = components(args.offset,3)
        answer = solve([rotate(p,qa) for p in a], [add(offset,rotate(p,qb)) for p in b],
                       components(args.velocity,3), Q(args.duration))
    encoded = fmt(answer)
    if args.verify and json.loads(encoded) != json.loads(args.verify.read_text(encoding="utf-8-sig")):
        raise ValueError("certificate and recomputed projection inequalities differ")
    if args.output:
        args.output.write_text(encoded+"\n", encoding="utf-8")
    else:
        print(encoded)


if __name__ == "__main__":
    main()
