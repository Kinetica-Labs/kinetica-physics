"""Offline exact separation certificate for two vertex hulls, no dependencies.

python rotation_certificate.py --self-test
python rotation_certificate.py path.json --max-depth 8 --max-nodes 127

JSON has keys A and B. Each holds vertices (metres), faces (optional index loops),
p0, p1 (world translations in metres), q0, q1 (world quaternions w,x,y,z).
Coordinates may be integers, decimal literals or strings like "1/3".
The certified path linearly interpolates p and the four supplied quaternion
coordinates before projective normalization. Their signs and scales matter.
Advance only if status == SAFE (exit 0). UNKNOWN (exit 1) and invalid input
(exit 2) never authorize the step. Each SAFE leaf supplies its time interval,
an unnormalized rational world axis, and a strict lower bound on axis dot (B-A).
"""
import argparse
import json
from dataclasses import dataclass
from fractions import Fraction as F
from itertools import product
from pathlib import Path


def vector(values, size):
    if len(values) != size or any(type(x) not in (int, str, F) for x in values):
        raise ValueError(f"expected {size} rational coordinates")
    return tuple(F(x) for x in values)


def add(x, y):
    return tuple(a + b for a, b in zip(x, y))


def sub(x, y):
    return tuple(a - b for a, b in zip(x, y))


def scale(s, x):
    return tuple(s * a for a in x)


def dot(x, y):
    return sum(a * b for a, b in zip(x, y))


def cross(x, y):
    a, b, c = x
    d, e, f = y
    return b*f-c*e, c*d-a*f, a*e-b*d


def lerp(x, y, t):
    return add(x, scale(t, sub(y, x)))


def rotate_numerator(q, v):
    # R(q)v = v + 2 (w u×v + u×(u×v)) / (q·q), u = (x,y,z).
    u = q[1:]
    uv = cross(u, v)
    return add(scale(dot(q, q), v), scale(2, add(scale(q[0], uv), cross(u, uv))))


@dataclass(frozen=True)
class Motion:
    vertices: tuple
    faces: tuple
    p0: tuple
    p1: tuple
    q0: tuple
    q1: tuple

    @classmethod
    def read(cls, data):
        p0, p1 = (vector(data[k], 3) for k in ('p0', 'p1'))
        q0, q1 = (vector(data[k], 4) for k in ('q0', 'q1'))
        d0, d1, c = dot(q0,q0), dot(q1,q1), dot(q0,q1)
        if not d0 or not d1 or (c < 0 and c*c == d0*d1):
            raise ValueError("quaternion segment contains zero")
        vertices = tuple(vector(v, 3) for v in data['vertices'])
        if not vertices:
            raise ValueError("empty vertex hull")
        faces = tuple(tuple(face) for face in data.get('faces', ()))
        if any(len(face) < 3 or any(type(i) is not int or not 0 <= i < len(vertices)
                                    for i in face) for face in faces):
            raise ValueError("invalid face indices")
        return cls(vertices, faces, p0, p1, q0, q1)

    def points(self, t):
        q, p = lerp(self.q0, self.q1, t), lerp(self.p0, self.p1, t)
        return [add(p, scale(1 / dot(q,q), rotate_numerator(q,v))) for v in self.vertices]

    def enclosure(self, a, b):
        # The numerator and denominator of the rotated vertex are quadratics.
        # In the Bernstein basis their middle coefficients are the polarized
        # quadratic forms. Positive denominator coefficients turn the three
        # numerator/weight ratios into a convex enclosure at every time.
        q, r = lerp(self.q0, self.q1, a), lerp(self.q0, self.q1, b)
        weights = dot(q,q), dot(q,r), dot(r,r)
        if min(weights) <= 0:
            return []  # Subdivision can restore positive weights.
        positions = (lerp(self.p0, self.p1, a), lerp(self.p0, self.p1, b))
        enclosure = []
        for v in self.vertices:
            first, last = rotate_numerator(q,v), rotate_numerator(r,v)
            middle = scale(F(1,2), sub(sub(rotate_numerator(add(q,r),v), first), last))
            for numerator, weight in zip((first,middle,last), weights):
                rotated = scale(1/weight, numerator)
                enclosure.extend(add(p, rotated) for p in positions)
        return enclosure


def suggested_axes(points, faces):
    edges, normals = [], []
    for face in faces:
        normals.append(cross(sub(points[face[1]], points[face[0]]),
                             sub(points[face[2]], points[face[0]])))
        edges.extend(sub(points[j], points[i]) for i,j in zip(face, face[1:] + face[:1]))
    return normals, edges


def axes_at_middle(a, b, pa, pb):
    na, ea = suggested_axes(pa, a.faces)
    nb, eb = suggested_axes(pb, b.faces)
    axes = [(1,0,0),(0,1,0),(0,0,1)] + na + nb
    axes.extend(cross(i,j) for i in ea for j in eb)
    seen = set()
    for axis in axes:
        pivot = next((v for v in axis if v), None)
        if pivot is None:
            continue
        axis = scale(F(1)/pivot, axis)
        if axis not in seen:
            seen.add(axis)
            yield axis


def gap(axis, a, b):
    return min(dot(axis, p) for p in b) - max(dot(axis, p) for p in a)


def certify(first, second, max_depth=8, max_nodes=127):
    """A positive separating gap on every closed leaf covers the entire step."""
    a, b = Motion.read(first), Motion.read(second)
    if max_depth < 0 or max_nodes < 1:
        raise ValueError("expected nonnegative depth and positive node budget")
    work = [(F(0), F(1), 0)]
    leaves, visited = [], 0
    while work and visited < max_nodes:
        left, right, depth = work.pop()
        visited += 1
        mid = (left+right)/2
        pa, pb = a.points(mid), b.points(mid)
        candidates = []
        for axis in axes_at_middle(a,b,pa,pb):
            if gap(axis,pa,pb) > 0:
                candidates.append(axis)
            elif gap(axis,pb,pa) > 0:
                candidates.append(scale(-1,axis))
        if not candidates:
            return {'status':'UNKNOWN', 'reason':'no strict midpoint separator', 'nodes':visited}
        ca, cb = a.enclosure(left,right), b.enclosure(left,right)
        if ca and cb:
            for axis in candidates:
                bound = gap(axis,ca,cb)
                if bound > 0:
                    leaves.append({'t':[str(left),str(right)], 'axis':list(map(str,axis)),
                                   'projected_gap_lower':str(bound)})
                    break
            else:
                bound = 0
            if bound > 0:
                continue
        if depth >= max_depth:
            return {'status':'UNKNOWN', 'reason':'depth budget', 'nodes':visited}
        work.extend([(mid,right,depth+1),(left,mid,depth+1)])
    if work:
        return {'status':'UNKNOWN', 'reason':'node budget', 'nodes':visited}
    return {'status':'SAFE', 'nodes':visited, 'leaves':leaves}


def self_test():
    ident = (1,0,0,0)
    # A long prism and an asymmetric tetrahedron; both have nonzero volume.
    prism = dict(vertices=list(product((-2,2),('-1/4','1/4'),('-1/5','1/5'))),
                 faces=[[0,1,3,2],[4,5,7,6],[0,1,5,4],[2,3,7,6],[0,2,6,4],[1,3,7,5]],
                 p0=(0,0,0),p1=('1/8',0,0),q0=ident,q1=(0,0,0,1))
    tetra = dict(vertices=[('1/6',0,0),(0,'1/8',0),(0,0,'1/7'),('-1/8','-1/8','-1/8')],
                 faces=[[0,1,2],[0,1,3],[0,2,3],[1,2,3]],p0=(0,1,2),p1=(0,'5/4',2),
                 q0=ident,q1=(1,1,0,0))
    def check(name, first, second, expected, **limits):
        answer = certify(first, second, **limits)
        if answer['status'] != expected:
            raise AssertionError((name, expected, answer))
        if expected == 'SAFE':
            end = F(0)
            for leaf in answer['leaves']:
                left, right = map(F, leaf['t'])
                if left != end or right <= left or F(leaf['projected_gap_lower']) <= 0:
                    raise AssertionError((name, leaf))
                end = right
            if end != 1:
                raise AssertionError((name, "incomplete interval"))
        print(f"{name},{expected},{answer['nodes']}")
    check('two_rotating_separated',prism,tetra,'SAFE')
    check('positive_gap_budget_exhausted',prism,tetra,'UNKNOWN',max_nodes=1)
    crossing = dict(tetra,p0=(0,1,0),p1=(0,'5/4',0))
    for side in ('0','1'):
        def stationary(body):
            return dict(body, p0=body['p'+side],p1=body['p'+side],q0=body['q'+side],q1=body['q'+side])
        check('clear_endpoint_'+side,stationary(prism),stationary(crossing),'SAFE')
    # At mid-time A's long x axis points along world y. B's origin is interior
    # to both solids there: its prism-local coordinates are (9/8, 1/16, 0).
    check('endpoints_clear_interior_crossing',prism,crossing,'UNKNOWN')
    still_a = dict(prism,p1=prism['p0'],q1=ident)
    touch_b = dict(tetra,p0=(0,'3/8',0),p1=(0,'3/8',0),q1=ident)
    check('stationary_touch',still_a,touch_b,'UNKNOWN')
    try:
        Motion.read(dict(prism, q1=(-2,0,0,0)))
    except ValueError as error:
        if str(error) != 'quaternion segment contains zero':
            raise
    else:
        raise AssertionError("zero quaternion accepted")
    print('zero_quaternion_path,INVALID,0')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', nargs='?', type=Path)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--max-depth', type=int, default=8)
    parser.add_argument('--max-nodes', type=int, default=127)
    args = parser.parse_args()
    if args.self_test:
        self_test()
    elif args.input:
        try:
            data = json.loads(args.input.read_text(encoding='utf-8'), parse_float=F)
            result = certify(data['A'], data['B'], args.max_depth, args.max_nodes)
        except (OSError, ValueError, TypeError, KeyError, ZeroDivisionError) as error:
            parser.exit(2, f"INVALID: {error}\n")
        print(json.dumps(result))
        raise SystemExit(0 if result['status'] == 'SAFE' else 1)
    else:
        parser.error('supply input.json or --self-test')
