#!/usr/bin/env python3
"""No escape offered whose track ends on a pad of its own net that has no copper there (awx/group_ends.py:
loose_ends, array_rules, write_field).

  python3 tests/test_622_group_loose_end.py

The group's field is checked before it is written: a pad some track of its net ends on has one of them on a layer
it has copper on, or a via of the net standing on it -- a stub on any other layer alone joins its pad to nothing,
and no clearance check sees it. That check refuses the whole field, and no bus is routed. An array's other balls
are fanned out by the same menus as the buses' lanes, and a move that runs out on an inner layer to its exit can end
exactly where a surface pad of its own net stands (a test point beside the array). Such a move is not offered: the
array's rules ask each move the field's own question (loose_ends) of the other parts' pads of its net, a run keeps
only the layers that pad has copper on, and a move left with none is no option.

A pad here is its place and its layers; a track its two ends and its layer; a via its centre and its radius.

1. a track that ends on a pad with copper on its layer joins it;
2. one that ends on a surface pad from an inner layer joins nothing, and the pad and the layer are named;
3. a via of the net standing on the pad joins it;
4. a pad through every layer is joined on any;
5. two tracks ending at one pad, one of them on the pad's layer: joined (a run that turns under its own ball);
6. a ball over a pad of its own net on the other side is one place: either's layer joins it;
7. a track that passes a pad without ending on it is none of it;
8. the array's rules ask it of every move, a run keeping the layers that join; the field's check asks the same.
"""
import ast
import math
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = open(os.path.join(ROOT, 'awx', 'group_ends.py')).read()

BAD = []
N = [0]


def check(what, got, want):
    N[0] += 1
    if got != want:
        BAD.append(what)
    print(('ok   ' if got == want else 'FAIL ') + what + ('' if got == want else f': {got!r}, wanted {want!r}'))


fn = next(n for n in ast.parse(SRC).body if isinstance(n, ast.FunctionDef) and n.name == 'loose_ends')
ns = {'math': math}
exec(compile(ast.Module(body=[fn], type_ignores=[]), 'group_ends.py', 'exec'), ns)
loose_ends = ns['loose_ends']


def pad(x, y, *layers, name='P'):
    return types.SimpleNamespace(global_x=x, global_y=y, layers=list(layers), name=name)


def named(out):
    return [(p.name, ls) for p, ls in out]


TOP = pad(5.0, 0.0, 'F.Cu', 'F.Mask', name='TP')
check('1. a track that ends on a pad with copper on its layer joins it',
      named(loose_ends([TOP], [((0.0, 0.0), (5.0, 0.0), 'F.Cu')], [])), [])
check('2. one that ends on a surface pad from an inner layer joins nothing: the pad and the layer named',
      named(loose_ends([TOP], [((0.0, 0.0), (5.0, 0.0), 'In1.Cu')], [])), [('TP', ['In1.Cu'])])
check('3. a via of the net standing on the pad joins it',
      named(loose_ends([TOP], [((0.0, 0.0), (5.0, 0.0), 'In1.Cu')], [((5.0, 0.05), 0.125)])), [])
check('3. ...one beside the pad does not',
      named(loose_ends([TOP], [((0.0, 0.0), (5.0, 0.0), 'In1.Cu')], [((5.0, 0.3), 0.125)])), [('TP', ['In1.Cu'])])
check('4. a pad through every layer is joined on any',
      named(loose_ends([pad(5.0, 0.0, '*.Cu', '*.Mask')], [((0.0, 0.0), (5.0, 0.0), 'In1.Cu')], [])), [])
check('5. two tracks ending at one pad, one on the pad\'s layer: joined',
      named(loose_ends([TOP], [((0.0, 0.0), (5.0, 0.0), 'In1.Cu'), ((5.0, 0.0), (5.0, 1.0), 'F.Cu')], [])), [])
BALL, UNDER = pad(5.0, 0.0, 'F.Cu', name='BALL'), pad(5.0, 0.0, 'B.Cu', name='UNDER')
check('6. a ball over a pad of its own net on the other side: either\'s layer joins the place',
      named(loose_ends([BALL, UNDER], [((0.0, 0.0), (5.0, 0.0), 'B.Cu')], [])), [])
check('6. ...and a layer neither has does not',
      named(loose_ends([BALL, UNDER], [((0.0, 0.0), (5.0, 0.0), 'In2.Cu')], [])), [('BALL', ['In2.Cu'])])
check('7. a track that passes a pad without ending on it is none of it',
      named(loose_ends([TOP], [((0.0, 0.0), (9.0, 0.0), 'In1.Cu')], [])), [])

rules = SRC[SRC.index('def array_rules(pcb, A):'):SRC.index('    A.dead = dead')]
check('8. the array\'s rules ask it of every move: a run keeps the layers that join',
      'joined = lambda L: not loose_ends(pads_, [(a, b, L if Lg == je.RUN else Lg)' in rules
      and 'and joined(L))' in rules, True)
check('8. ...of the other parts\' pads of its net (the array\'s own balls are lanes here, joined by their own copper)',
      "and p_.component_ref != A.ref:" in rules, True)
check('8. ...and a move on one layer that does not join is no option',
      'elif not joined(None):\n                dead.add((k, i))' in rules, True)
field = SRC[SRC.index('def write_field('):]
check('8. the field\'s check before it is written asks the same question',
      'in loose_ends(pads_.get(nid_, ()), by_net[nid_],' in field
      and 'whose only stub runs on a layer the pad has no' in field, True)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
