#!/usr/bin/env python3
"""plot_goals.py <goals.csv> <out.svg>: goals the node sent in the CI launch test (GOALS_CSV) on the test map:
exploration bounds, robot at the origin with the min-distance circle, the occupied block with the
0.35 m clearance band. Standard library only."""
import sys

src, out = sys.argv[1], sys.argv[2]
lines = open(src).read().splitlines()
h = lines[0].lstrip('# ').split()
bound, min_d = float(h[1]), float(h[3])
bx0, bx1, by0, by1, clr = map(float, (h[5], h[6], h[7], h[8], h[10]))
goals = [tuple(map(float, l.split(','))) for l in lines[2:] if l]
lo, hi = -bound - 0.6, bound + 0.6
W = 480
s = W / (hi - lo)
H = W + 44


def X(x):
    return (x - lo) * s


def Y(y):
    return (hi - y) * s


el = [f'<rect width="{W}" height="{H}" fill="#ffffff"/>']
for v in range(int(lo), int(hi) + 1):
    el.append(f'<line x1="{X(v):.1f}" y1="0" x2="{X(v):.1f}" y2="{W}" stroke="#eef1f3"/>'
              f'<line x1="0" y1="{Y(v):.1f}" x2="{W}" y2="{Y(v):.1f}" stroke="#eef1f3"/>')
el.append(f'<rect x="{X(-bound):.1f}" y="{Y(bound):.1f}" width="{2 * bound * s:.1f}" height="{2 * bound * s:.1f}" '
          f'fill="none" stroke="#005f73" stroke-dasharray="6 4"/>')
el.append(f'<text x="{X(-bound) + 4:.1f}" y="{Y(bound) - 6:.1f}" fill="#005f73">exploration_bounds ±{bound:g} m</text>')
el.append(f'<rect x="{X(bx0) - clr * s:.1f}" y="{Y(by1) - clr * s:.1f}" width="{(bx1 - bx0 + 2 * clr) * s:.1f}" '
          f'height="{(by1 - by0 + 2 * clr) * s:.1f}" rx="{clr * s:.1f}" fill="#f6e7e2"/>')
el.append(f'<rect x="{X(bx0):.1f}" y="{Y(by1):.1f}" width="{(bx1 - bx0) * s:.1f}" height="{(by1 - by0) * s:.1f}" fill="#a2624a"/>')
el.append(f'<text x="{X(bx1) + clr * s + 4:.1f}" y="{Y(by0) + 4:.1f}" fill="#7a4632">occupied + {clr} m clearance</text>')
el.append(f'<circle cx="{X(0):.1f}" cy="{Y(0):.1f}" r="{min_d * s:.1f}" fill="none" stroke="#9aa5b1" stroke-dasharray="3 3"/>')
el.append(f'<circle cx="{X(0):.1f}" cy="{Y(0):.1f}" r="5" fill="#10161e"/>')
el.append(f'<text x="{X(0) - 8:.1f}" y="{Y(0) + 18:.1f}" text-anchor="end" fill="#3f4d58">robot</text>')
for x, y in goals:
    el.append(f'<circle cx="{X(x):.1f}" cy="{Y(y):.1f}" r="4.5" fill="#005f73" fill-opacity="0.85"/>')
el.append(f'<text x="8" y="{W + 18}" fill="#3f4d58">{len(goals)} goals sent by the node in the CI launch test (known 10 x 10 m map,</text>'
          f'<text x="8" y="{W + 34}" fill="#3f4d58">stand-in Nav2). Dashed circle: min_goal_distance {min_d:g} m. Metres, map frame.</text>')
svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" '
       f'font-family="Helvetica, Arial, sans-serif" font-size="12">\n' + '\n'.join(el) + '\n</svg>\n')
open(out, 'w').write(svg)
print(f'{out}: {len(goals)} goals')
