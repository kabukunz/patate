#!/usr/bin/python3


##################

import sys
from sys import argv
from math import cos, sin, pi
from random import uniform

##################

def lerp(alpha, p0, p1):
	return [ (1-alpha)*c0 + alpha*c1 for c0, c1 in zip(p0, p1) ]

def rand_vec(fact):
	return ( uniform(-fact, fact), uniform(-fact, fact) )

def vec_sum(a, b):
	return ( x+y for x, y in zip(a, b) )

##################

if len(argv) == 3:
	filename = argv[2]
	nrings = int(argv[1])
	noise = 0.
elif len(argv) == 4:
	filename = argv[3]
	nrings = int(argv[1])
	noise = float(argv[2])
else:
	print("Usage:", argv[0], "N_RINGS [NOISE] FILENAME")
	exit(1)


out = open(filename, 'w')
sys.stdout = out

print("mvg 1.0")
print("attributes none")
print("v", *rand_vec(noise))

for r in range(1, nrings+1):
	for tri in range(6):
		alpha = [ tri * pi/3, (tri+1) * pi/3 ]
		p = [ [ cos(alpha[i]) * r, sin(alpha[i]) * r ] for i in range(2) ]
		for v in range(r):
			coords = vec_sum(lerp(v / r, p[0], p[1]), rand_vec(noise))
			print("v", *coords)

inner = 0
outer = 1
for r in range(1, nrings+1):
	first_inner = inner
	first_outer = outer
	for tri in range(6):
		for v in range(r-1):
			next_inner = inner + 1 if tri != 5 or v != r-2 else first_inner
			print("f", inner, outer, outer+1)
			print("f", inner, outer+1, next_inner)
			inner += 1
			outer += 1
		if tri == 5:
			print("f", first_inner, outer, first_outer)
		else:
			print("f", inner, outer, outer + 1)
		outer += 1
	if r == 1:
		inner += 1

#			print("f", inner+v, outer+v, outer+v+1)
#			print("f", inner+v, outer+v+1, inner+v+1)
#		print("f", inner+r-1, outer+r-1, outer+r)

print("pgc 0", .5/nrings, "0 0 0 0", .5/nrings, "0 0")
vx = 1 + 3 * nrings * (nrings - 1)
ring_size = nrings * 6
print("c ",
      ' '.join(map(lambda x: str(x + vx) + ' ' + str(x / ring_size), range(ring_size))),
      ' ', str(vx), ' 1', sep='')
grad = [
           '0 1    0.5   0 1',
    '0.166666 0.75 0.933 0 1',
    '0.333333 0.25 0.933 0 1',
          '.5 0    0.5   0 1',
    '0.666666 0.25 0.067 0 1',
    '0.833333 0.75 0.067 0 1',
           '1 1    0.5   0 1',
]
print("dcv 0", ' '.join(grad))
