#!/usr/bin/python3


##################

import sys
from sys import argv
from math import cos, sin, pi

##################

def lerp(alpha, p0, p1):
	return [ (1-alpha)*c0 + alpha*c1 for c0, c1 in zip(p0, p1) ]

##################

if len(argv) != 3:
	print("Usage:", argv[0], "N_RINGS FILENAME")
	exit(1)

filename = argv[2]
nrings = int(argv[1])


out = open(filename, 'w')
sys.stdout = out

print("mvg 1.0")
print("attributes none")
print("v 0 0")

for r in range(1, nrings+1):
	for tri in range(6):
		alpha = [ tri * pi/3, (tri+1) * pi/3 ]
		p = [ [ cos(alpha[i]) * r, sin(alpha[i]) * r ] for i in range(2) ]
		for v in range(r):
			coords = lerp(v / r, p[0], p[1])
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

print("pc x o;", .5/nrings, "0 0 0 0", .5/nrings, "0 0; 0")
vx = 1 + 3 * nrings * (nrings - 1)
ring_size = nrings * 6
grad = [
           '0:1,.5,0,1',
    '0.166666:.75,.933,0,1',
    '0.333333:.25,.933,0,1',
          '.5:0,.5,0,1',
    '0.666666:.25,.067,0,1',
    '0.833333:.75,.067,0,1',
           '1:1,.5,0,1',
]
print("dc o x; ", ' '.join(grad), "; ",
	' '.join(map(lambda x: str(x + vx) + ':' + str(x / ring_size), range(ring_size))),
	' ', str(vx), ':1', sep='')
