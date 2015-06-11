#!/usr/bin/python3

from numpy import (
	array,
	identity,
	dot,
	concatenate,
)
from numpy.linalg import (
	norm,
)


MvgLoaderError = RuntimeError


TO_VALUE_FLAG      = 1
FROM_VALUE_FLAG    = 2
EDGE_VALUE_FLAG    = 4
EDGE_GRADIENT_FLAG = 8


def always_true(*args, **kwargs):
	return True


class Face:
	def __init__(self):
		self.verts  = []
		self.v_from = []
		self.v_to   = []
		self.v_edge = []
		self.g_edge = []

	@staticmethod
	def parse_tokens(tokens, attrs):

		def parse_node(tok):
			if tok == "x":
				return None
			return int(tok)

		face = Face()

		tok_it = iter(tokens)
		for tok in tok_it:
			if tok == "-":
				break

			sub = tok.split('/')
			face.verts.append(int(sub[0]))
			if len(sub) > 1:
				face.v_to.append(parse_node(sub[1]))
				face.v_from.append(parse_node(sub[-1]))
			else:
				face.v_to.append(None)
				face.v_from.append(None)

		for tok in tok_it:
			sub = tok.split('/')
			i = 0
			if attrs & EDGE_VALUE_FLAG:
				face.v_edge.append(parse_node(sub[i]))
				i += 1
			if attrs & EDGE_GRADIENT_FLAG:
				face.g_edge.append(parse_node(sub[i]))
				i += 1

		return face


class Edge:
	def __init__(self, points, edge=None):
		self.points  =  points
		self.edge    =  edge


class PlacedNode:
	def __init__(self, p, type_, v=None, attr=None, vx=None, edge=None):
		self.p     =  p
		self.type  =  type_
		self.v     =  v
		self.attr  =  attr
		self.vx    =  vx
		self.edge  =  edge


class MVG:
	def __init__(self):
		"""Creates an empty mesh."""
		self.attrs    =  0
		self.ndims    =  0
		self.ncoeffs  =  0
		self.verts    =  []
		self.nodes    =  []
		self.faces    =  []

	@staticmethod
	def load_mvg(filename):
		"""Loads an mvg file."""

		in_file = open(filename)
		line_it = iter(in_file)

		line = next(line_it).strip()
		line_no = 1

		def error(msg):
			raise MvgLoaderError("{}:{}: {}\n>> \"{}\"".format(filename, line_no, msg, line))
		def error_if(cond, msg):
			if cond: error(msg)

		error_if(line != "mvg 1.0", "Bad header")

		mvg = MVG()

		for line in line_it:
			line_no += 1
			line = line.strip()

			if len(line) == 0 or line[0] == "#":
				continue

			tokens = line.split()
			if   tokens[0] == "attributes":
				error_if(len(tokens) != 2, "Wrong number of arguments")
				if   tokens[1] == "none":
					mvg.attrs = 0
				elif tokens[1] == "linear":
					mvg.attrs = TO_VALUE_FLAG | FROM_VALUE_FLAG
				elif tokens[1] == "quadratic":
					mvg.attrs = TO_VALUE_FLAG | FROM_VALUE_FLAG | EDGE_VALUE_FLAG
				elif tokens[1] == "morley":
					mvg.attrs = TO_VALUE_FLAG | FROM_VALUE_FLAG | EDGE_GRADIENT_FLAG
				elif tokens[1] == "fv":
					mvg.attrs = TO_VALUE_FLAG | FROM_VALUE_FLAG | EDGE_VALUE_FLAG | EDGE_GRADIENT_FLAG
				else:
					error("Unsupported attributes")
			elif tokens[0] == "dimensions":
				error_if(len(tokens) != 2, "Wrong number of arguments")
				mvg.ndims = int(tokens[1])
			elif tokens[0] == "coefficients":
				error_if(len(tokens) != 2, "Wrong number of arguments")
				mvg.ncoeffs = int(tokens[1])
			elif tokens[0] == "v":
				error_if(len(tokens) != mvg.ndims+1, "Wrong number of arguments")
				mvg.verts.append(array(tokens[1:], float))
			elif tokens[0] == "n":
				if len(tokens) == 2 and tokens[1] == "void":
					mvg.nodes.append(None)
				else:
					error_if(len(tokens) != mvg.ncoeffs+1, "Wrong number of arguments")
					mvg.nodes.append(array(tokens[1:], float))
			elif tokens[0] == "f":
				mvg.faces.append(Face.parse_tokens(tokens[1:], mvg.attrs))

		return mvg

	def edges(self):
		edges = set()
		for f in self.faces:
			for i0 in range(len(f.verts)):
				i1 = (i0+1) % len(f.verts)
				if f.verts[i1] < f.verts[i0]:
					i0, i1 = i1, i0
				e = (f.verts[i0], f.verts[i1])
				if e not in edges:
					edges.add(e)
					yield Edge((self.verts[e[0]], self.verts[e[1]]), edge=e)

	def placed_nodes(self, radius, offset):
		def node(p, n, **kwargs):
			if n is None: return PlacedNode(p, "invalid", **kwargs)
			v = self.nodes[n]
			if v is None: return PlacedNode(p, "unknown", n, **kwargs)
			return PlacedNode(p, "constraint", v, **kwargs)

		for f in self.faces:
			for i0 in range(len(f.verts)):
				i1 = (i0+1) % len(f.verts)
				p0 = self.verts[f.verts[i0]]
				p1 = self.verts[f.verts[i1]]
				v = p1 - p0
				v /= norm(v)
				n = array([v[1], -v[0]], float)
				if TO_VALUE_FLAG & self.attrs:
					yield node(p1 - v*radius + n*offset, f.v_to[i1],
					           attr=TO_VALUE_FLAG, vx=i1)
				if FROM_VALUE_FLAG & self.attrs:
					yield node(p0 + v*radius + n*offset, f.v_from[i0],
					           attr=FROM_VALUE_FLAG, vx=i0)
				if EDGE_VALUE_FLAG & self.attrs:
					yield node((p0 + p1) / 2 + n*offset, f.v_edge[i0],
					           attr=EDGE_VALUE_FLAG, edge=(i0, i1))
