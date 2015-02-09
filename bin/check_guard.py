#!/usr/bin/python3

from sys import argv
from os.path import basename, dirname, splitext
from re import compile
from io import StringIO

guard_re = compile(r'#ifndef\s+(.*)')
guard_re2 = compile(r'#define\s+(.*)')

def extract_guard(filename):
	f = open(filename)
	it = iter(f)
	for line in it:
		m = guard_re.match(line)
		if m:
			nl = next(it)
			m2 = guard_re2.match(nl)
			if not m2 or m2.group(1) != m.group(1):
				print('\033[32mError:\033[0m file "{}" has inconsistent include guard.'.format(filename))
			return m.group(1)
	return None

def print_all_caps_from_camel_case(toprint, out):
	skip = True
	for c in toprint:
		if c.isupper() and not skip:
			out.write('_')
			skip = True
		else:
			skip = False
		out.write(c.upper())

def guard_from_filename(filename):
	guard = StringIO('')
	guard.write('_')
	
	d = dirname(filename)
	if d == 'Patate':
		guard.write('PATATE')
	elif d.startswith('Patate/common/'):
		print_all_caps_from_camel_case(d.replace('/', '_'), guard)		
	else:
		print_all_caps_from_camel_case(basename(dirname(d)), guard)

	guard.write('_')
	
	name, ext = splitext(basename(filename))
	print_all_caps_from_camel_case(name, guard)

	guard.write('_')
	
	return guard.getvalue()

current = extract_guard(argv[1])
theorical = guard_from_filename(argv[1])

if current == theorical:
	print('\033[32m{}\033[0m: {}'.format(argv[1], current))
else:
	print('\033[31m{}\033[0m: {} - {}'.format(argv[1], current, theorical))
