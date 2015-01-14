#!/usr/bin/python3

from sys import argv
from re import compile
from io import StringIO

license_begin_re = compile(r'\s*/\*[* \t]*')
license_end_re = compile(r'[* \t]*\*/\s*')
license_line_re = compile(r'[* \t]*(.*)')

copyright_re = compile(r'Copyright \(C\)\s+(\d+)\s+(.*)')
copyright_mail_re = compile(r'^(.*)<(.*)>$')

def extract_license_info(filename):
	f = open(filename)
	it = iter(f)
	
	try:
		begin_line = 1
		while not license_begin_re.match(next(it)):
			begin_line += 1
	except StopIteration:
		return None, None, None

	license_text = StringIO()
	end_line = begin_line
	for line in it:
		end_line += 1
		if license_end_re.match(line):
			break
		license_text.write(license_line_re.match(line).group(1).rstrip())
		license_text.write('\n')
	
	return license_text.getvalue()[:-1], begin_line, end_line

def split_copyright_license(bloc):
	lines = bloc.splitlines()
	
	m = copyright_re.match(lines[0])
	if not m:
		return bloc, None, None

	
	year = int(m.group(1))
	name_mail = m.group(2).rstrip()
	
	first_line = 2
	for line in lines[first_line:]:
		if line != '':
			break
		first_line += 1
		
	return '\n'.join(lines[first_line:]), year, name_mail

def print_indented(text, indent='  '):
	for line in text.splitlines():
		print(indent, line, sep='')
		

mpl2 = '''This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.'''

filename = argv[1]
license_bloc, first_line, last_line = extract_license_info(filename)
if license_bloc:
	license_text, c_year, c_names = split_copyright_license(license_bloc)
else:
	license_text, c_year, c_names = None, None, None

on_first_line = first_line == 1
license_ok = license_text == mpl2
if on_first_line and license_ok:
	print('\033[32m{}\033[0m: Copyright: {} {}'.format(argv[1], c_year, c_names))
#	print_indented(license_text)
else:
	print('\033[31m{}\033[0m: Copyright: {} {}'.format(argv[1], c_year, c_names))
	if license_bloc:
		if not on_first_line:
			print('  License bloc does not start on first line.')
		# do not print license if not first line because chances are it
		# is some random comment (likely doc).
		elif not license_ok:
			print('  License seems wrong:')
			print_indented(license_text, '    ')
	else:
		print('  License not found.')
