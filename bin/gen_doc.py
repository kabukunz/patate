#!/usr/bin/python3

from sys import argv, stderr
from os.path import dirname, realpath, normpath, join, isdir, exists
from jinja2 import Environment, FileSystemLoader


def simplify(string):
	return string.replace(' ', '_').lower()


script_dir = dirname(realpath(__file__))

env = Environment(loader=FileSystemLoader(join(script_dir, "templates")))
tmpl = env.get_template("doc.tmpl")


if len(argv) != 4:
	print("Usage: {} MODULE NAME TITLE".format(argv[0]))
	exit(1)

module = argv[1]
name   = argv[2]
title  = argv[3]

base_name = "{}_{}".format(simplify(module), simplify(name))

out_dir = normpath(join(script_dir, '..', 'Patate', module))
if not isdir(out_dir):
	print("\"{}\" is not a directory.".format(out_dir))
	exit(1)
out_filename = "{}.mdoc".format(base_name).lower()
out_path = join(out_dir, out_filename)
if exists(out_path):
	print("\"{}\" already exists.".format(out_path))
	exit(1)

params = {
	'pagename' : "{}_page".format(base_name),
	'pagetitle': title
}
with open(out_path, "w") as out:
	out.write(tmpl.render(params))
