# A blender plugin to export the selected mesh a a .mvg.
# Edges marked as "seams" will be turned to curves.

import bpy
import bmesh
import random


bl_info = {
    "name": "MVG Export",
    "author": "Simon Boyé",
    "version": (1, 0, 0),
    "blender": (2, 70, 0),
    "location": "File > Export",
    "description": "Export the active mesh as a .mvg, turnig seams to curves.",
    "warning": "",
    "wiki_url": "",
    "category": "Import-Export"}


def write_path(fw, vmap):
    curr_vx = None
    for vx, elist in vmap.items():
        nedges = len(elist)
        if nedges > 0:
            curr_vx = vx
            if nedges == 1: break

    if curr_vx is None:
        return 0

    path = []
    while True:
#        print("from %d" % (curr_vx.index))
        elist = vmap[curr_vx]
        if len(elist) == 0: break
        edge = elist.pop()

        path.append(edge)
#        print("edge %d %d" % (edge.verts[0].index, edge.verts[1].index))

        curr_vx = edge.other_vert(curr_vx)
        vmap[curr_vx].remove(edge)
#        print("to %d" % (curr_vx.index))

    total_len = sum(map(lambda e: e.calc_length(), path))
    accum = 0
    fw("c %d 0" % (curr_vx.index))
#    print("v", curr_vx.index)
    for edge in reversed(path):
        accum += edge.calc_length()
        curr_vx = edge.other_vert(curr_vx)
        fw(" %d %f" % (curr_vx.index, accum / total_len))
#        print("v", curr_vx.index)
    fw("\n")

    return len(path)


def write_mvg_curves(fw, mesh):
    edges = [ e for e in mesh.edges if e.seam ]

    if len(edges) == 0:
        print("No edge selected.")
        return {'CANCELLED'}

    vmap = {}
    for edge in edges:
        for vx in edge.verts:
            elist = vmap.setdefault(vx, [])
            elist.append(edge)

#           print(vmap)
#        for e in edges:
#            print("e %d %d" % (e.verts[0].index, e.verts[1].index))
#        for v, el in vmap.items():
#            print("v %d:" % (v.index))
#            for e in el:
#                print("  e %d %d" % (e.verts[0].index, e.verts[1].index))

    count = 0
    while write_path(fw, vmap) > 0:
        count += 1

    return count


def write_mvg(context, filepath):
    mesh_data = context.active_object.data
    if type(mesh_data) is not bpy.types.Mesh:
        return {'CANCELED'}

    file = open(filepath, 'w', encoding='utf-8')
    fw = file.write

    if context.mode == 'EDIT_MESH':
        mesh = bmesh.from_edit_mesh(mesh_data)
    else:
        mesh = bmesh.new()
        mesh.from_mesh(mesh_data)

    fw("mvg 1.0\n")
    fw("dim 3\n")
    fw("coefficients 4\n")
    fw("attributes none\n")

    for v in mesh.verts:
        fw("v %f %f %f\n" % v.co.yzx[:])
    for f in mesh.faces:
        fw("f")
        for v in f.verts:
            fw(" %d" % (v.index))
        fw("\n")

    ncurves = write_mvg_curves(fw, mesh)

    random.seed()
    for i in range(ncurves):
        fw("dcv %d 0 %f %f %f 1\n" % (i, random.random(), random.random(), random.random()))

    file.close()

    return {'FINISHED'}


# ExportHelper is a helper class, defines filename and
# invoke() function which calls the file selector.
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy.types import Operator


class ExportMvg(Operator, ExportHelper):
    """Export mvg file"""
    bl_idname = "export_mesh.mvg"
    bl_label = "Export Mvg"

    # ExportHelper mixin class uses this
    filename_ext = ".mvg"

    filter_glob = StringProperty(
            default="*.mvg",
            options={'HIDDEN'},
            )

    random_values = BoolProperty(
            name="Random values",
            description="Export random color for each curve.",
            default=True
            )

    def execute(self, context):
        return write_mvg(context, self.filepath)


# Only needed if you want to add into a dynamic menu
def menu_func_export(self, context):
    self.layout.operator(ExportMvg.bl_idname, text="Mesh-based vector graphics (.mvg)")


def register():
    bpy.utils.register_class(ExportMvg)
    bpy.types.INFO_MT_file_export.append(menu_func_export)


def unregister():
    bpy.utils.unregister_class(ExportMvg)
    bpy.types.INFO_MT_file_export.remove(menu_func_export)


if __name__ == "__main__":
    register()

    # test call
    bpy.ops.export_mesh.mvg('INVOKE_DEFAULT')
