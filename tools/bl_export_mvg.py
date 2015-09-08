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
    # If there is one, start at an extremity (nedges == 1) else start at
    # an arbitrary curve vertex (this mean that open curves are
    # processed first, then closed curves.
    for vx, elist in vmap.items():
        nedges = len(elist)
        if nedges > 0:
            curr_vx = vx
            if nedges == 1: break

    if curr_vx is None:
        return 0

    # Create an ordered list of the edges of the curve
    path = []
    while True:
        elist = vmap[curr_vx]
        if len(elist) == 0: break
        edge = elist.pop()

        path.append(edge)

        curr_vx = edge.other_vert(curr_vx)
        vmap[curr_vx].remove(edge)

    # Write the curve
    total_len = sum(map(lambda e: e.calc_length(), path))
    accum = 0
    fw("c %d 0" % (curr_vx.index))
    for edge in reversed(path):
        accum += edge.calc_length()
        curr_vx = edge.other_vert(curr_vx)
        fw(" %d %f" % (curr_vx.index, accum / total_len))
    fw("\n")

    return len(path)


def write_mvg_curves(fw, mesh):
    edges = [ e for e in mesh.edges if e.seam ]

    if len(edges) == 0:
        print("No edge selected.")
        return 0

    vmap = {}
    for edge in edges:
        for vx in edge.verts:
            elist = vmap.setdefault(vx, [])
            elist.append(edge)

    count = 0
    while write_path(fw, vmap) > 0:
        count += 1

    return count


def write_mvg(context, filepath, triangulate = True, random_values = True):
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

    # Write header
    fw("mvg 1.0\n")
    fw("dimensions 3\n")
    fw("coefficients 4\n")
    fw("attributes none\n")
    fw("colorSpace srgb\n")

    # Write mesh
    for v in mesh.verts:
        fw("v %f %f %f\n" % (v.co.x, v.co.z, -v.co.y))

    for f in mesh.faces:
        if triangulate:
            verts = f.verts
            for i in range(2, len(verts)):
                fw("f %d %d %d\n" % (verts[0].index, verts[i-1].index, verts[i].index))
        else:
            fw("f")
            for v in f.verts:
                fw(" %d" % (v.index))
            fw("\n")

    # Write curves
    ncurves = write_mvg_curves(fw, mesh)

    if random_values:
        col_splits = [ 2, 2, 2 ]
        while col_splits[0] * col_splits[1] * col_splits[2] < ncurves:
            if col_splits[2] < col_splits[1]:
                col_splits[2] += 1
            elif col_splits[1] < col_splits[0]:
                col_splits[1] += 1
            else:
                col_splits[0] += 1

        color = [ 0, 0, 0 ]
        for i in range(ncurves):
            fw("dcv %d 0 %f %f %f 1\n" % (i, color[0] / (col_splits[0] - 1),
                                             color[1] / (col_splits[1] - 1),
                                             color[2] / (col_splits[2] - 1)))
            color[0] += 1
            if color[0] == col_splits[0]:
                color[0] = 0
                color[1] += 1
                if color[1] == col_splits[1]:
                    color[1] = 0
                    color[2] += 1

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

    triangulate = BoolProperty(
            name="Triangulate",
            description="Export triangulated mesh. Recommended because currently Vitelotte supports only triangulated mesh.",
            default=True
            )

    random_values = BoolProperty(
            name="Random values",
            description="Export random color for each curve.",
            default=True
            )

    def execute(self, context):
        return write_mvg(context, self.filepath, self.triangulate,
                         self.random_values)


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
