import sys
import bpy

if __name__ == "__main__":
    bpy.ops.object.delete(use_global=False)
    #bpy.ops.mesh.primitive_torus_add(layers=(True, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False), view_align=False, location=(0, 0, 0), rotation=(0, 0, 0), major_radius=1, minor_radius=0.25, abso_major_rad=1.25, abso_minor_rad=0.75)
    # Input path
    bpy.ops.import_scene.obj(filepath=str(sys.argv[0]))
    bpy.ops.uv.smart_project()
    #out path plus ext
    bpy.ops.uv.export_layout(filepath=str(sys.argv[1]), size=(1024, 1024))
