# Authors /Box3DUnreal/SM_Box3DLiquidSphere: a low-poly 100 cm sphere for the
# liquid particles. The engine's BasicShapes sphere is ~2.9k triangles — dense
# geometry x hundreds of instances x translucent overdraw is real GPU money for
# blobs a few centimeters on screen. This one is a few hundred triangles.
#
# Rerun with:
#   UnrealEditor-Cmd <project> -run=pythonscript -script=Tools/make_liquid_sphere_mesh.py
#     "-EnablePlugins=PythonScriptPlugin,GeometryScripting"
import unreal

ASSET_PATH = "/Box3DUnreal/SM_Box3DLiquidSphere"


def lib(*candidates):
    """GeometryScript libraries are exposed under ScriptName aliases that vary
    by engine version; try the known spellings."""
    for name in candidates:
        if hasattr(unreal, name):
            return getattr(unreal, name)
    available = sorted(n for n in dir(unreal) if n.startswith("GeometryScript_"))
    raise RuntimeError(f"none of {candidates} found; available: {available}")


primitives = lib("GeometryScript_Primitives", "GeometryScriptLibrary_MeshPrimitiveFunctions")
normals = lib("GeometryScript_Normals", "GeometryScriptLibrary_MeshNormalsFunctions")
new_assets = lib("GeometryScript_NewAssetUtils", "GeometryScriptLibrary_CreateNewAssetFunctions")

if unreal.EditorAssetLibrary.does_asset_exist(ASSET_PATH):
    unreal.EditorAssetLibrary.delete_asset(ASSET_PATH)

mesh = unreal.new_object(unreal.DynamicMesh)
prim_options = unreal.GeometryScriptPrimitiveOptions()
# Box-sphere: quad topology, even triangle distribution; steps 6 -> 6*6*6*2 =
# 432 triangles.
primitives.append_sphere_box(
    mesh, prim_options, unreal.Transform(), radius=50.0,
    steps_x=6, steps_y=6, steps_z=6)

# Smooth shading — blobs, not disco balls.
normals.set_per_vertex_normals(mesh)

options = unreal.GeometryScriptCreateNewStaticMeshAssetOptions()
options.enable_collision = False
options.enable_nanite = False
result = new_assets.create_new_static_mesh_asset_from_mesh(mesh, ASSET_PATH, options)
unreal.log(f"create outcome: {result}")

saved = unreal.EditorAssetLibrary.save_asset(ASSET_PATH, only_if_is_dirty=False)
unreal.log(f"SM_Box3DLiquidSphere saved: {saved}")
assert saved, "failed to save liquid sphere mesh"
