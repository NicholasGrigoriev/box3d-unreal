# Authors /Box3DUnreal/M_Box3DLiquid: a translucent water material for the
# liquid source's instanced particles. Parameterized so users can MID-tune it
# into any fluid (colors, opacity, roughness). PerInstanceCustomData slot 0 is
# the particle's normalized age (fed by the source actor) and fades opacity.
import unreal

PACKAGE_PATH = "/Box3DUnreal"
ASSET_NAME = "M_Box3DLiquid"
ASSET_PATH = f"{PACKAGE_PATH}/{ASSET_NAME}"

if unreal.EditorAssetLibrary.does_asset_exist(ASSET_PATH):
    unreal.EditorAssetLibrary.delete_asset(ASSET_PATH)

tools = unreal.AssetToolsHelpers.get_asset_tools()
mat = tools.create_asset(ASSET_NAME, PACKAGE_PATH, unreal.Material, unreal.MaterialFactoryNew())
assert mat is not None, "failed to create material asset"

mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_TRANSLUCENT)
mat.set_editor_property("translucency_lighting_mode",
                        unreal.TranslucencyLightingMode.TLM_SURFACE_PER_PIXEL_LIGHTING)
mat.set_editor_property("used_with_instanced_static_meshes", True)

MEL = unreal.MaterialEditingLibrary

def expr(klass, x, y):
    return MEL.create_material_expression(mat, klass, x, y)

def scalar_param(name, value, x, y):
    node = expr(unreal.MaterialExpressionScalarParameter, x, y)
    node.set_editor_property("parameter_name", name)
    node.set_editor_property("default_value", value)
    return node

# --- Base color: deep-to-shallow tint driven by fresnel -----------------------
deep = expr(unreal.MaterialExpressionVectorParameter, -800, -300)
deep.set_editor_property("parameter_name", "DeepColor")
deep.set_editor_property("default_value", unreal.LinearColor(0.004, 0.045, 0.09, 1.0))

shallow = expr(unreal.MaterialExpressionVectorParameter, -800, -100)
shallow.set_editor_property("parameter_name", "ShallowColor")
shallow.set_editor_property("default_value", unreal.LinearColor(0.05, 0.32, 0.45, 1.0))

fresnel = expr(unreal.MaterialExpressionFresnel, -800, 150)
fresnel.set_editor_property("exponent", 3.0)
fresnel.set_editor_property("base_reflect_fraction", 0.04)

color_lerp = expr(unreal.MaterialExpressionLinearInterpolate, -500, -200)
MEL.connect_material_expressions(deep, "", color_lerp, "A")
MEL.connect_material_expressions(shallow, "", color_lerp, "B")
MEL.connect_material_expressions(fresnel, "", color_lerp, "Alpha")
MEL.connect_material_property(color_lerp, "", unreal.MaterialProperty.MP_BASE_COLOR)

# --- Surface response ---------------------------------------------------------
rough = scalar_param("Roughness", 0.08, -500, 20)
MEL.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)

spec = scalar_param("Specular", 0.26, -500, 120)  # water F0 ~= 0.02
MEL.connect_material_property(spec, "", unreal.MaterialProperty.MP_SPECULAR)

# --- Opacity: fresnel-boosted body, faded by particle age ---------------------
op_base = scalar_param("BaseOpacity", 0.5, -800, 320)
op_edge = scalar_param("EdgeOpacity", 0.9, -800, 420)
op_lerp = expr(unreal.MaterialExpressionLinearInterpolate, -500, 320)
MEL.connect_material_expressions(op_base, "", op_lerp, "A")
MEL.connect_material_expressions(op_edge, "", op_lerp, "B")
MEL.connect_material_expressions(fresnel, "", op_lerp, "Alpha")

age = expr(unreal.MaterialExpressionPerInstanceCustomData, -800, 550)
age.set_editor_property("data_index", 0)
age_amount = scalar_param("AgeFade", 0.35, -800, 650)
age_mul = expr(unreal.MaterialExpressionMultiply, -600, 570)
MEL.connect_material_expressions(age, "", age_mul, "A")
MEL.connect_material_expressions(age_amount, "", age_mul, "B")
age_fade = expr(unreal.MaterialExpressionOneMinus, -450, 570)
MEL.connect_material_expressions(age_mul, "", age_fade, "")

opacity = expr(unreal.MaterialExpressionMultiply, -280, 420)
MEL.connect_material_expressions(op_lerp, "", opacity, "A")
MEL.connect_material_expressions(age_fade, "", opacity, "B")
MEL.connect_material_property(opacity, "", unreal.MaterialProperty.MP_OPACITY)

MEL.recompile_material(mat)
ok = unreal.EditorAssetLibrary.save_asset(ASSET_PATH, only_if_is_dirty=False)
unreal.log(f"M_Box3DLiquid saved: {ok}")
assert ok, "failed to save material asset"
