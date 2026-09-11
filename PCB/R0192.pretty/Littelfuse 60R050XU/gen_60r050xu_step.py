#!/usr/bin/env python3
"""Write a simplified STEP (AP214, faceted B-rep) model of the Littelfuse 60R050XU PPTC.

Coordinates in mm, origin = pad 1, Z up, board top surface at z = 0.
Datasheet (60R050X): A max 7.9 (disc dia), B max 13.7 (height), C 5.1 (lead spacing),
E max 3.1 (thickness), lead dia 0.51, lead offset 1.2 (matches footprint pad 2 at 5.1/1.2).
"""
import math
import sys

OUT = sys.argv[1]
NAME = "Fuse_Littelfuse_60R050XU"

DISC_D = 7.9
THICK = 3.1
HEIGHT = 13.7
PITCH = 5.1
OFFSET = 1.2
LEAD_D = 0.51
LEAD_BELOW = 3.0          # lead protrusion below board (KiCad convention ~3 mm)
LEAD_INTO_BODY = 1.7      # how far leads reach into the disc
DISC_SEG = 40
LEAD_SEG = 8

BODY_RGB = (0.88, 0.62, 0.12)   # yellow/orange epoxy coating
LEAD_RGB = (0.78, 0.78, 0.80)   # tinned copper

ents = []


def add(s):
    ents.append(s)
    return len(ents)


def f(v):
    s = "%.6f" % v
    s = s.rstrip("0")
    return s if not s.endswith(".") else s + "0" if False else s


def pt(p):
    return add("CARTESIAN_POINT('',(%s,%s,%s))" % tuple(f(c) for c in p))


def solid(faces, name):
    """faces: list of point lists, ordered counter-clockwise seen from outside."""
    face_ids = []
    for poly in faces:
        # Newell normal -> outward plane normal; first edge -> reference direction
        nx = ny = nz = 0.0
        for i in range(len(poly)):
            x1, y1, z1 = poly[i]
            x2, y2, z2 = poly[(i + 1) % len(poly)]
            nx += (y1 - y2) * (z1 + z2)
            ny += (z1 - z2) * (x1 + x2)
            nz += (x1 - x2) * (y1 + y2)
        ln = math.sqrt(nx * nx + ny * ny + nz * nz)
        n = (nx / ln, ny / ln, nz / ln)
        ex = [poly[1][k] - poly[0][k] for k in range(3)]
        le = math.sqrt(sum(c * c for c in ex))
        ex = [c / le for c in ex]
        axis = add("AXIS2_PLACEMENT_3D('',#%d,#%d,#%d)" % (
            pt(poly[0]),
            add("DIRECTION('',(%s,%s,%s))" % tuple(f(c) for c in n)),
            add("DIRECTION('',(%s,%s,%s))" % tuple(f(c) for c in ex))))
        plane = add("PLANE('',#%d)" % axis)
        pids = [pt(p) for p in poly]
        loop = add("POLY_LOOP('',(%s))" % ",".join("#%d" % i for i in pids))
        bound = add("FACE_OUTER_BOUND('',#%d,.T.)" % loop)
        face_ids.append(add("FACE_SURFACE('',(#%d),#%d,.T.)" % (bound, plane)))
    shell = add("CLOSED_SHELL('',(%s))" % ",".join("#%d" % i for i in face_ids))
    return add("FACETED_BREP('%s',#%d)" % (name, shell))


def prism(section, a, b, axis):
    """Extrude a CCW polygon (2D in the plane normal to axis) from a to b along axis."""
    def p3(u, v, w):
        if axis == "z":
            return (u, v, w)
        if axis == "y":  # section given in (x, z)
            return (u, w, v)
        raise ValueError(axis)

    n = len(section)
    lo = [p3(u, v, a) for u, v in section]
    hi = [p3(u, v, b) for u, v in section]
    # For axis y the (x,z) -> (x,y,z) mapping flips handedness, so reverse caps.
    flip = axis == "y"
    top = hi if not flip else list(reversed(hi))
    bot = list(reversed(lo)) if not flip else lo
    faces = [top, bot]
    for i in range(n):
        j = (i + 1) % n
        q = [lo[i], lo[j], hi[j], hi[i]]
        faces.append(q if not flip else list(reversed(q)))
    return faces


def circle(cx, cy, r, seg, phase=0.0):
    return [(cx + r * math.cos(phase + 2 * math.pi * k / seg),
             cy + r * math.sin(phase + 2 * math.pi * k / seg)) for k in range(seg)]


# KiCad footprint Y points down, 3D model Y points up -> pad 2 (5.1 / +1.2) is at model y = -1.2
MODEL_OFFSET_Y = -OFFSET

# --- body: disc in the XZ plane, thickness along Y --------------------------
cx = PITCH / 2.0
cy = MODEL_OFFSET_Y / 2.0
cz = HEIGHT - DISC_D / 2.0
disc = circle(cx, cz, DISC_D / 2.0, DISC_SEG)
body = solid(prism(disc, cy - THICK / 2.0, cy + THICK / 2.0, "y"), "body")

# --- leads: straight octagonal prisms ------------------------------------------
r_lead = LEAD_D / 2.0 / math.cos(math.pi / LEAD_SEG)  # octagon with flat-to-flat = LEAD_D
top_z = HEIGHT - DISC_D + LEAD_INTO_BODY
leads = []
for idx, (lx, ly) in enumerate(((0.0, 0.0), (PITCH, MODEL_OFFSET_Y))):
    sec = circle(lx, ly, r_lead, LEAD_SEG, math.pi / LEAD_SEG)
    leads.append(solid(prism(sec, -LEAD_BELOW, top_z, "z"), "lead%d" % (idx + 1)))

# --- units / context ----------------------------------------------------------
mm = add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )")
rad = add("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )")
sr = add("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )")
unc = add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#%d,'distance_accuracy_value','confusion accuracy')" % mm)
ctx = add("( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#%d)) "
          "GLOBAL_UNIT_ASSIGNED_CONTEXT((#%d,#%d,#%d)) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') )"
          % (unc, mm, rad, sr))

origin = add("AXIS2_PLACEMENT_3D('',#%d,#%d,#%d)" % (
    pt((0, 0, 0)), add("DIRECTION('',(0.,0.,1.))"), add("DIRECTION('',(1.,0.,0.))")))

items = [origin, body] + leads
shape_rep = add("FACETED_BREP_SHAPE_REPRESENTATION('%s',(%s),#%d)" % (
    NAME, ",".join("#%d" % i for i in items), ctx))

app_ctx = add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')")
add("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#%d)" % app_ctx)
prod_ctx = add("PRODUCT_CONTEXT('',#%d,'mechanical')" % app_ctx)
prod = add("PRODUCT('%s','%s','',(#%d))" % (NAME, NAME, prod_ctx))
add("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#%d))" % prod)
pdf = add("PRODUCT_DEFINITION_FORMATION('','',#%d)" % prod)
pdc = add("PRODUCT_DEFINITION_CONTEXT('part definition',#%d,'design')" % app_ctx)
pd = add("PRODUCT_DEFINITION('design','',#%d,#%d)" % (pdf, pdc))
pds = add("PRODUCT_DEFINITION_SHAPE('','',#%d)" % pd)
add("SHAPE_DEFINITION_REPRESENTATION(#%d,#%d)" % (pds, shape_rep))


# --- colours ------------------------------------------------------------------
def style(rgb):
    col = add("COLOUR_RGB('',%s,%s,%s)" % tuple(f(c) for c in rgb))
    fasc = add("FILL_AREA_STYLE_COLOUR('',#%d)" % col)
    fas = add("FILL_AREA_STYLE('',(#%d))" % fasc)
    ssfa = add("SURFACE_STYLE_FILL_AREA(#%d)" % fas)
    sss = add("SURFACE_SIDE_STYLE('',(#%d))" % ssfa)
    ssu = add("SURFACE_STYLE_USAGE(.BOTH.,#%d)" % sss)
    return add("PRESENTATION_STYLE_ASSIGNMENT((#%d))" % ssu)


body_style = style(BODY_RGB)
lead_style = style(LEAD_RGB)
styled = [add("STYLED_ITEM('color',(#%d),#%d)" % (body_style, body))]
styled += [add("STYLED_ITEM('color',(#%d),#%d)" % (lead_style, l)) for l in leads]
add("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',(%s),#%d)" % (
    ",".join("#%d" % i for i in styled), ctx))

with open(OUT, "w") as fh:
    fh.write("ISO-10303-21;\nHEADER;\n")
    fh.write("FILE_DESCRIPTION(('%s simplified model'),'2;1');\n" % NAME)
    fh.write("FILE_NAME('%s.step','2026-09-11T12:00:00',(''),(''),'gen_60r050xu_step.py','gen_60r050xu_step.py','');\n" % NAME)
    fh.write("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n")
    for i, e in enumerate(ents, 1):
        fh.write("#%d=%s;\n" % (i, e))
    fh.write("ENDSEC;\nEND-ISO-10303-21;\n")
print("wrote", OUT, len(ents), "entities")
