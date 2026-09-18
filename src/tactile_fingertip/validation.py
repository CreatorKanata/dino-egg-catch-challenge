"""Validate one solid with one intentional enclosed cavity, including insert access."""

import hashlib
from math import cos, sin, tau

import numpy as np
import trimesh

from .geometry import at_pause, cylinder, pocket


def require(ok, message):
    if not ok:
        raise ValueError(message)


def surface_groups(mesh):
    parents = list(range(len(mesh.faces)))

    def root(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i

    for a, b in mesh.face_adjacency:
        parents[root(int(a))] = root(int(b))
    groups = {}
    for i in range(len(parents)):
        groups.setdefault(root(i), []).append(i)
    return list(groups.values())


def validate_geometry(c, body):
    shape = body.val()
    box = shape.BoundingBox()
    tol = c.geometry_tolerance
    require(shape.isValid() and len(shape.Solids()) == 1, "Expected one valid TPU solid")
    require(len(shape.Shells()) == 2, "Expected exterior plus one closed pocket surface")
    require(np.allclose((box.xlen, box.ylen, box.zlen), (c.width, c.depth, c.height),
                        atol=tol, rtol=0) and abs(box.zmin) < tol, "Wrong print envelope")
    cavity = pocket(c).val()
    require(shape.intersect(cavity).Volume() < tol, "Pocket obstructed")
    filled = shape.fuse(cavity)
    require(len(filled.Shells()) == 1, "Pocket is not the sole enclosed void")
    require(abs(filled.Volume() - shape.Volume() - cavity.Volume()) < tol, "Pocket volume mismatch")
    for i in range(16):
        angle = tau * i / 16
        radius = c.pocket_diameter / 2 + c.pocket_wall / 2
        require(shape.isInside((radius * cos(angle), radius * sin(angle), c.magnet_center_z)),
                "Missing radial TPU wall")
    require(shape.isInside((0, 0, c.contact_skin / 2)), "Missing floor")
    require(shape.isInside((0, 0, c.pause_z + c.pocket_roof / 2)), "Missing roof")
    pre = at_pause(c, body).val()
    access = cylinder(c.magnet_diameter, c.height, c.contact_skin).val()
    require(pre.isValid() and len(pre.Solids()) == 1 and len(pre.Shells()) == 1,
            "Pause geometry is not a single open-pocket solid")
    require(pre.intersect(access).Volume() < tol, "Magnet cannot enter at pause")
    return {"cad_valid": True, "cad_solids": 1, "cad_surface_shells": 2,
            "size_mm": [box.xlen, box.ylen, box.zlen], "cad_volume_mm3": shape.Volume(),
            "pocket_volume_mm3": cavity.Volume(), "pause_access": "passed"}


def validate_export(c, body, stl):
    mesh = trimesh.load_mesh(stl, process=True)
    require(mesh.is_watertight and mesh.is_winding_consistent, "STL is open or inconsistently oriented")
    groups = surface_groups(mesh)
    require(len(groups) == 2, "STL must have exterior and cavity surfaces")
    volumes = sorted(float(mesh.submesh([ids], append=True).volume) for ids in groups)
    require(volumes[0] < 0 < volumes[1] and mesh.volume > 0, "Invalid inner/outer surface orientation")
    require(abs(-volumes[0] - pocket(c).val().Volume()) / pocket(c).val().Volume()
            < c.mesh_volume_relative_tolerance, "STL cavity size mismatch")
    require(abs(mesh.volume - body.val().Volume()) / body.val().Volume()
            < c.mesh_volume_relative_tolerance, "STL/CAD volume mismatch")
    require(np.allclose(mesh.extents, (c.width, c.depth, c.height), atol=c.stl_tolerance, rtol=0),
            "STL dimensions mismatch")
    return {"stl_watertight": True, "stl_surface_shells": 2, "surface_signed_volumes_mm3": volumes,
            "triangles": len(mesh.faces),
            "stl_sha256": hashlib.sha256(stl.read_bytes()).hexdigest()}
