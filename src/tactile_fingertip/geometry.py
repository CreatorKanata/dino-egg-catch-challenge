"""One TPU solid with an enclosed magnet cavity; no separate holder or cover."""

import cadquery as cq

from .config import Config


def slab(width, depth, height, z=0):
    return cq.Workplane("XY").box(width, depth, height, centered=(True, True, False)).translate((0, 0, z))


def cylinder(diameter, height, z=0):
    return cq.Workplane("XY").circle(diameter / 2).extrude(height).translate((0, 0, z))


def diamond(radius, depth):
    points = [(0, -radius), (radius, 0), (0, radius), (-radius, 0)]
    return cq.Workplane("XZ").polyline(points).close().extrude(depth / 2, both=True)


def pocket(c):
    return cylinder(c.pocket_diameter, c.pocket_depth, c.contact_skin)


def rib_bank(c):
    """Supported planar diamonds; a rotated copy creates depth connections."""
    body = slab(c.width, c.depth, c.contact_skin)
    outer = diamond(c.cell_radius, c.rib_depth)
    inner = diamond(c.inner_radius, c.rib_depth + 2 * c.geometry_tolerance)
    ring = outer.cut(inner)
    for x in c.centers(c.cell_count, c.cell_pitch):
        for y in c.centers(c.rib_count, c.rib_pitch):
            cell = ring.translate((x, y, c.cell_center_z))
            # Flat mounting feet grow from supported diamond shoulders.
            foot = slab(c.mount_land, c.rib_depth, c.height - c.land_bottom,
                        c.land_bottom).translate((x, y, 0))
            cell = cell.union(foot)
            # An arbitrarily cropped ring can start its upper outer branch in air.
            # Keep an inward half-ring instead; its turn remains inside the print.
            if x - c.cell_radius < -c.width / 2:
                cell = cell.intersect(slab(c.width, c.depth, c.height)
                                      .translate((x + c.width / 2, 0, 0)))
            elif x + c.cell_radius > c.width / 2:
                cell = cell.intersect(slab(c.width, c.depth, c.height)
                                      .translate((x - c.width / 2, 0, 0)))
            body = body.union(cell)
    # The contact skin defines the exact footprint.
    body = body.intersect(slab(c.width, c.depth, c.height))
    return body


def make_fingertip(c: Config):
    c.validate()
    bank = rib_bank(c)
    body = bank.union(bank.rotate((0, 0, 0), (0, 0, 1), 90))
    # Mid-height ties join the crossed banks without relying on either skin.
    # Their short horizontal gaps still need a TPU bridge trial in the slicer.
    for offset in (-c.rib_pitch, c.rib_pitch):
        tie = slab(c.tie_span, c.rib_depth, c.tie_thickness,
                   c.cell_center_z - c.tie_thickness / 2).translate((0, offset, 0))
        body = body.union(tie).union(tie.rotate((0, 0, 0), (0, 0, 1), 90))
    # A continuous backing distributes fixture loads across both rib directions.
    body = body.union(slab(c.width, c.depth, c.mounting_skin,
                           c.height - c.mounting_skin))
    # This local wall is fused to the contact skin, not a separate component.
    body = body.union(cylinder(c.enclosure_diameter, c.enclosure_top))
    return body.cut(pocket(c)).clean()


def at_pause(c, body):
    """The printed portion before any roof deposition; used to check insertion."""
    return body.intersect(slab(c.width, c.depth, c.pause_z)).clean()
