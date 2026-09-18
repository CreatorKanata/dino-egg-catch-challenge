"""Render the exported STL parts with VTK depth testing for an accurate CAD preview."""

import argparse
import json
import tempfile
from pathlib import Path

import vtk

from .config import Config, MountConfig
from .geometry import make_fingertip, slab, cylinder
from .mount import make_mount


def label(renderer, text, x, y, size=18):
    actor = vtk.vtkTextActor()
    actor.SetInput(text)
    actor.GetPositionCoordinate().SetCoordinateSystemToNormalizedViewport()
    actor.SetPosition(x, y)
    actor.GetTextProperty().SetFontSize(size)
    actor.GetTextProperty().SetColor(0.10, 0.20, 0.28)
    renderer.AddActor2D(actor)


def render(folder):
    folder = Path(folder)
    c = Config(**json.loads((folder / "validation.json").read_text())["parameters"])
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(1600, 900)
    window.SetMultiSamples(8)
    with tempfile.TemporaryDirectory() as tmp:
        # A true CAD section exposes the enclosed cavity without modifying the print file.
        half = slab(c.width * 2, c.depth, c.height * 2).translate((0, c.depth / 2, 0))
        cutaway = make_fingertip(c).intersect(half)
        section_path = Path(tmp) / "section.stl"
        cutaway.val().exportStl(str(section_path), tolerance=c.stl_tolerance, relative=False)
        magnet = cylinder(c.magnet_diameter, c.magnet_height, c.contact_skin).intersect(half)
        magnet_path = Path(tmp) / "magnet.stl"
        magnet.val().exportStl(str(magnet_path), tolerance=c.stl_tolerance, relative=False)
        mount_report = folder / "mount-draft.json"
        references = []
        if mount_report.exists():
            m = MountConfig(**json.loads(mount_report.read_text())["parameters"])
            base = make_mount(m).translate((0, 0, c.height))
            board_z = c.height + m.plate + m.standoff
            pcb = slab(m.board_width, m.board_depth, m.board_thickness, board_z)
            for x, y in m.hole_centers:
                pcb = pcb.cut(cylinder(m.screw_clearance, m.board_thickness, board_z)
                              .translate((x, y, 0)))
            for name, solid, color in (("base", base, (0.60, 0.63, 0.67)),
                                       ("board", pcb, (0.27, 0.41, 0.78))):
                path = Path(tmp) / f"{name}.stl"
                solid.intersect(half).val().exportStl(str(path), relative=False)
                references.append((path, color))
        for i, section in enumerate((False, True)):
            renderer = vtk.vtkRenderer()
            renderer.SetViewport(i / 2, 0, (i + 1) / 2, 1)
            renderer.SetBackground(0.96, 0.97, 0.98)
            window.AddRenderer(renderer)
            items = [(section_path if section else folder / "fingertip.stl", (0.22, 0.66, 0.63))]
            if section:
                items.append((magnet_path, (0.78, 0.25, 0.18)))
                items.extend(references)
            for path, color in items:
                reader = vtk.vtkSTLReader()
                reader.SetFileName(str(path))
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputConnection(reader.GetOutputPort())
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                # Show the mounted orientation: contact up, fixed sensor below.
                actor.SetOrientation(180, 0, 0)
                actor.SetPosition(0, 0, c.height)
                actor.GetProperty().SetColor(*color)
                renderer.AddActor(actor)
            camera = renderer.GetActiveCamera()
            camera.SetPosition(30, 65, 35)
            camera.SetFocalPoint(0, 0, 1 if section else c.height / 2)
            camera.SetViewUp(0, 0, 1)
            camera.ParallelProjectionOn()
            camera.SetParallelScale(27 if section else 19)
            renderer.ResetCameraClippingRange()
            label(renderer, "SECTION / DRAFT ASSEMBLY" if section else "CROSSED LATTICE / V3",
                  0.06, 0.93, 24)
            label(renderer, "Red: magnet | Gray: rigid base | Blue: PCB reference" if section else
                  f"{c.width:g} x {c.depth:g} x {c.height:g} mm | Contact face UP in this view", 0.06, 0.88, 18)
            label(renderer, f"Wall {c.pocket_wall:g} mm | Floor {c.contact_skin:g} mm | Roof {c.pocket_roof:g} mm" if section else
                  "Both skins and depth connections are integral TPU", 0.06, 0.13, 18)
            label(renderer, "Board outline and hole locations require a physical fit check" if section else
                  "STL prints upside down: contact face on the bed", 0.06, 0.08, 16)
        window.Render()
        capture = vtk.vtkWindowToImageFilter()
        capture.SetInput(window)
        capture.ReadFrontBufferOff()
        capture.Update()
        writer = vtk.vtkPNGWriter()
        writer.SetFileName(str(folder / "preview.png"))
        writer.SetInputConnection(capture.GetOutputPort())
        writer.Write()
        window.Finalize()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    render(parser.parse_args().folder)
