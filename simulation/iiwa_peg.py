import math
from pathlib import Path

from manipulation.utils import ConfigureParser
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    MeshcatVisualizer,
    Parser,
    RigidTransform,
    StartMeshcat,
)
from pydrake.multibody.parsing import (
    LoadModelDirectivesFromString,
    ProcessModelDirectives,
)

ASSEMBLY = """
directives:
- add_directives:
    file: package://manipulation/iiwa_and_wsg.dmd.yaml

- add_model:
    name: peg
    file: package://project_assets/peg.sdf

- add_weld:
    parent: wsg::body
    child: peg::peg_base
    X_PC:
        translation: [0.0, 0.11, 0.0]
        rotation: !Rpy {deg: [-90.0, 0.0, 0.0]}

- add_model:
    name: table
    file: package://manipulation/hydro/extra_heavy_duty_table_surface_only_collision.sdf
- add_weld:
    parent: world
    child: table::link
    X_PC:
        translation: [0.5, 0.0, -0.7645]
"""

def add_socket(plant, parser):
    """Weld a 20 mm tall socket with an 11 mm bore onto the table."""
    inner_radius = 0.0055
    outer_radius = 0.020
    height = 0.020
    segment_count = 32

    thickness = outer_radius - inner_radius
    center_radius = inner_radius + thickness / 2
    width = 2 * outer_radius * math.tan(math.pi / segment_count)

    elements = []
    for i in range(segment_count):
        yaw = 2 * math.pi * i / segment_count
        x = center_radius * math.cos(yaw)
        y = center_radius * math.sin(yaw)

        box = f"""
          <pose>{x} {y} 0 0 0 {yaw}</pose>
          <geometry>
            <box><size>{thickness} {width} {height}</size></box>
          </geometry>
        """
        elements.append(f'<visual name="wall_{i}">{box}</visual>')
        elements.append(f'<collision name="wall_{i}">{box}</collision>')

    sdf = f"""
    <sdf version="1.7">
      <model name="socket">
        <link name="body">
          {"".join(elements)}
        </link>
      </model>
    </sdf>
    """
    socket = parser.AddModelsFromString(sdf, "sdf")[0]
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("body", socket),
        RigidTransform([0.55, 0.0, height / 2]),
    )

def build_assembly(meshcat=None):
    """Build the Drake diagram and return it alongside the plant."""
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=0.001
    )

    parser = Parser(plant)
    ConfigureParser(parser)
    assets = Path(__file__).resolve().parents[1] / "assets"
    parser.package_map().Add("project_assets", str(assets))

    directives = LoadModelDirectivesFromString(ASSEMBLY)
    ProcessModelDirectives(directives, parser)
    add_socket(plant, parser)
    plant.Finalize()

    if meshcat is not None:
        MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    return builder.Build(), plant


def main():
    meshcat = StartMeshcat()
    diagram, _ = build_assembly(meshcat)
    context = diagram.CreateDefaultContext()
    diagram.ForcedPublish(context)

    print(f"Open Meshcat: {meshcat.web_url()}")
    input("Press Enter to close Meshcat. ")


if __name__ == "__main__":
    main()
