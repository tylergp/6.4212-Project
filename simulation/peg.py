"""Load the peg and rigidly attach its base to an existing robot tool frame."""

from pathlib import Path

from pydrake.all import Parser


def add_peg(plant, tool_frame):
    """Return the peg model instance; call before plant.Finalize().

    The tool frame coincides with peg_base. The tip extends 50 mm along
    tool +Z. The plant must already be connected to a SceneGraph.
    """
    model_path = Path(__file__).resolve().parents[1] / "assets" / "peg.sdf"
    model_instance = Parser(plant).AddModels(str(model_path))[0]
    plant.WeldFrames(
        tool_frame, plant.GetFrameByName("peg_base", model_instance)
    )
    return model_instance
