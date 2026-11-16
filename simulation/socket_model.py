"""Load the socket and weld its entrance frame to the scene."""

from pathlib import Path

from pydrake.all import Parser


def add_socket(plant, parent_frame, X_PS):
    """Attach the socket at X_PS relative to parent_frame before Finalize().

    The socket body origin is the center of the mouth and +Z points inward.
    """
    model_path = Path(__file__).resolve().parents[1] / "assets" / "socket.sdf"
    model_instance = Parser(plant).AddModels(str(model_path))[0]
    socket_frame = plant.GetFrameByName("socket_body", model_instance)
    plant.WeldFrames(parent_frame, socket_frame, X_PS)
    return model_instance
