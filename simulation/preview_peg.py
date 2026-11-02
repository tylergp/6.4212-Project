"""Preview the peg in Meshcat; run with python simulation/preview_peg.py."""

from pathlib import Path

from pydrake.all import ModelVisualizer, StartMeshcat


def main():
    model_path = Path(__file__).resolve().parents[1] / "assets" / "peg.sdf"
    meshcat = StartMeshcat()
    print(f"Open the peg preview: {meshcat.web_url()}", flush=True)
    visualizer = ModelVisualizer(meshcat=meshcat)
    visualizer.AddModels(filename=str(model_path))
    visualizer.Run()


if __name__ == "__main__":
    main()
