# Simulation

Drake scene construction, robot/connector attachment, socket placement, cameras,
contact settings, and reproducible episode reset logic belong here.

First target: ideal insertion plus deliberate rim collision, both with contact
force and seating-depth logs. Keep policy training outside the simulation.

## Preview the peg

In a Python environment with Drake installed (Ubuntu/WSL on Windows), run
from the repository root:

```bash
python simulation/preview_peg.py
```

Open the printed Meshcat URL in your browser. Stop with the Meshcat
"Stop Running" button or Ctrl+C. This previews geometry; it does not simulate
contact or insertion. The asset path is resolved relative to the script, so
the script can also be launched from another working directory.

## Attach to a robot

Once the scene contains a robot and a SceneGraph, use the attachment helper
before finalizing the plant:

```python
from simulation.peg import add_peg

# tool_frame is the attachment frame from your loaded robot model.
peg_model = add_peg(plant, tool_frame)
peg_tip = plant.GetFrameByName("peg_tip", peg_model)
plant.Finalize()
```

The peg base is welded to the tool frame. Its tip lies 50 mm along tool +Z.
The helper does not load a robot or add a controller.
