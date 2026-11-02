# Robotic Electrical Connector Assembly

MIT 6.4212 final project comparing classical force-feedback control with
behavior cloning for connector insertion in Drake. Vision is the primary
extension after the state-based system works.

## Repository layout

| Folder | Purpose |
| --- | --- |
| `simulation/` | Drake scene construction, cameras, contact configuration, and episode resets |
| `assets/` | Robot/connector models, meshes, and scene configuration files |
| `controllers/` | Classical alignment, insertion, recovery, and shared robot command interface |
| `learning/` | Demonstration collection, dataset loading, behavior cloning, and policy execution |
| `evaluation/` | Shared trial runner, metrics, experiment configurations, and analysis |
| `notebooks/` | Exploratory work and visual inspection; reusable code belongs in the folders above |
| `docs/` | Project scope, setup instructions, design decisions, and report/video planning |
| `data/` | Local demonstration datasets; generated contents are ignored by Git |
| `results/` | Local experiment outputs and checkpoints; generated contents are ignored by Git |
| `utils/` | Existing course utilities; retain their current import paths |

## Initial implementation milestones

1. Reproducible Drake setup and a visible IIWA/connector/socket scene.
2. Ideal insertion and a blocked insertion with force/depth logging.
3. Classical force-feedback control under randomized misalignment.
4. Demonstration collection and a state-based behavior-cloning baseline.
5. Matched evaluation, then vision integration.

Runtime dependencies and executable entry points will be documented as they
are implemented. No runnable insertion simulation is included yet.

The starter peg can be previewed in an environment with Drake installed:

```bash
python simulation/preview_peg.py
```

See `simulation/README.md` for robot attachment and `assets/README.md` for
dimensions and coordinate frames.

Keep source code and small configuration files in Git. Keep generated datasets,
checkpoints, and videos outside commits; place selected report figures in `docs/`.
