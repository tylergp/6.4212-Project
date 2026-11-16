# Assets

Store connector/socket model descriptions, collision geometry, meshes, and scene
configuration here. Use relative paths and record the origin/license of reused
assets. Keep dimensions, mating clearance, and coordinate frames documented.

## Starter peg

`peg.sdf` defines a 10 mm diameter, 50 mm long, 0.1 kg solid cylinder with
matching visual and collision geometry. Its body origin is at the center;
`peg_base` is at Z = -25 mm and `peg_tip` is at Z = +25 mm. All frames share
the same orientation; +Z points from base to tip. If dimensions or mass change,
update the inertia as well: Ixx = Iyy = m(3r^2 + L^2)/12; Izz = mr^2/2.

Contact/friction tuning for insertion will be configured with the task scene.

`socket.sdf` has an 11 mm minimum bore diameter, 13 mm entrance, 1 mm × 45°
lead-in, and 20 mm through depth. The entrance is at the origin with +Z inward.
`simulation.socket_model.add_socket` loads and welds the model to a mounting frame.
