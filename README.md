# rust-physics

An educational and experimental 3D physics engine built from scratch in Rust.

The project is intentionally growing in small, testable milestones. The physics
library is independent of rendering and application frameworks, while separate
command-line and desktop sandboxes provide ways to observe and experiment with
the simulation.

## Current status

The project currently includes the work completed through Milestone 1.0:

| Milestone | Capability | Status |
| --- | --- | --- |
| 0.1 | Gravity, forces, and basic rigid-body translation | Complete |
| 0.2 | Sphere shapes, static planes, contacts, and bouncing | Complete |
| 0.2.1 | Interactive desktop sandbox with a 2D projection | Complete |
| 0.3 | Sphere-to-sphere collision and impulse response | Complete |
| 1.0 | Quaternion orientation, torque, and rotational integration | Complete |
| 1.1 | Box colliders and rotational collision response | Next |

### Physics engine

- Three-dimensional linear and angular motion using `glam`
- Validated mass and inverse mass
- Quaternion orientation and angular velocity
- Per-step torque accumulation and force application at a point
- Local diagonal inertia and world-space inverse-inertia tensors
- Static bodies represented by zero inverse mass
- Per-step force and torque accumulation and clearing
- Configurable gravity with an Earth-like default of `(0, -9.81, 0)` m/s²
- Semi-implicit Euler integration using timesteps in seconds
- Validated sphere and infinite-plane collision shapes
- Dynamic-sphere versus static-plane contact detection
- Dynamic-sphere collisions with dynamic or static spheres
- Contact point, normal, penetration depth, and body handles
- Positional penetration correction
- Frictionless restitution-based collision response
- Inverse-mass-weighted impulses and penetration correction
- Rejection of invalid mass, restitution, shape, and timestep values

### Interactive sandbox

- X/Y projection of the 3D simulation with a world-space grid
- Add and remove spheres
- Independently enable a ground, side walls, and ceiling
- Configure wall half-width, ceiling distance, and boundary restitution
- Edit position, velocity, angular velocity, mass, radius, restitution, and gravity
- Configure the fixed simulation timestep
- Play, pause, single-step, and reset the simulation
- Inspect linear and angular runtime state
- View velocity arrows, orientation markers, contact points, and contact normals
- Pan by dragging the viewport and zoom with the scroll wheel

The GUI is a development tool layered on top of `physics-core`; none of its
windowing or rendering dependencies are included in the core engine or the
command-line sandbox.

## Requirements

- A current stable Rust toolchain with Cargo
- For the GUI sandbox, a desktop environment with OpenGL support and either
  X11 or Wayland on Linux

Install Rust through [rustup](https://rustup.rs/) if it is not already
available.

## Quick start

Run these commands from the repository root.

### Interactive GUI

```bash
cargo run -p gui-sandbox
```

The first GUI build takes longer because Cargo must compile the native windowing
stack. The default scene demonstrates two rotating spheres colliding head-on.
Edit scene values in the left panel and press **Reset** to apply them, then press
**Play** or **Single Step**.

### Command-line sandbox

```bash
cargo run -p sandbox
```

This applies a continuous off-center force for one second and prints the body's
linear and angular motion. It requires no desktop GUI.

### Tests

```bash
cargo test
```

The test suite covers fundamental motion, gravity, mass-independent
gravitational acceleration, forces, torque, inertia, static bodies, invalid
inputs, quaternion normalization, force-at-point behavior, collision geometry,
penetration correction, restitution, momentum and energy conservation, and
agreement between GUI-configured and directly constructed headless simulations.

## Using `physics-core`

Both sandboxes use the same public physics API. A minimal simulation looks like
this:

```rust
use physics_core::{PhysicsWorld, Plane, RigidBody, Sphere, Vec3};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();

    let mut ball = RigidBody::new(
        Vec3::new(0.0, 5.0, 0.0),
        Vec3::ZERO,
        1.0,
    )?
    .with_collision_shape(Sphere::new(0.5)?);
    ball.set_inertia(Vec3::splat(0.1))?; // Solid sphere: 2/5 mr².
    ball.apply_force_at_point(Vec3::X * 4.0, Vec3::new(0.0, 5.5, 0.0));
    ball.set_restitution(0.75)?;
    let ball_handle = world.add_body(ball);

    let mut ground = RigidBody::static_body(Vec3::ZERO)
        .with_collision_shape(Plane::new(Vec3::Y)?);
    ground.set_restitution(0.75)?;
    world.add_body(ground);

    let dt = 1.0 / 60.0;
    for _ in 0..180 {
        world.step(dt)?;
    }

    let ball = world.body(ball_handle).expect("body handle remains valid");
    println!("position: {:?}, velocity: {:?}", ball.position(), ball.velocity());

    Ok(())
}
```

## Physics conventions

The engine uses SI units wherever practical:

| Quantity | Unit |
| --- | --- |
| Position | meters |
| Velocity | meters per second |
| Angular velocity | radians per second |
| Acceleration | meters per second² |
| Force | Newtons |
| Torque | Newton-meters |
| Mass | kilograms |
| Moment of inertia | kilogram-meters² |
| Timestep | seconds |

Each successful world step currently performs these operations:

1. Apply gravity to dynamic bodies as a force proportional to mass.
2. Compute linear and angular acceleration from accumulated force, torque, and
   inverse mass/inertia.
3. Update linear and angular velocity, then position and quaternion orientation.
4. Clear accumulated forces and torques.
5. Detect sphere-plane and sphere-sphere contacts.
6. Correct penetration and resolve incoming relative normal velocity.

Collision response is discrete and frictionless. A collision pair uses the
lower restitution coefficient of its two bodies. Sphere-pair impulses and
position corrections are weighted by inverse mass, so linear momentum is
conserved by isolated collision response.

## Workspace layout

```text
rust-physics/
├── Cargo.toml
├── Cargo.lock
├── crates/
│   └── physics-core/       # Renderer-independent physics library
└── examples/
    ├── sandbox/            # Headless command-line demonstration
    └── gui-sandbox/        # Interactive eframe/egui desktop application
```

The packages are deliberately separated:

- `physics-core` depends only on `glam`.
- `sandbox` depends only on `physics-core`.
- `gui-sandbox` contains the optional `eframe`/`egui` dependency stack and
  consumes `physics-core` through its public API.

## Development

Before submitting changes, run:

```bash
cargo fmt --check
cargo check --workspace --all-targets
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
```

Generated compilation output is written to `target/` and is intentionally
ignored by Git. Cargo recreates it locally whenever the project is built.

## Current limitations

The following systems have not been implemented yet:

- Box collision shapes
- Rotational collision response
- Gyroscopic torque and non-diagonal local inertia
- Friction and stable contact solving
- Constraints and joints
- Broad-phase collision acceleration
- Continuous collision detection
- Built-in 3D rendering
- ECS integration

The next planned milestone is **1.1: Box Colliders**, adding box geometry,
box contacts, and rotational collision response.
