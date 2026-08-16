# rust-physics

An educational and experimental 3D physics engine built from scratch in Rust.

The project is intentionally growing in small, testable milestones. The physics
library is independent of rendering and application frameworks, while separate
command-line and desktop sandboxes provide ways to observe and experiment with
the simulation.

## Current status

The project currently includes the work completed through Milestone 0.3:

| Milestone | Capability | Status |
| --- | --- | --- |
| 0.1 | Gravity, forces, and basic rigid-body translation | Complete |
| 0.2 | Sphere shapes, static planes, contacts, and bouncing | Complete |
| 0.2.1 | Interactive desktop sandbox with a 2D projection | Complete |
| 0.3 | Sphere-to-sphere collision and impulse response | Complete |
| 1.0 | Rigid-body rotation | Next |

### Physics engine

- Three-dimensional position and velocity using `glam::Vec3`
- Validated mass and inverse mass
- Static bodies represented by zero inverse mass
- Per-step force accumulation and clearing
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
- Edit position, velocity, mass, radius, restitution, gravity, and ground height
- Configure the fixed simulation timestep
- Play, pause, single-step, and reset the simulation
- Inspect simulation time and selected-body position and velocity
- View velocity arrows, contact points, and contact normals
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
stack. The default scene demonstrates two spheres of different masses colliding
head-on. Edit scene values in the left panel and press **Reset** to apply them,
then press **Play** or **Single Step**.

### Command-line sandbox

```bash
cargo run -p sandbox
```

This runs a fixed-timestep head-on collision between two spheres and prints
their positions, velocities, and contact event. It requires no desktop GUI.

### Tests

```bash
cargo test
```

The test suite covers fundamental motion, gravity, mass-independent
gravitational acceleration, forces, static bodies, invalid inputs, collision
shape validation, sphere-plane and sphere-sphere contact geometry, penetration
correction, restitution, momentum and energy conservation, coincident centers,
and agreement between GUI-configured and directly constructed headless
simulations.

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
| Acceleration | meters per second² |
| Force | Newtons |
| Mass | kilograms |
| Timestep | seconds |

Each successful world step currently performs these operations:

1. Apply gravity to dynamic bodies as a force proportional to mass.
2. Compute acceleration from accumulated force and inverse mass.
3. Update velocity, then position, using semi-implicit Euler integration.
4. Clear accumulated forces.
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

- Rotation, angular velocity, and torque
- Box collision shapes
- Friction and stable contact solving
- Constraints and joints
- Broad-phase collision acceleration
- Continuous collision detection
- Built-in 3D rendering
- ECS integration

The next planned milestone is **1.0: Rotation**, extending bodies with
orientation, angular velocity, torque, moment of inertia, and rotational
integration.
