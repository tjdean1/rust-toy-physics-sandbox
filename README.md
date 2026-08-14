# rust-physics

`rust-physics` is an educational and experimental 3D physics engine written in
Rust. The engine is being built from scratch as a long-term project, with an
emphasis on clear, testable physics code that remains independent of rendering.

Milestone 0.2 currently provides basic rigid-body translation, accumulated
forces, configurable gravity, fixed-timestep semi-implicit Euler integration,
sphere and plane collision shapes, and restitution-based sphere collisions with
static planes. It does not yet include sphere-to-sphere collisions, rotation,
friction, constraints, or rendering.

Run the command-line bouncing-sphere demonstration with:

```bash
cargo run -p sandbox
```

Run the project tests with:

```bash
cargo test
```
