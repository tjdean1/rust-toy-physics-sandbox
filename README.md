# rust-physics

`rust-physics` is an educational and experimental 3D physics engine written in
Rust. The engine is being built from scratch as a long-term project, with an
emphasis on clear, testable physics code that remains independent of rendering.

Milestone 0.1 currently provides basic rigid-body translation, accumulated
forces, configurable gravity, and fixed-timestep semi-implicit Euler
integration. It does not yet include collisions, rotation, constraints, or
rendering.

Run the command-line free-fall demonstration with:

```bash
cargo run -p sandbox
```

Run the project tests with:

```bash
cargo test
```
