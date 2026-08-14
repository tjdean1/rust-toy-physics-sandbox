//! Core types for the `rust-physics` simulation engine.
//!
//! Milestone 0.1 models translational rigid-body motion in three dimensions.
//! Unless otherwise noted, the crate uses SI units.

mod integration;
mod rigid_body;
mod world;

pub use glam::Vec3;
pub use rigid_body::{InvalidMass, RigidBody};
pub use world::{BodyHandle, InvalidTimestep, PhysicsWorld};
