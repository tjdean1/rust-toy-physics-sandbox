//! Core types for the `rust-physics` simulation engine.
//!
//! Milestone 0.2 models translational rigid-body motion and sphere collisions
//! with static planes. Unless otherwise noted, the crate uses SI units.

mod collision;
mod integration;
mod rigid_body;
mod world;

pub use collision::{
    CollisionShape, Contact, InvalidPlaneNormal, InvalidSphereRadius, Plane, Sphere,
};
pub use glam::Vec3;
pub use rigid_body::{InvalidMass, InvalidRestitution, RigidBody};
pub use world::{BodyHandle, InvalidTimestep, PhysicsWorld};
