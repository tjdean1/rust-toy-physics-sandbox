//! Core types for the `rust-physics` simulation engine.
//!
//! Milestone 1.0 models translational and rotational rigid-body motion, sphere
//! collisions, and collisions with static planes. Unless otherwise noted, the
//! crate uses SI units.

mod collision;
mod integration;
mod rigid_body;
mod world;

pub use collision::{
    CollisionShape, Contact, InvalidPlaneNormal, InvalidSphereRadius, Plane, Sphere,
};
pub use glam::{Mat3, Quat, Vec3};
pub use rigid_body::{InvalidInertia, InvalidMass, InvalidRestitution, RigidBody};
pub use world::{BodyHandle, InvalidTimestep, PhysicsWorld};
