//! Core types for the `rust-physics` simulation engine.
//!
//! Milestone 1.1 models translational and rotational rigid-body motion with
//! sphere, box, and static-plane collisions. Unless otherwise noted, the crate
//! uses SI units.

mod box_collision;
mod collision;
mod integration;
mod rigid_body;
mod world;

pub use box_collision::{BoxCollider, InvalidBoxHalfExtents};
pub use collision::{
    CollisionShape, Contact, InvalidPlaneNormal, InvalidSphereRadius, Plane, Sphere,
};
pub use glam::{Mat3, Quat, Vec3};
pub use rigid_body::{
    InvalidInertia, InvalidMass, InvalidOrientation, InvalidRestitution, RigidBody,
};
pub use world::{BodyHandle, InvalidTimestep, PhysicsWorld};
