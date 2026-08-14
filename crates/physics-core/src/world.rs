use std::{error::Error, fmt};

use glam::Vec3;

use crate::{RigidBody, integration::semi_implicit_euler};

/// Identifies a rigid body stored in a [`PhysicsWorld`].
///
/// Handles remain valid because Milestone 0.1 only appends bodies. The internal
/// representation is private so it can evolve when body removal is introduced.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct BodyHandle(usize);

/// A minimal collection of rigid bodies sharing one gravity field.
#[derive(Debug)]
pub struct PhysicsWorld {
    gravity: Vec3,
    bodies: Vec<RigidBody>,
}

impl PhysicsWorld {
    /// Earth-like gravity in meters per second squared.
    pub const EARTH_GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

    /// Creates an empty world with Earth-like gravity.
    pub fn new() -> Self {
        Self::with_gravity(Self::EARTH_GRAVITY)
    }

    /// Creates an empty world with the supplied acceleration due to gravity.
    pub fn with_gravity(gravity: Vec3) -> Self {
        Self {
            gravity,
            bodies: Vec::new(),
        }
    }

    /// Returns the gravity acceleration in meters per second squared.
    pub fn gravity(&self) -> Vec3 {
        self.gravity
    }

    /// Replaces the world's gravity acceleration.
    pub fn set_gravity(&mut self, gravity: Vec3) {
        self.gravity = gravity;
    }

    /// Adds a body and returns its handle.
    pub fn add_body(&mut self, body: RigidBody) -> BodyHandle {
        let handle = BodyHandle(self.bodies.len());
        self.bodies.push(body);
        handle
    }

    /// Returns a body for a valid handle.
    pub fn body(&self, handle: BodyHandle) -> Option<&RigidBody> {
        self.bodies.get(handle.0)
    }

    /// Returns a mutable body for a valid handle.
    pub fn body_mut(&mut self, handle: BodyHandle) -> Option<&mut RigidBody> {
        self.bodies.get_mut(handle.0)
    }

    /// Returns all bodies in insertion order.
    pub fn bodies(&self) -> &[RigidBody] {
        &self.bodies
    }

    /// Advances the simulation by `dt` seconds using semi-implicit Euler.
    ///
    /// Zero is accepted as a no-time step and still clears accumulated forces.
    /// Negative, infinite, and NaN timesteps are rejected without changing the
    /// world.
    pub fn step(&mut self, dt: f32) -> Result<(), InvalidTimestep> {
        if !dt.is_finite() || dt < 0.0 {
            return Err(InvalidTimestep { dt });
        }

        for body in &mut self.bodies {
            if !body.is_static() {
                let gravity_force = self.gravity * body.mass();
                body.apply_force(gravity_force);
            }
            semi_implicit_euler(body, dt);
        }

        Ok(())
    }
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

/// Error returned when a world step is given an invalid timestep.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InvalidTimestep {
    dt: f32,
}

impl InvalidTimestep {
    /// Returns the rejected timestep in seconds.
    pub fn value(self) -> f32 {
        self.dt
    }
}

impl fmt::Display for InvalidTimestep {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "timestep must be finite and non-negative, got {}",
            self.dt
        )
    }
}

impl Error for InvalidTimestep {}
