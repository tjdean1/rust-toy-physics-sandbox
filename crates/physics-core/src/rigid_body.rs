use std::{error::Error, fmt};

use glam::Vec3;

/// A rigid body's translational state.
///
/// Position is measured in meters, velocity in meters per second, mass in
/// kilograms, and accumulated force in Newtons. Rotation is intentionally not
/// part of the Milestone 0.1 model.
#[derive(Clone, Debug)]
pub struct RigidBody {
    position: Vec3,
    velocity: Vec3,
    accumulated_force: Vec3,
    mass: f32,
    inverse_mass: f32,
}

impl RigidBody {
    /// Creates a dynamic body with a finite, strictly positive mass.
    pub fn new(position: Vec3, velocity: Vec3, mass: f32) -> Result<Self, InvalidMass> {
        if !mass.is_finite() || mass <= 0.0 {
            return Err(InvalidMass { mass });
        }

        Ok(Self {
            position,
            velocity,
            accumulated_force: Vec3::ZERO,
            mass,
            inverse_mass: mass.recip(),
        })
    }

    /// Creates an immovable body at `position`.
    ///
    /// Static bodies have infinite mass and zero inverse mass. Forces applied
    /// to them are discarded on the next world step without changing motion.
    pub fn static_body(position: Vec3) -> Self {
        Self {
            position,
            velocity: Vec3::ZERO,
            accumulated_force: Vec3::ZERO,
            mass: f32::INFINITY,
            inverse_mass: 0.0,
        }
    }

    /// Adds a force in Newtons to this step's force accumulator.
    pub fn apply_force(&mut self, force: Vec3) {
        self.accumulated_force += force;
    }

    /// Returns the position in meters.
    pub fn position(&self) -> Vec3 {
        self.position
    }

    /// Returns the velocity in meters per second.
    pub fn velocity(&self) -> Vec3 {
        self.velocity
    }

    /// Returns the accumulated force in Newtons.
    pub fn accumulated_force(&self) -> Vec3 {
        self.accumulated_force
    }

    /// Returns the mass in kilograms, or positive infinity for a static body.
    pub fn mass(&self) -> f32 {
        self.mass
    }

    /// Returns the inverse mass in inverse kilograms.
    pub fn inverse_mass(&self) -> f32 {
        self.inverse_mass
    }

    /// Returns whether this body is immovable.
    pub fn is_static(&self) -> bool {
        self.inverse_mass == 0.0
    }

    pub(crate) fn set_position(&mut self, position: Vec3) {
        self.position = position;
    }

    pub(crate) fn set_velocity(&mut self, velocity: Vec3) {
        self.velocity = velocity;
    }

    pub(crate) fn clear_forces(&mut self) {
        self.accumulated_force = Vec3::ZERO;
    }
}

/// Error returned when a dynamic body is given an invalid mass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InvalidMass {
    mass: f32,
}

impl InvalidMass {
    /// Returns the rejected mass value.
    pub fn value(self) -> f32 {
        self.mass
    }
}

impl fmt::Display for InvalidMass {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "mass must be finite and greater than zero, got {}",
            self.mass
        )
    }
}

impl Error for InvalidMass {}
