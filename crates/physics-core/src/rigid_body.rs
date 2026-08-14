use std::{error::Error, fmt};

use glam::Vec3;

use crate::CollisionShape;

/// A rigid body's translational state.
///
/// Position is measured in meters, velocity in meters per second, mass in
/// kilograms, and accumulated force in Newtons. Rotation is intentionally not
/// part of the current model.
#[derive(Clone, Debug)]
pub struct RigidBody {
    position: Vec3,
    velocity: Vec3,
    accumulated_force: Vec3,
    mass: f32,
    inverse_mass: f32,
    collision_shape: Option<CollisionShape>,
    restitution: f32,
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
            collision_shape: None,
            restitution: 0.0,
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
            collision_shape: None,
            restitution: 0.0,
        }
    }

    /// Adds a force in Newtons to this step's force accumulator.
    pub fn apply_force(&mut self, force: Vec3) {
        self.accumulated_force += force;
    }

    /// Attaches a collision shape centered or anchored at the body's position.
    pub fn set_collision_shape(&mut self, shape: impl Into<CollisionShape>) {
        self.collision_shape = Some(shape.into());
    }

    /// Attaches a collision shape and returns the body for construction chains.
    pub fn with_collision_shape(mut self, shape: impl Into<CollisionShape>) -> Self {
        self.set_collision_shape(shape);
        self
    }

    /// Returns the attached collision shape, if any.
    pub fn collision_shape(&self) -> Option<CollisionShape> {
        self.collision_shape
    }

    /// Sets the coefficient of restitution used by collision response.
    ///
    /// Restitution must be finite and between zero (inelastic) and one
    /// (perfectly elastic), inclusive. A collision pair uses the lower
    /// restitution of its two bodies.
    pub fn set_restitution(&mut self, restitution: f32) -> Result<(), InvalidRestitution> {
        if !restitution.is_finite() || !(0.0..=1.0).contains(&restitution) {
            return Err(InvalidRestitution { restitution });
        }

        self.restitution = restitution;
        Ok(())
    }

    /// Returns the body's coefficient of restitution.
    pub fn restitution(&self) -> f32 {
        self.restitution
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

/// Error returned when a body is given an invalid coefficient of restitution.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InvalidRestitution {
    restitution: f32,
}

impl InvalidRestitution {
    /// Returns the rejected coefficient.
    pub fn value(self) -> f32 {
        self.restitution
    }
}

impl fmt::Display for InvalidRestitution {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "restitution must be finite and between zero and one, got {}",
            self.restitution
        )
    }
}

impl Error for InvalidRestitution {}
