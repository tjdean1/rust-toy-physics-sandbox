use std::{error::Error, fmt};

use glam::{Mat3, Quat, Vec3};

use crate::CollisionShape;

/// A rigid body's translational and rotational state.
///
/// `position` is the center of mass in meters. Linear velocity uses meters per
/// second, angular velocity uses radians per second, force uses Newtons, and
/// torque uses Newton-meters.
#[derive(Clone, Debug)]
pub struct RigidBody {
    position: Vec3,
    velocity: Vec3,
    accumulated_force: Vec3,
    orientation: Quat,
    angular_velocity: Vec3,
    accumulated_torque: Vec3,
    mass: f32,
    inverse_mass: f32,
    inertia: Vec3,
    inverse_inertia: Vec3,
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
            orientation: Quat::IDENTITY,
            angular_velocity: Vec3::ZERO,
            accumulated_torque: Vec3::ZERO,
            mass,
            inverse_mass: mass.recip(),
            inertia: Vec3::splat(mass),
            inverse_inertia: Vec3::splat(mass.recip()),
            collision_shape: None,
            restitution: 0.0,
        })
    }

    /// Creates an immovable body at `position`.
    ///
    /// Static bodies have infinite mass and inertia with zero inverses. Forces
    /// and torques are discarded on the next step without changing motion.
    pub fn static_body(position: Vec3) -> Self {
        Self {
            position,
            velocity: Vec3::ZERO,
            accumulated_force: Vec3::ZERO,
            orientation: Quat::IDENTITY,
            angular_velocity: Vec3::ZERO,
            accumulated_torque: Vec3::ZERO,
            mass: f32::INFINITY,
            inverse_mass: 0.0,
            inertia: Vec3::INFINITY,
            inverse_inertia: Vec3::ZERO,
            collision_shape: None,
            restitution: 0.0,
        }
    }

    /// Adds a force in Newtons to this step's force accumulator.
    pub fn apply_force(&mut self, force: Vec3) {
        self.accumulated_force += force;
    }

    /// Adds a torque in Newton-meters to this step's torque accumulator.
    pub fn apply_torque(&mut self, torque: Vec3) {
        self.accumulated_torque += torque;
    }

    /// Applies a force at a world-space point.
    ///
    /// The force contributes to linear motion and produces torque about the
    /// center of mass using `lever_arm x force`.
    pub fn apply_force_at_point(&mut self, force: Vec3, point: Vec3) {
        self.apply_force(force);
        self.apply_torque((point - self.center_of_mass()).cross(force));
    }

    /// Sets the diagonal moment of inertia in local body space, in kg*m^2.
    ///
    /// Dynamic bodies default to an isotropic inertia equal to their mass.
    /// Geometry-specific callers should replace that approximation.
    pub fn set_inertia(&mut self, inertia: Vec3) -> Result<(), InvalidInertia> {
        if self.is_static() || !inertia.is_finite() || inertia.cmple(Vec3::ZERO).any() {
            return Err(InvalidInertia {
                inertia,
                static_body: self.is_static(),
            });
        }

        self.inertia = inertia;
        self.inverse_inertia = inertia.recip();
        Ok(())
    }

    /// Sets angular velocity in radians per second around world-space axes.
    pub fn set_angular_velocity(&mut self, angular_velocity: Vec3) {
        self.angular_velocity = angular_velocity;
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

    /// Returns the center-of-mass position in meters.
    pub fn position(&self) -> Vec3 {
        self.position
    }

    /// Returns the center-of-mass position in meters.
    pub fn center_of_mass(&self) -> Vec3 {
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

    /// Returns the local-to-world orientation.
    pub fn orientation(&self) -> Quat {
        self.orientation
    }

    /// Returns angular velocity in radians per second around world-space axes.
    pub fn angular_velocity(&self) -> Vec3 {
        self.angular_velocity
    }

    /// Returns the accumulated torque in Newton-meters.
    pub fn accumulated_torque(&self) -> Vec3 {
        self.accumulated_torque
    }

    /// Returns the mass in kilograms, or positive infinity for a static body.
    pub fn mass(&self) -> f32 {
        self.mass
    }

    /// Returns the inverse mass in inverse kilograms.
    pub fn inverse_mass(&self) -> f32 {
        self.inverse_mass
    }

    /// Returns the diagonal moment of inertia in local body space, in kg*m^2.
    pub fn inertia(&self) -> Vec3 {
        self.inertia
    }

    /// Returns the diagonal inverse inertia in local body space.
    pub fn inverse_inertia(&self) -> Vec3 {
        self.inverse_inertia
    }

    /// Returns the inverse inertia tensor rotated into world space.
    pub fn world_inverse_inertia(&self) -> Mat3 {
        let rotation = Mat3::from_quat(self.orientation);
        rotation * Mat3::from_diagonal(self.inverse_inertia) * rotation.transpose()
    }

    /// Returns angular acceleration from the currently accumulated torque.
    pub fn angular_acceleration(&self) -> Vec3 {
        self.world_inverse_inertia() * self.accumulated_torque
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

    pub(crate) fn set_orientation(&mut self, orientation: Quat) {
        self.orientation = orientation.normalize();
    }

    pub(crate) fn clear_forces(&mut self) {
        self.accumulated_force = Vec3::ZERO;
    }

    pub(crate) fn clear_torques(&mut self) {
        self.accumulated_torque = Vec3::ZERO;
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

/// Error returned for non-positive inertia or when changing a static body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InvalidInertia {
    inertia: Vec3,
    static_body: bool,
}

impl InvalidInertia {
    /// Returns the rejected diagonal inertia.
    pub fn value(self) -> Vec3 {
        self.inertia
    }
}

impl fmt::Display for InvalidInertia {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.static_body {
            return write!(formatter, "cannot set inertia on a static body");
        }

        write!(
            formatter,
            "inertia must be finite and greater than zero on every axis, got {}",
            self.inertia
        )
    }
}

impl Error for InvalidInertia {}

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
