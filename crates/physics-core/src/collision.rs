use std::{error::Error, fmt};

use glam::Vec3;

use crate::{RigidBody, world::BodyHandle};

/// A sphere collision shape with a radius measured in meters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sphere {
    radius: f32,
}

impl Sphere {
    /// Creates a sphere with a finite, strictly positive radius.
    pub fn new(radius: f32) -> Result<Self, InvalidSphereRadius> {
        if !radius.is_finite() || radius <= 0.0 {
            return Err(InvalidSphereRadius { radius });
        }

        Ok(Self { radius })
    }

    /// Returns the sphere radius in meters.
    pub fn radius(self) -> f32 {
        self.radius
    }
}

/// An infinite, one-sided plane collision shape.
///
/// The plane passes through its body's position, and its normal points toward
/// the non-penetrating side.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane {
    normal: Vec3,
}

impl Plane {
    /// Creates a plane and normalizes the supplied world-space normal.
    pub fn new(normal: Vec3) -> Result<Self, InvalidPlaneNormal> {
        if !normal.is_finite() {
            return Err(InvalidPlaneNormal { normal });
        }

        let Some(normal) = normal.try_normalize() else {
            return Err(InvalidPlaneNormal { normal });
        };

        Ok(Self { normal })
    }

    /// Returns the normalized world-space plane normal.
    pub fn normal(self) -> Vec3 {
        self.normal
    }
}

/// A collision shape attached to a rigid body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CollisionShape {
    /// A sphere centered on the body's position.
    Sphere(Sphere),
    /// An infinite plane passing through the body's position.
    Plane(Plane),
}

impl From<Sphere> for CollisionShape {
    fn from(sphere: Sphere) -> Self {
        Self::Sphere(sphere)
    }
}

impl From<Plane> for CollisionShape {
    fn from(plane: Plane) -> Self {
        Self::Plane(plane)
    }
}

/// Contact information generated before penetration correction.
///
/// The normal always points from `body_b` toward `body_a`. The contact point is
/// expressed in world-space meters, and penetration depth is measured in
/// meters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact {
    body_a: BodyHandle,
    body_b: BodyHandle,
    point: Vec3,
    normal: Vec3,
    penetration: f32,
}

impl Contact {
    pub(crate) fn new(body_a: BodyHandle, body_b: BodyHandle, geometry: ContactGeometry) -> Self {
        Self {
            body_a,
            body_b,
            point: geometry.point,
            normal: geometry.normal,
            penetration: geometry.penetration,
        }
    }

    /// Returns the first body handle.
    pub fn body_a(self) -> BodyHandle {
        self.body_a
    }

    /// Returns the second body handle.
    pub fn body_b(self) -> BodyHandle {
        self.body_b
    }

    /// Returns the contact point in world-space meters.
    pub fn point(self) -> Vec3 {
        self.point
    }

    /// Returns the normalized contact normal from `body_b` toward `body_a`.
    pub fn normal(self) -> Vec3 {
        self.normal
    }

    /// Returns the penetration depth in meters.
    pub fn penetration(self) -> f32 {
        self.penetration
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct ContactGeometry {
    point: Vec3,
    normal: Vec3,
    penetration: f32,
}

pub(crate) fn sphere_plane_contact(
    sphere_body: &RigidBody,
    sphere: Sphere,
    plane_body: &RigidBody,
    plane: Plane,
) -> Option<ContactGeometry> {
    let normal = plane.normal();
    let center_to_plane = sphere_body.position() - plane_body.position();
    let signed_distance = center_to_plane.dot(normal);
    let penetration = sphere.radius() - signed_distance;

    if penetration < 0.0 {
        return None;
    }

    Some(ContactGeometry {
        point: sphere_body.position() - normal * signed_distance,
        normal,
        penetration,
    })
}

pub(crate) fn sphere_sphere_contact(
    body_a: &RigidBody,
    sphere_a: Sphere,
    body_b: &RigidBody,
    sphere_b: Sphere,
) -> Option<ContactGeometry> {
    let center_delta = body_a.position() - body_b.position();
    let radius_sum = sphere_a.radius() + sphere_b.radius();
    let distance_squared = center_delta.length_squared();

    if distance_squared > radius_sum * radius_sum {
        return None;
    }

    let (normal, distance) = if distance_squared > f32::EPSILON {
        let distance = distance_squared.sqrt();
        (center_delta / distance, distance)
    } else {
        let fallback = body_b.velocity() - body_a.velocity();
        (fallback.try_normalize().unwrap_or(Vec3::X), 0.0)
    };

    let point_a = body_a.position() - normal * sphere_a.radius();
    let point_b = body_b.position() + normal * sphere_b.radius();

    Some(ContactGeometry {
        point: (point_a + point_b) * 0.5,
        normal,
        penetration: radius_sum - distance,
    })
}

pub(crate) fn resolve_contact(
    body_a: &mut RigidBody,
    body_b: &mut RigidBody,
    contact: Contact,
    restitution: f32,
) {
    let inverse_mass_sum = body_a.inverse_mass() + body_b.inverse_mass();
    if inverse_mass_sum == 0.0 {
        return;
    }

    let correction = contact.normal() * (contact.penetration() / inverse_mass_sum);
    body_a.set_position(body_a.position() + correction * body_a.inverse_mass());
    body_b.set_position(body_b.position() - correction * body_b.inverse_mass());

    let relative_velocity = body_a.velocity() - body_b.velocity();
    let normal_velocity = relative_velocity.dot(contact.normal());
    if normal_velocity < 0.0 {
        let impulse_magnitude = -(1.0 + restitution) * normal_velocity / inverse_mass_sum;
        let impulse = contact.normal() * impulse_magnitude;
        body_a.set_velocity(body_a.velocity() + impulse * body_a.inverse_mass());
        body_b.set_velocity(body_b.velocity() - impulse * body_b.inverse_mass());
    }
}

/// Error returned when a sphere is given an invalid radius.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InvalidSphereRadius {
    radius: f32,
}

impl InvalidSphereRadius {
    /// Returns the rejected radius in meters.
    pub fn value(self) -> f32 {
        self.radius
    }
}

impl fmt::Display for InvalidSphereRadius {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "sphere radius must be finite and greater than zero, got {}",
            self.radius
        )
    }
}

impl Error for InvalidSphereRadius {}

/// Error returned when a plane is given an invalid normal.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InvalidPlaneNormal {
    normal: Vec3,
}

impl InvalidPlaneNormal {
    /// Returns the rejected normal.
    pub fn value(self) -> Vec3 {
        self.normal
    }
}

impl fmt::Display for InvalidPlaneNormal {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "plane normal must be finite and non-zero, got {:?}",
            self.normal
        )
    }
}

impl Error for InvalidPlaneNormal {}
