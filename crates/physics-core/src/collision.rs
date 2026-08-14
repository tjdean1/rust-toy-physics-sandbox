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
/// For Milestone 0.2, `body_a` is a dynamic sphere and `body_b` is a static
/// plane. The normal points from the plane toward the sphere, the point lies on
/// the plane, and penetration depth is measured in meters.
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

    /// Returns the dynamic sphere body handle.
    pub fn body_a(self) -> BodyHandle {
        self.body_a
    }

    /// Returns the static plane body handle.
    pub fn body_b(self) -> BodyHandle {
        self.body_b
    }

    /// Returns the contact point in world-space meters.
    pub fn point(self) -> Vec3 {
        self.point
    }

    /// Returns the normalized contact normal from the plane toward the sphere.
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

pub(crate) fn resolve_static_contact(body: &mut RigidBody, contact: Contact, restitution: f32) {
    body.set_position(body.position() + contact.normal() * contact.penetration());

    let normal_velocity = body.velocity().dot(contact.normal());
    if normal_velocity < 0.0 {
        body.set_velocity(
            body.velocity() - contact.normal() * ((1.0 + restitution) * normal_velocity),
        );
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
