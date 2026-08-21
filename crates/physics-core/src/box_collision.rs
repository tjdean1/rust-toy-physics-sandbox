use std::{error::Error, fmt};

use glam::Vec3;

use crate::{RigidBody, Sphere, collision::ContactGeometry};

const AXIS_EPSILON_SQUARED: f32 = 1.0e-8;

/// A box collider centered on a body's position.
///
/// It is axis-aligned when the body orientation is identity and oriented with
/// the body otherwise. Half-extents are measured in local-space meters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoxCollider {
    half_extents: Vec3,
}

impl BoxCollider {
    /// Creates a box with finite, strictly positive half-extents.
    pub fn new(half_extents: Vec3) -> Result<Self, InvalidBoxHalfExtents> {
        if !half_extents.is_finite() || half_extents.cmple(Vec3::ZERO).any() {
            return Err(InvalidBoxHalfExtents { half_extents });
        }

        Ok(Self { half_extents })
    }

    /// Returns local-space half-extents in meters.
    pub fn half_extents(self) -> Vec3 {
        self.half_extents
    }

    /// Returns the diagonal inertia of a solid box with `mass` kilograms.
    pub fn solid_inertia(self, mass: f32) -> Vec3 {
        let squared = self.half_extents * self.half_extents;
        Vec3::new(
            squared.y + squared.z,
            squared.x + squared.z,
            squared.x + squared.y,
        ) * (mass / 3.0)
    }
}

pub(crate) fn box_plane_contact(
    box_body: &RigidBody,
    box_shape: BoxCollider,
    plane_body: &RigidBody,
    plane_normal: Vec3,
) -> Option<ContactGeometry> {
    let axes = box_axes(box_body);
    let half = box_shape.half_extents();
    let projected_radius = half.x * plane_normal.dot(axes[0]).abs()
        + half.y * plane_normal.dot(axes[1]).abs()
        + half.z * plane_normal.dot(axes[2]).abs();
    let center_distance = (box_body.position() - plane_body.position()).dot(plane_normal);
    let penetration = projected_radius - center_distance;

    if penetration < 0.0 {
        return None;
    }

    let mut point_sum = Vec3::ZERO;
    let mut point_count = 0;
    for vertex in box_vertices(box_body, box_shape) {
        let distance = (vertex - plane_body.position()).dot(plane_normal);
        if distance <= 1.0e-5 {
            point_sum += vertex - plane_normal * distance;
            point_count += 1;
        }
    }

    let point = if point_count == 0 {
        box_body.position() - plane_normal * center_distance
    } else {
        point_sum / point_count as f32
    };

    Some(ContactGeometry::new(point, plane_normal, penetration))
}

pub(crate) fn sphere_box_contact(
    sphere_body: &RigidBody,
    sphere: Sphere,
    box_body: &RigidBody,
    box_shape: BoxCollider,
) -> Option<ContactGeometry> {
    let inverse_rotation = box_body.orientation().conjugate();
    let local_center = inverse_rotation * (sphere_body.position() - box_body.position());
    let half = box_shape.half_extents();
    let closest_local = local_center.clamp(-half, half);
    let offset = local_center - closest_local;
    let distance_squared = offset.length_squared();

    if distance_squared > sphere.radius() * sphere.radius() {
        return None;
    }

    let (normal_local, point_local, penetration) = if distance_squared > f32::EPSILON {
        let distance = distance_squared.sqrt();
        (offset / distance, closest_local, sphere.radius() - distance)
    } else {
        let face_distance = half - local_center.abs();
        let axis = smallest_axis(face_distance);
        let sign = if local_center[axis] < 0.0 { -1.0 } else { 1.0 };
        let mut normal = Vec3::ZERO;
        normal[axis] = sign;
        let mut point = local_center;
        point[axis] = half[axis] * sign;
        (normal, point, sphere.radius() + face_distance[axis])
    };

    let normal = box_body.orientation() * normal_local;
    let point = box_body.position() + box_body.orientation() * point_local;
    Some(ContactGeometry::new(point, normal, penetration))
}

pub(crate) fn box_box_contact(
    body_a: &RigidBody,
    box_a: BoxCollider,
    body_b: &RigidBody,
    box_b: BoxCollider,
) -> Option<ContactGeometry> {
    let axes_a = box_axes(body_a);
    let axes_b = box_axes(body_b);
    let center_delta = body_a.position() - body_b.position();
    let mut best_overlap = f32::INFINITY;
    let mut best_normal = Vec3::X;

    // OBB separation can occur on six face normals or nine edge cross-products.
    for axis in axes_a.into_iter().chain(axes_b).chain(
        axes_a
            .into_iter()
            .flat_map(|axis_a| axes_b.map(|axis_b| axis_a.cross(axis_b))),
    ) {
        if axis.length_squared() <= AXIS_EPSILON_SQUARED {
            continue;
        }

        let axis = axis.normalize();
        let radius_a = projection_radius(box_a, axes_a, axis);
        let radius_b = projection_radius(box_b, axes_b, axis);
        let overlap = radius_a + radius_b - center_delta.dot(axis).abs();
        if overlap < 0.0 {
            return None;
        }

        if overlap < best_overlap {
            best_overlap = overlap;
            best_normal = if center_delta.dot(axis) < 0.0 {
                -axis
            } else {
                axis
            };
        }
    }

    // A single representative point keeps response small until manifolds are added.
    let point_a = closest_point(body_a, box_a, body_b.position());
    let point_b = closest_point(body_b, box_b, body_a.position());
    Some(ContactGeometry::new(
        (point_a + point_b) * 0.5,
        best_normal,
        best_overlap,
    ))
}

fn box_axes(body: &RigidBody) -> [Vec3; 3] {
    let orientation = body.orientation();
    [
        orientation * Vec3::X,
        orientation * Vec3::Y,
        orientation * Vec3::Z,
    ]
}

fn projection_radius(shape: BoxCollider, axes: [Vec3; 3], axis: Vec3) -> f32 {
    let half = shape.half_extents();
    half.x * axis.dot(axes[0]).abs()
        + half.y * axis.dot(axes[1]).abs()
        + half.z * axis.dot(axes[2]).abs()
}

fn closest_point(body: &RigidBody, shape: BoxCollider, point: Vec3) -> Vec3 {
    let local = body.orientation().conjugate() * (point - body.position());
    body.position() + body.orientation() * local.clamp(-shape.half_extents(), shape.half_extents())
}

fn box_vertices(body: &RigidBody, shape: BoxCollider) -> [Vec3; 8] {
    let half = shape.half_extents();
    let orientation = body.orientation();
    let position = body.position();
    [
        Vec3::new(-half.x, -half.y, -half.z),
        Vec3::new(-half.x, -half.y, half.z),
        Vec3::new(-half.x, half.y, -half.z),
        Vec3::new(-half.x, half.y, half.z),
        Vec3::new(half.x, -half.y, -half.z),
        Vec3::new(half.x, -half.y, half.z),
        Vec3::new(half.x, half.y, -half.z),
        Vec3::new(half.x, half.y, half.z),
    ]
    .map(|vertex| position + orientation * vertex)
}

fn smallest_axis(vector: Vec3) -> usize {
    if vector.x <= vector.y && vector.x <= vector.z {
        0
    } else if vector.y <= vector.z {
        1
    } else {
        2
    }
}

/// Error returned when a box is given invalid half-extents.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InvalidBoxHalfExtents {
    half_extents: Vec3,
}

impl InvalidBoxHalfExtents {
    /// Returns the rejected half-extents in meters.
    pub fn value(self) -> Vec3 {
        self.half_extents
    }
}

impl fmt::Display for InvalidBoxHalfExtents {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "box half-extents must be finite and greater than zero, got {}",
            self.half_extents
        )
    }
}

impl Error for InvalidBoxHalfExtents {}
