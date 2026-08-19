use std::{error::Error, fmt};

use glam::Vec3;

use crate::{
    CollisionShape, Contact, RigidBody,
    collision::{resolve_contact, sphere_plane_contact, sphere_sphere_contact},
    integration::semi_implicit_euler,
};

/// Identifies a rigid body stored in a [`PhysicsWorld`].
///
/// Handles remain valid because the world currently only appends bodies. The
/// internal representation is private so it can evolve when removal is added.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct BodyHandle(usize);

/// A minimal collection of rigid bodies sharing one gravity field.
#[derive(Debug)]
pub struct PhysicsWorld {
    gravity: Vec3,
    bodies: Vec<RigidBody>,
    contacts: Vec<Contact>,
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
            contacts: Vec::new(),
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

    /// Returns contacts generated during the most recent successful step.
    ///
    /// Contact geometry describes the state before penetration correction.
    pub fn contacts(&self) -> &[Contact] {
        &self.contacts
    }

    /// Advances the simulation by `dt` seconds using semi-implicit Euler.
    ///
    /// Collision detection runs after integration. Dynamic spheres collide with
    /// static planes and other spheres using positional correction and a
    /// frictionless normal impulse.
    ///
    /// Zero is accepted as a no-time step and still clears accumulated forces
    /// and torques. Negative, infinite, and NaN timesteps are rejected without
    /// changing the world.
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

        self.contacts = self.generate_contacts();
        self.resolve_contacts();

        Ok(())
    }

    fn generate_contacts(&self) -> Vec<Contact> {
        let mut contacts = Vec::new();

        for (sphere_index, sphere_body) in self.bodies.iter().enumerate() {
            if sphere_body.is_static() {
                continue;
            }

            let Some(CollisionShape::Sphere(sphere)) = sphere_body.collision_shape() else {
                continue;
            };

            for (plane_index, plane_body) in self.bodies.iter().enumerate() {
                if !plane_body.is_static() {
                    continue;
                }

                let Some(CollisionShape::Plane(plane)) = plane_body.collision_shape() else {
                    continue;
                };

                if let Some(geometry) = sphere_plane_contact(sphere_body, sphere, plane_body, plane)
                {
                    contacts.push(Contact::new(
                        BodyHandle(sphere_index),
                        BodyHandle(plane_index),
                        geometry,
                    ));
                }
            }
        }

        for body_a_index in 0..self.bodies.len() {
            let body_a = &self.bodies[body_a_index];
            let Some(CollisionShape::Sphere(sphere_a)) = body_a.collision_shape() else {
                continue;
            };

            for body_b_index in (body_a_index + 1)..self.bodies.len() {
                let body_b = &self.bodies[body_b_index];
                if body_a.is_static() && body_b.is_static() {
                    continue;
                }

                let Some(CollisionShape::Sphere(sphere_b)) = body_b.collision_shape() else {
                    continue;
                };

                if let Some(geometry) = sphere_sphere_contact(body_a, sphere_a, body_b, sphere_b) {
                    contacts.push(Contact::new(
                        BodyHandle(body_a_index),
                        BodyHandle(body_b_index),
                        geometry,
                    ));
                }
            }
        }

        contacts
    }

    fn resolve_contacts(&mut self) {
        for contact in self.contacts.iter().copied() {
            let (body_a, body_b) =
                two_bodies_mut(&mut self.bodies, contact.body_a().0, contact.body_b().0);
            let restitution = body_a.restitution().min(body_b.restitution());
            resolve_contact(body_a, body_b, contact, restitution);
        }
    }
}

fn two_bodies_mut(
    bodies: &mut [RigidBody],
    body_a_index: usize,
    body_b_index: usize,
) -> (&mut RigidBody, &mut RigidBody) {
    debug_assert_ne!(body_a_index, body_b_index);

    if body_a_index < body_b_index {
        let (before_b, from_b) = bodies.split_at_mut(body_b_index);
        (&mut before_b[body_a_index], &mut from_b[0])
    } else {
        let (before_a, from_a) = bodies.split_at_mut(body_a_index);
        (&mut from_a[0], &mut before_a[body_b_index])
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
