use physics_core::{CollisionShape, PhysicsWorld, Plane, RigidBody, Sphere, Vec3};

const TOLERANCE: f32 = 1.0e-5;

fn assert_close(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() <= TOLERANCE,
        "expected {expected}, got {actual}"
    );
}

fn assert_vec3_close(actual: Vec3, expected: Vec3) {
    let difference = (actual - expected).length();
    assert!(
        difference <= TOLERANCE,
        "expected {expected:?}, got {actual:?} (difference {difference})"
    );
}

fn sphere_body(position: Vec3, velocity: Vec3, radius: f32) -> RigidBody {
    RigidBody::new(position, velocity, 1.0)
        .unwrap()
        .with_collision_shape(Sphere::new(radius).unwrap())
}

fn ground_plane() -> RigidBody {
    RigidBody::static_body(Vec3::ZERO).with_collision_shape(Plane::new(Vec3::Y).unwrap())
}

#[test]
fn shapes_validate_and_normalize_their_dimensions() {
    for radius in [0.0, -1.0, f32::INFINITY, f32::NAN] {
        assert!(Sphere::new(radius).is_err());
    }
    for normal in [
        Vec3::ZERO,
        Vec3::splat(f32::NAN),
        Vec3::splat(f32::INFINITY),
    ] {
        assert!(Plane::new(normal).is_err());
    }

    let sphere = Sphere::new(2.5).unwrap();
    let plane = Plane::new(Vec3::new(0.0, 4.0, 0.0)).unwrap();
    assert_close(sphere.radius(), 2.5);
    assert_vec3_close(plane.normal(), Vec3::Y);
}

#[test]
fn restitution_must_be_between_zero_and_one() {
    let mut body = RigidBody::new(Vec3::ZERO, Vec3::ZERO, 1.0).unwrap();

    for restitution in [-0.1, 1.1, f32::INFINITY, f32::NAN] {
        assert!(body.set_restitution(restitution).is_err());
    }

    body.set_restitution(0.75).unwrap();
    assert_close(body.restitution(), 0.75);
}

#[test]
fn separated_sphere_and_plane_do_not_generate_a_contact() {
    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    world.add_body(sphere_body(Vec3::new(0.0, 2.0, 0.0), Vec3::ZERO, 0.5));
    world.add_body(ground_plane());

    world.step(0.0).unwrap();

    assert!(world.contacts().is_empty());
}

#[test]
fn sphere_plane_contact_reports_geometry_and_corrects_penetration() {
    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let sphere = world.add_body(sphere_body(Vec3::new(2.0, 0.75, 3.0), Vec3::ZERO, 1.0));
    let plane = world.add_body(ground_plane());

    world.step(0.0).unwrap();

    assert_eq!(world.contacts().len(), 1);
    let contact = world.contacts()[0];
    assert_eq!(contact.body_a(), sphere);
    assert_eq!(contact.body_b(), plane);
    assert_vec3_close(contact.normal(), Vec3::Y);
    assert_vec3_close(contact.point(), Vec3::new(2.0, 0.0, 3.0));
    assert_close(contact.penetration(), 0.25);
    assert_vec3_close(
        world.body(sphere).unwrap().position(),
        Vec3::new(2.0, 1.0, 3.0),
    );
}

#[test]
fn collision_restitution_reverses_normal_velocity() {
    let mut sphere = sphere_body(Vec3::new(0.0, 0.5, 0.0), Vec3::new(0.0, -2.0, 0.0), 0.5);
    sphere.set_restitution(0.5).unwrap();
    let mut plane = ground_plane();
    plane.set_restitution(0.8).unwrap();

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let sphere = world.add_body(sphere);
    world.add_body(plane);
    world.step(0.01).unwrap();

    assert_vec3_close(
        world.body(sphere).unwrap().velocity(),
        Vec3::new(0.0, 1.0, 0.0),
    );
}

#[test]
fn collision_response_preserves_tangential_velocity() {
    let mut sphere = sphere_body(Vec3::new(0.0, 0.5, 0.0), Vec3::new(3.0, -2.0, -4.0), 0.5);
    sphere.set_restitution(1.0).unwrap();
    let mut plane = ground_plane();
    plane.set_restitution(1.0).unwrap();

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let sphere = world.add_body(sphere);
    world.add_body(plane);
    world.step(0.01).unwrap();

    assert_vec3_close(
        world.body(sphere).unwrap().velocity(),
        Vec3::new(3.0, 2.0, -4.0),
    );
}

#[test]
fn static_plane_does_not_move_during_collision_response() {
    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    world.add_body(sphere_body(Vec3::new(0.0, 0.25, 0.0), Vec3::Y, 0.5));
    let plane = world.add_body(ground_plane());

    world.step(0.0).unwrap();

    assert_vec3_close(world.body(plane).unwrap().position(), Vec3::ZERO);
    assert_vec3_close(world.body(plane).unwrap().velocity(), Vec3::ZERO);
}

#[test]
fn collision_shape_is_associated_with_its_body() {
    let sphere = Sphere::new(0.75).unwrap();
    let body = RigidBody::new(Vec3::ZERO, Vec3::ZERO, 1.0)
        .unwrap()
        .with_collision_shape(sphere);

    assert_eq!(body.collision_shape(), Some(CollisionShape::Sphere(sphere)));
}

#[test]
fn spheres_dropped_from_different_heights_settle_on_the_ground() {
    let mut world = PhysicsWorld::new();
    let low = world.add_body(sphere_body(Vec3::new(-2.0, 2.0, 0.0), Vec3::ZERO, 0.5));
    let high = world.add_body(sphere_body(Vec3::new(2.0, 6.0, 0.0), Vec3::ZERO, 0.5));
    world.add_body(ground_plane());

    for _ in 0..240 {
        world.step(1.0 / 60.0).unwrap();
    }

    for handle in [low, high] {
        let body = world.body(handle).unwrap();
        assert_close(body.position().y, 0.5);
        assert_close(body.velocity().y, 0.0);
    }
}
