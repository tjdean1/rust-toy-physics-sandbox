use physics_core::{PhysicsWorld, Quat, RigidBody, Vec3};

const TOLERANCE: f32 = 1.0e-5;

fn assert_vec3_close(actual: Vec3, expected: Vec3) {
    let difference = (actual - expected).length();
    assert!(
        difference <= TOLERANCE,
        "expected {expected:?}, got {actual:?} (difference {difference})"
    );
}

#[test]
fn torque_changes_angular_velocity_using_inertia() {
    let mut body = RigidBody::new(Vec3::ZERO, Vec3::ZERO, 2.0).unwrap();
    body.set_inertia(Vec3::new(2.0, 4.0, 8.0)).unwrap();
    body.apply_torque(Vec3::new(4.0, 0.0, 0.0));

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let handle = world.add_body(body);
    world.step(0.5).unwrap();

    assert_vec3_close(world.body(handle).unwrap().angular_velocity(), Vec3::X);
}

#[test]
fn inverse_inertia_rotates_with_the_body() {
    let mut body = RigidBody::new(Vec3::ZERO, Vec3::ZERO, 1.0).unwrap();
    body.set_inertia(Vec3::new(2.0, 4.0, 8.0)).unwrap();
    body.set_angular_velocity(Vec3::Z * std::f32::consts::FRAC_PI_2);

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let handle = world.add_body(body);
    world.step(1.0).unwrap();

    let body = world.body_mut(handle).unwrap();
    body.set_angular_velocity(Vec3::ZERO);
    body.apply_torque(Vec3::X);
    world.step(1.0).unwrap();

    assert_vec3_close(
        world.body(handle).unwrap().angular_velocity(),
        Vec3::X * 0.25,
    );
}

#[test]
fn orientation_uses_updated_angular_velocity() {
    let mut body = RigidBody::new(Vec3::ZERO, Vec3::ZERO, 1.0).unwrap();
    body.apply_torque(Vec3::Z);

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let handle = world.add_body(body);
    world.step(1.0).unwrap();

    let expected = Quat::from_rotation_z(1.0) * Vec3::X;
    let actual = world.body(handle).unwrap().orientation() * Vec3::X;
    assert_vec3_close(actual, expected);
}

#[test]
fn off_center_force_causes_translation_and_rotation() {
    let mut body = RigidBody::new(Vec3::ZERO, Vec3::ZERO, 2.0).unwrap();
    body.set_inertia(Vec3::splat(2.0)).unwrap();
    body.apply_force_at_point(Vec3::new(0.0, 10.0, 0.0), Vec3::X);

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let handle = world.add_body(body);
    world.step(0.5).unwrap();

    let body = world.body(handle).unwrap();
    assert_vec3_close(body.velocity(), Vec3::new(0.0, 2.5, 0.0));
    assert_vec3_close(body.angular_velocity(), Vec3::new(0.0, 0.0, 2.5));
}

#[test]
fn force_through_center_of_mass_produces_no_torque() {
    let position = Vec3::new(2.0, 3.0, 4.0);
    let mut body = RigidBody::new(position, Vec3::ZERO, 1.0).unwrap();
    body.apply_force_at_point(Vec3::Y, body.center_of_mass());

    assert_vec3_close(body.accumulated_force(), Vec3::Y);
    assert_vec3_close(body.accumulated_torque(), Vec3::ZERO);
}

#[test]
fn force_and_torque_accumulators_are_cleared() {
    let mut body = RigidBody::new(Vec3::ZERO, Vec3::ZERO, 1.0).unwrap();
    body.apply_force(Vec3::X);
    body.apply_torque(Vec3::Z);

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let handle = world.add_body(body);
    world.step(0.25).unwrap();

    let body = world.body(handle).unwrap();
    assert_vec3_close(body.accumulated_force(), Vec3::ZERO);
    assert_vec3_close(body.accumulated_torque(), Vec3::ZERO);
}

#[test]
fn static_body_ignores_force_and_torque() {
    let position = Vec3::new(1.0, 2.0, 3.0);
    let mut body = RigidBody::static_body(position);
    body.apply_force_at_point(Vec3::Y * 10.0, position + Vec3::X);

    let mut world = PhysicsWorld::new();
    let handle = world.add_body(body);
    world.step(1.0).unwrap();

    let body = world.body(handle).unwrap();
    assert_vec3_close(body.position(), position);
    assert_eq!(body.orientation(), Quat::IDENTITY);
    assert_vec3_close(body.angular_velocity(), Vec3::ZERO);
    assert_vec3_close(body.accumulated_torque(), Vec3::ZERO);
}

#[test]
fn invalid_inertia_is_rejected() {
    let invalid = [
        Vec3::ZERO,
        Vec3::new(-1.0, 1.0, 1.0),
        Vec3::splat(f32::INFINITY),
        Vec3::splat(f32::NAN),
    ];

    for inertia in invalid {
        let mut body = RigidBody::new(Vec3::ZERO, Vec3::ZERO, 1.0).unwrap();
        assert!(body.set_inertia(inertia).is_err());
    }

    assert!(
        RigidBody::static_body(Vec3::ZERO)
            .set_inertia(Vec3::ONE)
            .is_err()
    );
}

#[test]
fn repeated_rotation_keeps_orientation_normalized() {
    let mut body = RigidBody::new(Vec3::ZERO, Vec3::ZERO, 1.0).unwrap();
    body.set_angular_velocity(Vec3::new(1.0, 2.0, 3.0));

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let handle = world.add_body(body);
    for _ in 0..1_000 {
        world.step(1.0 / 120.0).unwrap();
    }

    let length = world.body(handle).unwrap().orientation().length();
    assert!((length - 1.0).abs() <= TOLERANCE, "length was {length}");
}
