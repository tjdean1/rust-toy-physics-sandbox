use physics_core::{PhysicsWorld, RigidBody, Vec3};

const TOLERANCE: f32 = 1.0e-5;

fn assert_vec3_close(actual: Vec3, expected: Vec3) {
    let difference = (actual - expected).length();
    assert!(
        difference <= TOLERANCE,
        "expected {expected:?}, got {actual:?} (difference {difference})"
    );
}

#[test]
fn no_forces_and_no_gravity_maintains_constant_velocity() {
    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let handle = world.add_body(
        RigidBody::new(Vec3::new(1.0, 2.0, 3.0), Vec3::new(2.0, -1.0, 0.5), 3.0).unwrap(),
    );

    world.step(0.5).unwrap();

    let body = world.body(handle).unwrap();
    assert_vec3_close(body.velocity(), Vec3::new(2.0, -1.0, 0.5));
    assert_vec3_close(body.position(), Vec3::new(2.0, 1.5, 3.25));
}

#[test]
fn gravity_accelerates_a_dynamic_body_downward() {
    let mut world = PhysicsWorld::new();
    let handle = world.add_body(RigidBody::new(Vec3::ZERO, Vec3::ZERO, 1.0).unwrap());

    world.step(1.0).unwrap();

    let body = world.body(handle).unwrap();
    assert_vec3_close(body.velocity(), Vec3::new(0.0, -9.81, 0.0));
    assert_vec3_close(body.position(), Vec3::new(0.0, -9.81, 0.0));
}

#[test]
fn gravity_gives_different_masses_the_same_acceleration() {
    let mut world = PhysicsWorld::new();
    let light = world.add_body(RigidBody::new(Vec3::ZERO, Vec3::ZERO, 1.0).unwrap());
    let heavy = world.add_body(RigidBody::new(Vec3::ZERO, Vec3::ZERO, 100.0).unwrap());

    world.step(0.25).unwrap();

    assert_vec3_close(
        world.body(light).unwrap().velocity(),
        world.body(heavy).unwrap().velocity(),
    );
    assert_vec3_close(
        world.body(light).unwrap().velocity(),
        Vec3::new(0.0, -2.4525, 0.0),
    );
}

#[test]
fn static_bodies_do_not_move() {
    let initial_position = Vec3::new(4.0, 5.0, 6.0);
    let mut body = RigidBody::static_body(initial_position);
    body.apply_force(Vec3::new(1_000.0, 1_000.0, 1_000.0));

    let mut world = PhysicsWorld::new();
    let handle = world.add_body(body);
    world.step(2.0).unwrap();

    let body = world.body(handle).unwrap();
    assert!(body.is_static());
    assert_vec3_close(body.position(), initial_position);
    assert_vec3_close(body.velocity(), Vec3::ZERO);
}

#[test]
fn accumulated_forces_are_cleared_after_integration() {
    let mut body = RigidBody::new(Vec3::ZERO, Vec3::ZERO, 2.0).unwrap();
    body.apply_force(Vec3::new(8.0, 0.0, 0.0));

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let handle = world.add_body(body);
    world.step(0.5).unwrap();

    assert_vec3_close(world.body(handle).unwrap().accumulated_force(), Vec3::ZERO);
}

#[test]
fn constant_force_changes_velocity_as_expected() {
    let mut body = RigidBody::new(Vec3::ZERO, Vec3::ZERO, 2.0).unwrap();
    body.apply_force(Vec3::new(10.0, 0.0, 0.0));

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let handle = world.add_body(body);
    world.step(0.5).unwrap();

    assert_vec3_close(
        world.body(handle).unwrap().velocity(),
        Vec3::new(2.5, 0.0, 0.0),
    );
}

#[test]
fn invalid_mass_is_rejected() {
    for mass in [0.0, -1.0, f32::INFINITY, f32::NAN] {
        assert!(RigidBody::new(Vec3::ZERO, Vec3::ZERO, mass).is_err());
    }
}

#[test]
fn invalid_timestep_is_rejected_without_changing_the_world() {
    let mut body = RigidBody::new(Vec3::ZERO, Vec3::ZERO, 1.0).unwrap();
    body.apply_force(Vec3::X);
    let mut world = PhysicsWorld::new();
    let handle = world.add_body(body);

    assert!(world.step(-0.1).is_err());
    assert!(world.step(f32::NAN).is_err());

    let body = world.body(handle).unwrap();
    assert_vec3_close(body.position(), Vec3::ZERO);
    assert_vec3_close(body.velocity(), Vec3::ZERO);
    assert_vec3_close(body.accumulated_force(), Vec3::X);
}
