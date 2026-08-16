use physics_core::{PhysicsWorld, RigidBody, Sphere, Vec3};

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

fn dynamic_sphere(position: Vec3, velocity: Vec3, mass: f32, radius: f32) -> RigidBody {
    RigidBody::new(position, velocity, mass)
        .unwrap()
        .with_collision_shape(Sphere::new(radius).unwrap())
}

#[test]
fn separated_spheres_do_not_generate_a_contact() {
    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    world.add_body(dynamic_sphere(Vec3::ZERO, Vec3::ZERO, 1.0, 0.5));
    world.add_body(dynamic_sphere(
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::ZERO,
        1.0,
        0.5,
    ));

    world.step(0.0).unwrap();

    assert!(world.contacts().is_empty());
}

#[test]
fn sphere_contact_reports_geometry_and_corrects_both_positions() {
    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let body_a = world.add_body(dynamic_sphere(
        Vec3::new(-0.75, 0.0, 0.0),
        Vec3::ZERO,
        1.0,
        1.0,
    ));
    let body_b = world.add_body(dynamic_sphere(
        Vec3::new(0.75, 0.0, 0.0),
        Vec3::ZERO,
        1.0,
        1.0,
    ));

    world.step(0.0).unwrap();

    assert_eq!(world.contacts().len(), 1);
    let contact = world.contacts()[0];
    assert_eq!(contact.body_a(), body_a);
    assert_eq!(contact.body_b(), body_b);
    assert_vec3_close(contact.normal(), Vec3::NEG_X);
    assert_vec3_close(contact.point(), Vec3::ZERO);
    assert_close(contact.penetration(), 0.5);
    assert_vec3_close(world.body(body_a).unwrap().position(), Vec3::NEG_X);
    assert_vec3_close(world.body(body_b).unwrap().position(), Vec3::X);
}

#[test]
fn equal_mass_elastic_collision_exchanges_velocities() {
    let mut body_a = dynamic_sphere(Vec3::NEG_X, Vec3::X, 1.0, 1.0);
    let mut body_b = dynamic_sphere(Vec3::X, Vec3::NEG_X, 1.0, 1.0);
    body_a.set_restitution(1.0).unwrap();
    body_b.set_restitution(1.0).unwrap();

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let body_a = world.add_body(body_a);
    let body_b = world.add_body(body_b);
    world.step(0.0).unwrap();

    assert_vec3_close(world.body(body_a).unwrap().velocity(), Vec3::NEG_X);
    assert_vec3_close(world.body(body_b).unwrap().velocity(), Vec3::X);
}

#[test]
fn unequal_mass_elastic_collision_conserves_momentum_and_energy() {
    let mut body_a = dynamic_sphere(Vec3::NEG_X, Vec3::new(2.0, 0.0, 0.0), 1.0, 1.0);
    let mut body_b = dynamic_sphere(Vec3::X, Vec3::ZERO, 3.0, 1.0);
    body_a.set_restitution(1.0).unwrap();
    body_b.set_restitution(1.0).unwrap();

    let initial_momentum =
        body_a.mass() * body_a.velocity().x + body_b.mass() * body_b.velocity().x;
    let initial_energy = 0.5 * body_a.mass() * body_a.velocity().length_squared()
        + 0.5 * body_b.mass() * body_b.velocity().length_squared();

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let body_a = world.add_body(body_a);
    let body_b = world.add_body(body_b);
    world.step(0.0).unwrap();

    let body_a = world.body(body_a).unwrap();
    let body_b = world.body(body_b).unwrap();
    assert_vec3_close(body_a.velocity(), Vec3::NEG_X);
    assert_vec3_close(body_b.velocity(), Vec3::X);

    let final_momentum = body_a.mass() * body_a.velocity().x + body_b.mass() * body_b.velocity().x;
    let final_energy = 0.5 * body_a.mass() * body_a.velocity().length_squared()
        + 0.5 * body_b.mass() * body_b.velocity().length_squared();
    assert_close(final_momentum, initial_momentum);
    assert_close(final_energy, initial_energy);
}

#[test]
fn collision_uses_the_lower_restitution() {
    let mut body_a = dynamic_sphere(Vec3::NEG_X, Vec3::X, 1.0, 1.0);
    let mut body_b = dynamic_sphere(Vec3::X, Vec3::NEG_X, 1.0, 1.0);
    body_a.set_restitution(0.25).unwrap();
    body_b.set_restitution(0.75).unwrap();

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let body_a = world.add_body(body_a);
    let body_b = world.add_body(body_b);
    world.step(0.0).unwrap();

    assert_vec3_close(
        world.body(body_a).unwrap().velocity(),
        Vec3::new(-0.25, 0.0, 0.0),
    );
    assert_vec3_close(
        world.body(body_b).unwrap().velocity(),
        Vec3::new(0.25, 0.0, 0.0),
    );
}

#[test]
fn sphere_collision_preserves_tangential_velocity() {
    let mut body_a = dynamic_sphere(Vec3::NEG_X, Vec3::new(1.0, 2.0, 0.0), 1.0, 1.0);
    let mut body_b = dynamic_sphere(Vec3::X, Vec3::new(-1.0, -3.0, 0.0), 1.0, 1.0);
    body_a.set_restitution(1.0).unwrap();
    body_b.set_restitution(1.0).unwrap();

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let body_a = world.add_body(body_a);
    let body_b = world.add_body(body_b);
    world.step(0.0).unwrap();

    assert_vec3_close(
        world.body(body_a).unwrap().velocity(),
        Vec3::new(-1.0, 2.0, 0.0),
    );
    assert_vec3_close(
        world.body(body_b).unwrap().velocity(),
        Vec3::new(1.0, -3.0, 0.0),
    );
}

#[test]
fn separating_spheres_are_corrected_without_an_impulse() {
    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let body_a = world.add_body(dynamic_sphere(
        Vec3::new(-0.75, 0.0, 0.0),
        Vec3::NEG_X,
        1.0,
        1.0,
    ));
    let body_b = world.add_body(dynamic_sphere(Vec3::new(0.75, 0.0, 0.0), Vec3::X, 1.0, 1.0));

    world.step(0.0).unwrap();

    assert_vec3_close(world.body(body_a).unwrap().velocity(), Vec3::NEG_X);
    assert_vec3_close(world.body(body_b).unwrap().velocity(), Vec3::X);
    assert_vec3_close(world.body(body_a).unwrap().position(), Vec3::NEG_X);
    assert_vec3_close(world.body(body_b).unwrap().position(), Vec3::X);
}

#[test]
fn dynamic_sphere_bounces_off_a_static_sphere() {
    let mut dynamic = dynamic_sphere(
        Vec3::new(-0.75, 0.0, 0.0),
        Vec3::new(2.0, 0.0, 0.0),
        1.0,
        1.0,
    );
    dynamic.set_restitution(1.0).unwrap();
    let mut fixed = RigidBody::static_body(Vec3::new(0.75, 0.0, 0.0))
        .with_collision_shape(Sphere::new(1.0).unwrap());
    fixed.set_restitution(1.0).unwrap();

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let dynamic = world.add_body(dynamic);
    let fixed = world.add_body(fixed);
    world.step(0.0).unwrap();

    assert_vec3_close(
        world.body(dynamic).unwrap().velocity(),
        Vec3::new(-2.0, 0.0, 0.0),
    );
    assert_vec3_close(world.body(fixed).unwrap().velocity(), Vec3::ZERO);
    assert_vec3_close(
        world.body(fixed).unwrap().position(),
        Vec3::new(0.75, 0.0, 0.0),
    );
}

#[test]
fn coincident_sphere_centers_are_resolved_without_non_finite_values() {
    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let body_a = world.add_body(dynamic_sphere(Vec3::ZERO, Vec3::X, 1.0, 1.0));
    let body_b = world.add_body(dynamic_sphere(Vec3::ZERO, Vec3::NEG_X, 1.0, 1.0));

    world.step(0.0).unwrap();

    for handle in [body_a, body_b] {
        let body = world.body(handle).unwrap();
        assert!(body.position().is_finite());
        assert!(body.velocity().is_finite());
    }
    assert_close(
        (world.body(body_a).unwrap().position() - world.body(body_b).unwrap().position()).length(),
        2.0,
    );
}

#[test]
fn multiple_sphere_pairs_generate_contacts_once_per_pair() {
    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let left = world.add_body(dynamic_sphere(
        Vec3::new(-1.5, 0.0, 0.0),
        Vec3::ZERO,
        1.0,
        1.0,
    ));
    let middle = world.add_body(dynamic_sphere(Vec3::ZERO, Vec3::ZERO, 1.0, 1.0));
    let right = world.add_body(dynamic_sphere(
        Vec3::new(1.5, 0.0, 0.0),
        Vec3::ZERO,
        1.0,
        1.0,
    ));

    world.step(0.0).unwrap();

    assert_eq!(world.contacts().len(), 2);
    assert_eq!(world.contacts()[0].body_a(), left);
    assert_eq!(world.contacts()[0].body_b(), middle);
    assert_eq!(world.contacts()[1].body_a(), middle);
    assert_eq!(world.contacts()[1].body_b(), right);
}
