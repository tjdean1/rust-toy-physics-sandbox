use physics_core::{
    BoxCollider, CollisionShape, PhysicsWorld, Plane, Quat, RigidBody, Sphere, Vec3,
};

const TOLERANCE: f32 = 1.0e-4;

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

fn dynamic_box(position: Vec3, velocity: Vec3, half_extents: Vec3) -> RigidBody {
    let shape = BoxCollider::new(half_extents).unwrap();
    let mut body = RigidBody::new(position, velocity, 1.0)
        .unwrap()
        .with_collision_shape(shape);
    body.set_inertia(shape.solid_inertia(body.mass())).unwrap();
    body
}

fn ground() -> RigidBody {
    RigidBody::static_body(Vec3::ZERO).with_collision_shape(Plane::new(Vec3::Y).unwrap())
}

#[test]
fn box_validates_dimensions_and_computes_solid_inertia() {
    for half_extents in [
        Vec3::ZERO,
        Vec3::new(-1.0, 1.0, 1.0),
        Vec3::splat(f32::INFINITY),
        Vec3::splat(f32::NAN),
    ] {
        assert!(BoxCollider::new(half_extents).is_err());
    }

    let shape = BoxCollider::new(Vec3::new(1.0, 2.0, 3.0)).unwrap();
    assert_vec3_close(shape.half_extents(), Vec3::new(1.0, 2.0, 3.0));
    assert_vec3_close(shape.solid_inertia(6.0), Vec3::new(26.0, 20.0, 10.0));

    let body = dynamic_box(Vec3::ZERO, Vec3::ZERO, Vec3::ONE);
    assert_eq!(
        body.collision_shape(),
        Some(CollisionShape::Box(BoxCollider::new(Vec3::ONE).unwrap()))
    );
}

#[test]
fn invalid_orientation_is_rejected() {
    let mut body = dynamic_box(Vec3::ZERO, Vec3::ZERO, Vec3::ONE);
    assert!(
        body.set_orientation(Quat::from_xyzw(0.0, 0.0, 0.0, 0.0))
            .is_err()
    );
    assert!(
        body.set_orientation(Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0))
            .is_err()
    );

    body.set_orientation(Quat::from_rotation_z(0.5) * 2.0)
        .unwrap();
    assert_close(body.orientation().length(), 1.0);
}

#[test]
fn axis_aligned_box_contacts_plane() {
    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let box_handle = world.add_body(dynamic_box(
        Vec3::new(2.0, 0.75, 3.0),
        Vec3::ZERO,
        Vec3::new(1.0, 1.0, 0.5),
    ));
    let plane_handle = world.add_body(ground());

    world.step(0.0).unwrap();

    let contact = world.contacts()[0];
    assert_eq!(contact.body_a(), box_handle);
    assert_eq!(contact.body_b(), plane_handle);
    assert_vec3_close(contact.normal(), Vec3::Y);
    assert_vec3_close(contact.point(), Vec3::new(2.0, 0.0, 3.0));
    assert_close(contact.penetration(), 0.25);
    assert_close(world.body(box_handle).unwrap().position().y, 1.0);
}

#[test]
fn oriented_box_contacts_plane_at_its_low_corner() {
    let mut body = dynamic_box(Vec3::Y, Vec3::ZERO, Vec3::new(1.0, 0.5, 0.5));
    body.set_orientation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_4))
        .unwrap();

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    world.add_body(body);
    world.add_body(ground());
    world.step(0.0).unwrap();

    assert_eq!(world.contacts().len(), 1);
    assert_close(
        world.contacts()[0].penetration(),
        1.5 / 2.0_f32.sqrt() - 1.0,
    );
    assert_close(world.contacts()[0].point().y, 0.0);
}

#[test]
fn sphere_contacts_axis_aligned_and_oriented_boxes() {
    for orientation in [
        Quat::IDENTITY,
        Quat::from_rotation_z(0.5),
        Quat::from_rotation_y(0.7),
    ] {
        let mut box_body = RigidBody::static_body(Vec3::ZERO)
            .with_collision_shape(BoxCollider::new(Vec3::ONE).unwrap());
        box_body.set_orientation(orientation).unwrap();
        let direction = orientation * Vec3::X;
        let sphere = RigidBody::new(direction * 1.4, Vec3::ZERO, 1.0)
            .unwrap()
            .with_collision_shape(Sphere::new(0.5).unwrap());

        let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
        let sphere_handle = world.add_body(sphere);
        world.add_body(box_body);
        world.step(0.0).unwrap();

        assert_eq!(world.contacts().len(), 1);
        assert_vec3_close(world.contacts()[0].normal(), direction);
        assert_close(world.contacts()[0].penetration(), 0.1);
        assert_vec3_close(
            world.body(sphere_handle).unwrap().position(),
            direction * 1.5,
        );
    }
}

#[test]
fn sphere_inside_box_is_moved_to_the_nearest_face() {
    let sphere = RigidBody::new(Vec3::ZERO, Vec3::ZERO, 1.0)
        .unwrap()
        .with_collision_shape(Sphere::new(0.25).unwrap());
    let box_body = RigidBody::static_body(Vec3::ZERO)
        .with_collision_shape(BoxCollider::new(Vec3::ONE).unwrap());

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let sphere_handle = world.add_body(sphere);
    world.add_body(box_body);
    world.step(0.0).unwrap();

    assert_vec3_close(world.contacts()[0].normal(), Vec3::X);
    assert_close(world.contacts()[0].penetration(), 1.25);
    assert_vec3_close(
        world.body(sphere_handle).unwrap().position(),
        Vec3::X * 1.25,
    );
}

#[test]
fn axis_aligned_and_oriented_boxes_generate_contacts() {
    let cases = [
        (Quat::IDENTITY, Vec3::new(1.8, 0.0, 0.0)),
        (Quat::from_rotation_z(0.5), Vec3::new(1.6, 0.0, 0.0)),
        (
            Quat::from_rotation_y(0.4) * Quat::from_rotation_x(0.3),
            Vec3::new(1.6, 0.1, 0.1),
        ),
    ];

    for (orientation, position) in cases {
        let body_a = dynamic_box(Vec3::ZERO, Vec3::ZERO, Vec3::ONE);
        let mut body_b = dynamic_box(position, Vec3::ZERO, Vec3::ONE);
        body_b.set_orientation(orientation).unwrap();

        let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
        world.add_body(body_a);
        world.add_body(body_b);
        world.step(0.0).unwrap();

        assert_eq!(world.contacts().len(), 1);
        assert!(world.contacts()[0].penetration() >= 0.0);
    }
}

#[test]
fn separated_boxes_do_not_generate_contacts() {
    let mut body = dynamic_box(Vec3::new(3.0, 0.0, 0.0), Vec3::ZERO, Vec3::ONE);
    body.set_orientation(Quat::from_rotation_z(0.5)).unwrap();

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    world.add_body(dynamic_box(Vec3::ZERO, Vec3::ZERO, Vec3::ONE));
    world.add_body(body);
    world.step(0.0).unwrap();

    assert!(world.contacts().is_empty());
}

#[test]
fn centered_box_collision_does_not_create_rotation() {
    let mut moving = dynamic_box(Vec3::new(-1.0, 0.0, 0.0), Vec3::X, Vec3::ONE);
    moving.set_restitution(1.0).unwrap();
    let mut fixed =
        RigidBody::static_body(Vec3::X).with_collision_shape(BoxCollider::new(Vec3::ONE).unwrap());
    fixed.set_restitution(1.0).unwrap();

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let moving = world.add_body(moving);
    world.add_body(fixed);
    world.step(0.0).unwrap();

    assert_vec3_close(world.body(moving).unwrap().velocity(), Vec3::NEG_X);
    assert_vec3_close(world.body(moving).unwrap().angular_velocity(), Vec3::ZERO);
}

#[test]
fn off_center_collision_creates_angular_velocity() {
    let mut box_body = dynamic_box(Vec3::ZERO, Vec3::X, Vec3::ONE);
    box_body.set_restitution(1.0).unwrap();
    let mut sphere = RigidBody::static_body(Vec3::new(1.4, 0.5, 0.0))
        .with_collision_shape(Sphere::new(0.5).unwrap());
    sphere.set_restitution(1.0).unwrap();

    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let box_handle = world.add_body(box_body);
    world.add_body(sphere);
    world.step(0.0).unwrap();

    let body = world.body(box_handle).unwrap();
    assert!(body.velocity().x < 1.0);
    assert!(body.angular_velocity().z > 0.0);
}

#[test]
fn tilted_box_falls_and_rotates_after_ground_contact() {
    let mut body = dynamic_box(Vec3::new(0.0, 2.0, 0.0), Vec3::ZERO, Vec3::ONE * 0.5);
    body.set_orientation(Quat::from_rotation_z(0.3)).unwrap();
    body.set_restitution(0.25).unwrap();

    let mut world = PhysicsWorld::new();
    let handle = world.add_body(body);
    world.add_body(ground());
    let mut contacted = false;
    let mut rotated = false;

    for _ in 0..180 {
        world.step(1.0 / 60.0).unwrap();
        contacted |= !world.contacts().is_empty();
        rotated |= world.body(handle).unwrap().angular_velocity().length() > 0.1;
    }

    let body = world.body(handle).unwrap();
    assert!(contacted && rotated);
    assert!(body.position().is_finite());
    assert!(body.orientation().is_finite());
}
