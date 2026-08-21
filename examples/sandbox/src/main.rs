use physics_core::{BoxCollider, PhysicsWorld, Plane, Quat, RigidBody, Vec3};

fn main() {
    let mut world = PhysicsWorld::new();
    let shape = BoxCollider::new(Vec3::new(0.75, 0.5, 0.5)).expect("box dimensions are valid");
    let mut body = RigidBody::new(Vec3::new(0.0, 4.0, 0.0), Vec3::X, 2.0)
        .expect("mass is valid")
        .with_collision_shape(shape);
    body.set_orientation(Quat::from_rotation_z(0.35))
        .expect("orientation is valid");
    body.set_inertia(shape.solid_inertia(body.mass()))
        .expect("inertia is valid");
    body.set_restitution(0.4).expect("restitution is valid");
    let handle = world.add_body(body);

    let mut ground = RigidBody::static_body(Vec3::ZERO)
        .with_collision_shape(Plane::new(Vec3::Y).expect("plane normal is valid"));
    ground.set_restitution(0.4).expect("restitution is valid");
    world.add_body(ground);

    let dt = 1.0 / 60.0;
    let step_count = 180;

    println!("Tilted 2 kg box falling onto a static plane");
    println!("time (s) | x (m) | y (m) | angle (rad) | wz (rad/s) | contacts");

    for step in 0..=step_count {
        let body = world.body(handle).expect("body handle remains valid");
        if step % 15 == 0 {
            let local_x = body.orientation() * Vec3::X;
            println!(
                "{:>8.2} | {:>5.2} | {:>5.2} | {:>11.3} | {:>10.3} | {}",
                step as f32 * dt,
                body.position().x,
                body.position().y,
                local_x.y.atan2(local_x.x),
                body.angular_velocity().z,
                world.contacts().len(),
            );
        }

        if step < step_count {
            world.step(dt).expect("fixed timestep is valid");
        }
    }
}
