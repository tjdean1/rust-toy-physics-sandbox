use physics_core::{PhysicsWorld, RigidBody, Vec3};

fn main() {
    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);
    let mut body = RigidBody::new(Vec3::ZERO, Vec3::ZERO, 2.0).expect("mass is valid");
    body.set_inertia(Vec3::splat(2.0))
        .expect("inertia is valid");
    let handle = world.add_body(body);

    let force = Vec3::new(0.0, 10.0, 0.0);
    let dt = 1.0 / 60.0;
    let step_count = 60;

    println!("10 N applied 1 m right of a 2 kg body's center for 1 second");
    println!("time (s) | y (m) | vy (m/s) | wz (rad/s) | local x axis");

    for step in 0..=step_count {
        let body = world.body(handle).expect("body handle remains valid");
        if step % 15 == 0 {
            let local_x = body.orientation() * Vec3::X;
            println!(
                "{:>8.2} | {:>5.3} | {:>8.3} | {:>10.3} | ({:>6.3}, {:>6.3})",
                step as f32 * dt,
                body.position().y,
                body.velocity().y,
                body.angular_velocity().z,
                local_x.x,
                local_x.y,
            );
        }

        if step < step_count {
            let body = world.body_mut(handle).expect("body handle remains valid");
            let application_point = body.center_of_mass() + Vec3::X;
            body.apply_force_at_point(force, application_point);
            world.step(dt).expect("fixed timestep is valid");
        }
    }
}
