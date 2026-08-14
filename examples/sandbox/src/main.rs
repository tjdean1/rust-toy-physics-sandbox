use physics_core::{PhysicsWorld, RigidBody, Vec3};

fn main() {
    let mut world = PhysicsWorld::new();
    let body = RigidBody::new(Vec3::new(0.0, 10.0, 0.0), Vec3::ZERO, 1.0)
        .expect("the sandbox body has a valid mass");
    let body_handle = world.add_body(body);

    let dt = 1.0 / 60.0;
    let step_count = 180;

    println!("Free fall from 10 m (no ground or collisions in Milestone 0.1)");
    println!("time (s) | position (m)            | velocity (m/s)");

    for step in 0..=step_count {
        if step % 30 == 0 {
            let body = world.body(body_handle).expect("body handle remains valid");
            println!(
                "{:>8.2} | ({:>7.3}, {:>7.3}, {:>7.3}) | ({:>7.3}, {:>7.3}, {:>7.3})",
                step as f32 * dt,
                body.position().x,
                body.position().y,
                body.position().z,
                body.velocity().x,
                body.velocity().y,
                body.velocity().z,
            );
        }

        if step < step_count {
            world.step(dt).expect("fixed timestep is valid");
        }
    }
}
