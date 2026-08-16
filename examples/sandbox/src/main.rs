use physics_core::{PhysicsWorld, RigidBody, Sphere, Vec3};

fn main() {
    let mut world = PhysicsWorld::with_gravity(Vec3::ZERO);

    let mut sphere_a = RigidBody::new(Vec3::new(-3.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0), 1.0)
        .expect("sphere A has a valid mass")
        .with_collision_shape(Sphere::new(0.5).expect("sphere A radius is valid"));
    sphere_a
        .set_restitution(0.9)
        .expect("sphere A restitution is valid");
    let sphere_a_handle = world.add_body(sphere_a);

    let mut sphere_b = RigidBody::new(Vec3::new(3.0, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0), 2.0)
        .expect("sphere B has a valid mass")
        .with_collision_shape(Sphere::new(0.5).expect("sphere B radius is valid"));
    sphere_b
        .set_restitution(0.9)
        .expect("sphere B restitution is valid");
    let sphere_b_handle = world.add_body(sphere_b);

    let dt = 1.0 / 60.0;
    let step_count = 180;

    println!("Head-on sphere collision with restitution 0.9");
    println!("sphere A: 1 kg at x=-3 m, vx=2 m/s");
    println!("sphere B: 2 kg at x= 3 m, vx=-1 m/s");
    println!("time (s) | A x (m) | A vx (m/s) | B x (m) | B vx (m/s) | contact");

    for step in 0..=step_count {
        if step % 15 == 0 || !world.contacts().is_empty() {
            let sphere_a = world
                .body(sphere_a_handle)
                .expect("sphere A handle remains valid");
            let sphere_b = world
                .body(sphere_b_handle)
                .expect("sphere B handle remains valid");
            println!(
                "{:>8.2} | {:>7.3} | {:>10.3} | {:>7.3} | {:>10.3} | {}",
                step as f32 * dt,
                sphere_a.position().x,
                sphere_a.velocity().x,
                sphere_b.position().x,
                sphere_b.velocity().x,
                if world.contacts().is_empty() {
                    ""
                } else {
                    "sphere-sphere"
                },
            );
        }

        if step < step_count {
            world.step(dt).expect("fixed timestep is valid");
        }
    }
}
