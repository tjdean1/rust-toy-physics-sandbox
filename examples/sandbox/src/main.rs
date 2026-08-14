use physics_core::{PhysicsWorld, Plane, RigidBody, Sphere, Vec3};

fn main() {
    let mut world = PhysicsWorld::new();
    let mut sphere = RigidBody::new(Vec3::new(0.0, 5.0, 0.0), Vec3::ZERO, 1.0)
        .expect("the sandbox sphere has a valid mass")
        .with_collision_shape(Sphere::new(0.5).expect("the sphere radius is valid"));
    sphere
        .set_restitution(0.75)
        .expect("the sphere restitution is valid");
    let sphere_handle = world.add_body(sphere);

    let mut ground = RigidBody::static_body(Vec3::ZERO)
        .with_collision_shape(Plane::new(Vec3::Y).expect("the ground normal is valid"));
    ground
        .set_restitution(0.75)
        .expect("the ground restitution is valid");
    world.add_body(ground);

    let dt = 1.0 / 60.0;
    let step_count = 240;

    println!("Bouncing 0.5 m sphere dropped from 5 m");
    println!("time (s) | center y (m) | vertical velocity (m/s) | contact");
    println!("{:>8.2} | {:>12.3} | {:>23.3} |", 0.0, 5.0, 0.0);

    for step in 1..=step_count {
        world.step(dt).expect("fixed timestep is valid");

        if step % 30 == 0 || !world.contacts().is_empty() {
            let sphere = world
                .body(sphere_handle)
                .expect("sphere handle remains valid");
            println!(
                "{:>8.2} | {:>12.3} | {:>23.3} | {}",
                step as f32 * dt,
                sphere.position().y,
                sphere.velocity().y,
                if world.contacts().is_empty() {
                    ""
                } else {
                    "sphere-plane"
                },
            );
        }
    }
}
