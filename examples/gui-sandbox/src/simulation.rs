use physics_core::{BodyHandle, PhysicsWorld, Plane, RigidBody, Sphere, Vec3};

#[derive(Clone, Debug)]
pub struct SphereConfig {
    pub position: [f32; 2],
    pub velocity: [f32; 2],
    pub mass: f32,
    pub radius: f32,
    pub restitution: f32,
}

impl Default for SphereConfig {
    fn default() -> Self {
        Self {
            position: [0.0, 5.0],
            velocity: [0.0, 0.0],
            mass: 1.0,
            radius: 0.5,
            restitution: 0.75,
        }
    }
}

#[derive(Clone, Debug)]
pub struct SceneConfig {
    pub spheres: Vec<SphereConfig>,
    pub ground_enabled: bool,
    pub ground_height: f32,
    pub ground_restitution: f32,
    pub gravity: [f32; 2],
    pub fixed_dt: f32,
}

impl Default for SceneConfig {
    fn default() -> Self {
        Self {
            spheres: vec![SphereConfig::default()],
            ground_enabled: true,
            ground_height: 0.0,
            ground_restitution: 0.75,
            gravity: [0.0, -9.81],
            fixed_dt: 1.0 / 60.0,
        }
    }
}

pub struct Simulation {
    world: PhysicsWorld,
    sphere_handles: Vec<BodyHandle>,
    elapsed: f32,
}

impl Simulation {
    pub fn from_config(config: &SceneConfig) -> Result<Self, String> {
        if !config.fixed_dt.is_finite() || config.fixed_dt <= 0.0 {
            return Err("fixed timestep must be finite and greater than zero".into());
        }

        let gravity = Vec3::new(config.gravity[0], config.gravity[1], 0.0);
        if !gravity.is_finite() {
            return Err("gravity components must be finite".into());
        }

        let mut world = PhysicsWorld::with_gravity(gravity);
        let mut sphere_handles = Vec::with_capacity(config.spheres.len());

        for (index, sphere) in config.spheres.iter().enumerate() {
            let position = Vec3::new(sphere.position[0], sphere.position[1], 0.0);
            let velocity = Vec3::new(sphere.velocity[0], sphere.velocity[1], 0.0);
            if !position.is_finite() || !velocity.is_finite() {
                return Err(format!(
                    "sphere {} position and velocity must be finite",
                    index + 1
                ));
            }

            let shape = Sphere::new(sphere.radius)
                .map_err(|error| format!("sphere {}: {error}", index + 1))?;
            let mut body = RigidBody::new(position, velocity, sphere.mass)
                .map_err(|error| format!("sphere {}: {error}", index + 1))?
                .with_collision_shape(shape);
            body.set_restitution(sphere.restitution)
                .map_err(|error| format!("sphere {}: {error}", index + 1))?;
            sphere_handles.push(world.add_body(body));
        }

        if config.ground_enabled {
            if !config.ground_height.is_finite() {
                return Err("ground height must be finite".into());
            }

            let mut ground = RigidBody::static_body(Vec3::new(0.0, config.ground_height, 0.0))
                .with_collision_shape(
                    Plane::new(Vec3::Y).expect("the constant upward normal is valid"),
                );
            ground
                .set_restitution(config.ground_restitution)
                .map_err(|error| format!("ground: {error}"))?;
            world.add_body(ground);
        }

        Ok(Self {
            world,
            sphere_handles,
            elapsed: 0.0,
        })
    }

    pub fn step(&mut self, dt: f32) {
        self.world
            .step(dt)
            .expect("GUI simulation only supplies a validated fixed timestep");
        self.elapsed += dt;
    }

    pub fn world(&self) -> &PhysicsWorld {
        &self.world
    }

    pub fn sphere_handle(&self, index: usize) -> Option<BodyHandle> {
        self.sphere_handles.get(index).copied()
    }

    pub fn elapsed(&self) -> f32 {
        self.elapsed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f32 = 1.0e-5;

    #[test]
    fn configured_simulation_matches_direct_headless_world() {
        let config = SceneConfig::default();
        let mut gui_simulation = Simulation::from_config(&config).unwrap();

        let mut headless_world = PhysicsWorld::new();
        let mut sphere = RigidBody::new(Vec3::new(0.0, 5.0, 0.0), Vec3::ZERO, 1.0)
            .unwrap()
            .with_collision_shape(Sphere::new(0.5).unwrap());
        sphere.set_restitution(0.75).unwrap();
        let sphere_handle = headless_world.add_body(sphere);
        let mut ground =
            RigidBody::static_body(Vec3::ZERO).with_collision_shape(Plane::new(Vec3::Y).unwrap());
        ground.set_restitution(0.75).unwrap();
        headless_world.add_body(ground);

        for _ in 0..180 {
            gui_simulation.step(config.fixed_dt);
            headless_world.step(config.fixed_dt).unwrap();
        }

        let gui_body = gui_simulation
            .world()
            .body(gui_simulation.sphere_handle(0).unwrap())
            .unwrap();
        let headless_body = headless_world.body(sphere_handle).unwrap();
        assert!((gui_body.position() - headless_body.position()).length() <= TOLERANCE);
        assert!((gui_body.velocity() - headless_body.velocity()).length() <= TOLERANCE);
        assert!((gui_simulation.elapsed() - 3.0).abs() <= TOLERANCE);
    }

    #[test]
    fn invalid_scene_values_are_reported() {
        let mut config = SceneConfig::default();
        config.spheres[0].mass = 0.0;
        assert!(Simulation::from_config(&config).is_err());

        config.spheres[0].mass = 1.0;
        config.fixed_dt = -1.0;
        assert!(Simulation::from_config(&config).is_err());
    }
}
