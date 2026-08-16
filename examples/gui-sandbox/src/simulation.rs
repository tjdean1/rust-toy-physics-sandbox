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
    pub side_walls_enabled: bool,
    pub wall_distance: f32,
    pub ceiling_enabled: bool,
    pub ceiling_distance: f32,
    pub boundary_restitution: f32,
    pub gravity: [f32; 2],
    pub fixed_dt: f32,
}

impl Default for SceneConfig {
    fn default() -> Self {
        let sphere_a = SphereConfig {
            position: [-3.0, 2.0],
            velocity: [2.0, 0.0],
            mass: 1.0,
            radius: 0.5,
            restitution: 0.9,
        };
        let sphere_b = SphereConfig {
            position: [3.0, 2.0],
            velocity: [-1.0, 0.0],
            mass: 2.0,
            radius: 0.5,
            restitution: 0.9,
        };

        Self {
            spheres: vec![sphere_a, sphere_b],
            ground_enabled: false,
            ground_height: 0.0,
            side_walls_enabled: false,
            wall_distance: 6.0,
            ceiling_enabled: false,
            ceiling_distance: 6.0,
            boundary_restitution: 0.75,
            gravity: [0.0, 0.0],
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

        if (config.ground_enabled || config.ceiling_enabled) && !config.ground_height.is_finite() {
            return Err("ground height must be finite".into());
        }

        if config.ground_enabled {
            add_boundary_plane(
                &mut world,
                Vec3::new(0.0, config.ground_height, 0.0),
                Vec3::Y,
                config.boundary_restitution,
                "ground",
            )?;
        }

        if config.side_walls_enabled {
            if !config.wall_distance.is_finite() || config.wall_distance <= 0.0 {
                return Err("side-wall distance must be finite and greater than zero".into());
            }

            add_boundary_plane(
                &mut world,
                Vec3::new(-config.wall_distance, 0.0, 0.0),
                Vec3::X,
                config.boundary_restitution,
                "left wall",
            )?;
            add_boundary_plane(
                &mut world,
                Vec3::new(config.wall_distance, 0.0, 0.0),
                Vec3::NEG_X,
                config.boundary_restitution,
                "right wall",
            )?;
        }

        if config.ceiling_enabled {
            if !config.ceiling_distance.is_finite() || config.ceiling_distance <= 0.0 {
                return Err("ceiling distance must be finite and greater than zero".into());
            }

            add_boundary_plane(
                &mut world,
                Vec3::new(0.0, config.ground_height + config.ceiling_distance, 0.0),
                Vec3::NEG_Y,
                config.boundary_restitution,
                "ceiling",
            )?;
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

fn add_boundary_plane(
    world: &mut PhysicsWorld,
    position: Vec3,
    normal: Vec3,
    restitution: f32,
    name: &str,
) -> Result<(), String> {
    let mut boundary = RigidBody::static_body(position).with_collision_shape(
        Plane::new(normal).expect("sandbox boundary normals are constant unit vectors"),
    );
    boundary
        .set_restitution(restitution)
        .map_err(|error| format!("{name}: {error}"))?;
    world.add_body(boundary);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f32 = 1.0e-5;

    #[test]
    fn configured_simulation_matches_direct_headless_world() {
        let config = SceneConfig::default();
        let mut gui_simulation = Simulation::from_config(&config).unwrap();

        let mut headless_world = PhysicsWorld::with_gravity(Vec3::ZERO);
        let mut sphere_a = RigidBody::new(Vec3::new(-3.0, 2.0, 0.0), Vec3::new(2.0, 0.0, 0.0), 1.0)
            .unwrap()
            .with_collision_shape(Sphere::new(0.5).unwrap());
        sphere_a.set_restitution(0.9).unwrap();
        let sphere_a_handle = headless_world.add_body(sphere_a);
        let mut sphere_b = RigidBody::new(Vec3::new(3.0, 2.0, 0.0), Vec3::new(-1.0, 0.0, 0.0), 2.0)
            .unwrap()
            .with_collision_shape(Sphere::new(0.5).unwrap());
        sphere_b.set_restitution(0.9).unwrap();
        let sphere_b_handle = headless_world.add_body(sphere_b);

        for _ in 0..180 {
            gui_simulation.step(config.fixed_dt);
            headless_world.step(config.fixed_dt).unwrap();
        }

        for (index, headless_handle) in [sphere_a_handle, sphere_b_handle].into_iter().enumerate() {
            let gui_body = gui_simulation
                .world()
                .body(gui_simulation.sphere_handle(index).unwrap())
                .unwrap();
            let headless_body = headless_world.body(headless_handle).unwrap();
            assert!((gui_body.position() - headless_body.position()).length() <= TOLERANCE);
            assert!((gui_body.velocity() - headless_body.velocity()).length() <= TOLERANCE);
        }
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

    #[test]
    fn optional_side_walls_and_ceiling_confine_spheres() {
        let mut config = SceneConfig {
            spheres: vec![
                SphereConfig {
                    position: [3.75, 1.0],
                    velocity: [1.0, 0.0],
                    mass: 1.0,
                    radius: 0.5,
                    restitution: 1.0,
                },
                SphereConfig {
                    position: [0.0, 5.75],
                    velocity: [0.0, 1.0],
                    mass: 1.0,
                    radius: 0.5,
                    restitution: 1.0,
                },
            ],
            ground_enabled: true,
            ground_height: 0.0,
            side_walls_enabled: true,
            wall_distance: 4.0,
            ceiling_enabled: true,
            ceiling_distance: 6.0,
            boundary_restitution: 1.0,
            gravity: [0.0, 0.0],
            fixed_dt: 1.0 / 60.0,
        };
        let mut simulation = Simulation::from_config(&config).unwrap();

        simulation.step(0.0);

        let wall_sphere = simulation
            .world()
            .body(simulation.sphere_handle(0).unwrap())
            .unwrap();
        assert!((wall_sphere.position().x - 3.5).abs() <= TOLERANCE);
        assert!((wall_sphere.velocity().x + 1.0).abs() <= TOLERANCE);

        let ceiling_sphere = simulation
            .world()
            .body(simulation.sphere_handle(1).unwrap())
            .unwrap();
        assert!((ceiling_sphere.position().y - 5.5).abs() <= TOLERANCE);
        assert!((ceiling_sphere.velocity().y + 1.0).abs() <= TOLERANCE);

        config.wall_distance = 0.0;
        assert!(Simulation::from_config(&config).is_err());
    }
}
