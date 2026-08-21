use physics_core::{BodyHandle, BoxCollider, PhysicsWorld, Plane, Quat, RigidBody, Sphere, Vec3};

#[derive(Clone, Debug)]
pub struct SphereConfig {
    pub position: [f32; 2],
    pub velocity: [f32; 2],
    pub angular_velocity: f32,
    pub mass: f32,
    pub radius: f32,
    pub restitution: f32,
}

impl Default for SphereConfig {
    fn default() -> Self {
        Self {
            position: [0.0, 5.0],
            velocity: [0.0, 0.0],
            angular_velocity: 1.0,
            mass: 1.0,
            radius: 0.5,
            restitution: 0.75,
        }
    }
}

#[derive(Clone, Debug)]
pub struct BoxConfig {
    pub position: [f32; 2],
    pub velocity: [f32; 2],
    pub angle: f32,
    pub angular_velocity: f32,
    pub mass: f32,
    pub half_extents: [f32; 2],
    pub restitution: f32,
}

impl Default for BoxConfig {
    fn default() -> Self {
        Self {
            position: [0.0, 5.0],
            velocity: [0.0, 0.0],
            angle: 0.35,
            angular_velocity: 0.0,
            mass: 1.0,
            half_extents: [0.75, 0.5],
            restitution: 0.4,
        }
    }
}

#[derive(Clone, Debug)]
pub struct SceneConfig {
    pub spheres: Vec<SphereConfig>,
    pub boxes: Vec<BoxConfig>,
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
        let sphere = SphereConfig {
            position: [-2.0, 4.5],
            velocity: [1.0, 0.0],
            angular_velocity: 0.0,
            mass: 1.0,
            radius: 0.5,
            restitution: 0.5,
        };
        let box_a = BoxConfig::default();
        let box_b = BoxConfig {
            position: [1.4, 2.5],
            angle: -0.2,
            half_extents: [0.65, 0.65],
            ..BoxConfig::default()
        };

        Self {
            spheres: vec![sphere],
            boxes: vec![box_a, box_b],
            ground_enabled: true,
            ground_height: 0.0,
            side_walls_enabled: true,
            wall_distance: 6.0,
            ceiling_enabled: false,
            ceiling_distance: 6.0,
            boundary_restitution: 0.75,
            gravity: [0.0, -9.81],
            fixed_dt: 1.0 / 60.0,
        }
    }
}

pub struct Simulation {
    world: PhysicsWorld,
    sphere_handles: Vec<BodyHandle>,
    box_handles: Vec<BodyHandle>,
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
            if !position.is_finite()
                || !velocity.is_finite()
                || !sphere.angular_velocity.is_finite()
            {
                return Err(format!(
                    "sphere {} position and velocities must be finite",
                    index + 1
                ));
            }

            let shape = Sphere::new(sphere.radius)
                .map_err(|error| format!("sphere {}: {error}", index + 1))?;
            let mut body = RigidBody::new(position, velocity, sphere.mass)
                .map_err(|error| format!("sphere {}: {error}", index + 1))?
                .with_collision_shape(shape);
            body.set_inertia(shape.solid_inertia(sphere.mass))
                .map_err(|error| format!("sphere {}: {error}", index + 1))?;
            body.set_angular_velocity(Vec3::Z * sphere.angular_velocity);
            body.set_restitution(sphere.restitution)
                .map_err(|error| format!("sphere {}: {error}", index + 1))?;
            sphere_handles.push(world.add_body(body));
        }

        let mut box_handles = Vec::with_capacity(config.boxes.len());
        for (index, box_config) in config.boxes.iter().enumerate() {
            let position = Vec3::new(box_config.position[0], box_config.position[1], 0.0);
            let velocity = Vec3::new(box_config.velocity[0], box_config.velocity[1], 0.0);
            if !position.is_finite()
                || !velocity.is_finite()
                || !box_config.angle.is_finite()
                || !box_config.angular_velocity.is_finite()
            {
                return Err(format!(
                    "box {} transform and velocities must be finite",
                    index + 1
                ));
            }

            let shape = BoxCollider::new(Vec3::new(
                box_config.half_extents[0],
                box_config.half_extents[1],
                0.5,
            ))
            .map_err(|error| format!("box {}: {error}", index + 1))?;
            let mut body = RigidBody::new(position, velocity, box_config.mass)
                .map_err(|error| format!("box {}: {error}", index + 1))?
                .with_collision_shape(shape);
            body.set_orientation(Quat::from_rotation_z(box_config.angle))
                .map_err(|error| format!("box {}: {error}", index + 1))?;
            body.set_angular_velocity(Vec3::Z * box_config.angular_velocity);
            body.set_inertia(shape.solid_inertia(box_config.mass))
                .map_err(|error| format!("box {}: {error}", index + 1))?;
            body.set_restitution(box_config.restitution)
                .map_err(|error| format!("box {}: {error}", index + 1))?;
            box_handles.push(world.add_body(body));
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
            box_handles,
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

    pub fn box_handle(&self, index: usize) -> Option<BodyHandle> {
        self.box_handles.get(index).copied()
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
        let mut config = SceneConfig::default();
        config.spheres.clear();
        config.boxes = vec![BoxConfig {
            position: [1.0, 2.0],
            velocity: [0.5, -0.25],
            angle: 0.3,
            angular_velocity: 0.75,
            mass: 2.0,
            half_extents: [1.0, 0.5],
            restitution: 0.6,
        }];
        config.ground_enabled = false;
        config.side_walls_enabled = false;
        config.gravity = [0.0, 0.0];
        let mut gui_simulation = Simulation::from_config(&config).unwrap();

        let mut headless_world = PhysicsWorld::with_gravity(Vec3::ZERO);
        let shape = BoxCollider::new(Vec3::new(1.0, 0.5, 0.5)).unwrap();
        let mut body = RigidBody::new(Vec3::new(1.0, 2.0, 0.0), Vec3::new(0.5, -0.25, 0.0), 2.0)
            .unwrap()
            .with_collision_shape(shape);
        body.set_orientation(Quat::from_rotation_z(0.3)).unwrap();
        body.set_angular_velocity(Vec3::Z * 0.75);
        body.set_inertia(shape.solid_inertia(2.0)).unwrap();
        body.set_restitution(0.6).unwrap();
        let headless_handle = headless_world.add_body(body);

        for _ in 0..180 {
            gui_simulation.step(config.fixed_dt);
            headless_world.step(config.fixed_dt).unwrap();
        }

        let gui_body = gui_simulation
            .world()
            .body(gui_simulation.box_handle(0).unwrap())
            .unwrap();
        let headless_body = headless_world.body(headless_handle).unwrap();
        assert!((gui_body.position() - headless_body.position()).length() <= TOLERANCE);
        assert!((gui_body.velocity() - headless_body.velocity()).length() <= TOLERANCE);
        assert!(
            gui_body
                .orientation()
                .dot(headless_body.orientation())
                .abs()
                >= 1.0 - TOLERANCE
        );
        assert!((gui_simulation.elapsed() - 3.0).abs() <= TOLERANCE);
    }

    #[test]
    fn invalid_scene_values_are_reported() {
        let mut config = SceneConfig::default();
        config.spheres[0].mass = 0.0;
        assert!(Simulation::from_config(&config).is_err());

        config.spheres[0].mass = 1.0;
        config.boxes[0].half_extents[0] = 0.0;
        assert!(Simulation::from_config(&config).is_err());

        config.boxes[0].half_extents[0] = 0.5;
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
                    angular_velocity: 0.0,
                    mass: 1.0,
                    radius: 0.5,
                    restitution: 1.0,
                },
                SphereConfig {
                    position: [0.0, 5.75],
                    velocity: [0.0, 1.0],
                    angular_velocity: 0.0,
                    mass: 1.0,
                    radius: 0.5,
                    restitution: 1.0,
                },
            ],
            boxes: Vec::new(),
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
