mod simulation;

use std::time::Instant;

use eframe::egui::{self, Color32, Pos2, Rect, Sense, Stroke, Vec2};
use physics_core::CollisionShape;
use simulation::{SceneConfig, Simulation, SphereConfig};

const MAX_STEPS_PER_FRAME: usize = 16;

#[derive(Default)]
struct AxisBoundaries {
    left: Option<f32>,
    right: Option<f32>,
    ground: Option<f32>,
    ceiling: Option<f32>,
}

impl AxisBoundaries {
    fn record(&mut self, position: Vec2, normal: Vec2) {
        if normal.x.abs() > normal.y.abs() {
            if normal.x > 0.0 {
                self.left = Some(position.x);
            } else {
                self.right = Some(position.x);
            }
        } else if normal.y > 0.0 {
            self.ground = Some(position.y);
        } else {
            self.ceiling = Some(position.y);
        }
    }

    fn segments(&self, visible_min: Vec2, visible_max: Vec2) -> Vec<(Vec2, Vec2)> {
        let min_x = self.left.unwrap_or(visible_min.x);
        let max_x = self.right.unwrap_or(visible_max.x);
        let min_y = self.ground.unwrap_or(visible_min.y);
        let max_y = self.ceiling.unwrap_or(visible_max.y);
        let mut segments = Vec::with_capacity(4);

        if let Some(ground) = self.ground {
            segments.push((Vec2::new(min_x, ground), Vec2::new(max_x, ground)));
        }
        if let Some(ceiling) = self.ceiling {
            segments.push((Vec2::new(min_x, ceiling), Vec2::new(max_x, ceiling)));
        }
        if let Some(left) = self.left {
            segments.push((Vec2::new(left, min_y), Vec2::new(left, max_y)));
        }
        if let Some(right) = self.right {
            segments.push((Vec2::new(right, min_y), Vec2::new(right, max_y)));
        }

        segments
    }
}

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 760.0])
            .with_min_inner_size([850.0, 560.0]),
        ..Default::default()
    };

    eframe::run_native(
        "rust-physics interactive sandbox",
        options,
        Box::new(|_creation_context| Ok(Box::new(SandboxApp::new()))),
    )
}

struct SandboxApp {
    config: SceneConfig,
    simulation: Simulation,
    selected_sphere: usize,
    running: bool,
    accumulator: f32,
    last_frame: Instant,
    camera_center: Vec2,
    pixels_per_meter: f32,
    status: Option<String>,
}

impl SandboxApp {
    fn new() -> Self {
        let config = SceneConfig::default();
        let simulation = Simulation::from_config(&config).expect("the default scene is valid");
        Self {
            config,
            simulation,
            selected_sphere: 0,
            running: false,
            accumulator: 0.0,
            last_frame: Instant::now(),
            camera_center: Vec2::new(0.0, 3.0),
            pixels_per_meter: 70.0,
            status: None,
        }
    }

    fn reset(&mut self) {
        match Simulation::from_config(&self.config) {
            Ok(simulation) => {
                self.simulation = simulation;
                self.running = false;
                self.accumulator = 0.0;
                self.last_frame = Instant::now();
                self.status = None;
            }
            Err(error) => {
                self.running = false;
                self.status = Some(error);
            }
        }
    }

    fn advance_real_time(&mut self, elapsed: f32) {
        self.accumulator += elapsed.min(0.25);
        let mut steps = 0;
        while self.accumulator >= self.config.fixed_dt && steps < MAX_STEPS_PER_FRAME {
            self.simulation.step(self.config.fixed_dt);
            self.accumulator -= self.config.fixed_dt;
            steps += 1;
        }

        if steps == MAX_STEPS_PER_FRAME && self.accumulator >= self.config.fixed_dt {
            self.accumulator = 0.0;
            self.status = Some("simulation fell behind; excess frame time was discarded".into());
        }
    }

    fn controls(&mut self, root_ui: &mut egui::Ui) {
        let mut reset_requested = false;
        let mut step_requested = false;

        egui::Panel::left("controls")
            .resizable(true)
            .default_size(330.0)
            .show(root_ui, |ui| {
                ui.heading("rust-physics");
                ui.label("Milestone 0.3 · Sphere-to-Sphere Collisions");
                ui.add_space(8.0);

                ui.horizontal(|ui| {
                    if ui
                        .button(if self.running { "Pause" } else { "Play" })
                        .clicked()
                    {
                        self.running = !self.running;
                        self.last_frame = Instant::now();
                        ui.ctx().request_repaint();
                    }
                    if ui
                        .add_enabled(!self.running, egui::Button::new("Single Step"))
                        .clicked()
                    {
                        step_requested = true;
                    }
                    if ui.button("Reset").clicked() {
                        reset_requested = true;
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("Time");
                    ui.monospace(format!("{:.3} s", self.simulation.elapsed()));
                    ui.separator();
                    ui.label("Contacts");
                    ui.monospace(self.simulation.world().contacts().len().to_string());
                });

                if let Some(status) = &self.status {
                    ui.colored_label(Color32::LIGHT_RED, status);
                }

                ui.separator();
                ui.strong("Simulation");
                egui::Grid::new("simulation_settings")
                    .num_columns(2)
                    .show(ui, |ui| {
                        ui.label("Gravity X");
                        ui.add(egui::DragValue::new(&mut self.config.gravity[0]).speed(0.1));
                        ui.end_row();
                        ui.label("Gravity Y");
                        ui.add(egui::DragValue::new(&mut self.config.gravity[1]).speed(0.1));
                        ui.end_row();
                        ui.label("Fixed dt");
                        ui.add(
                            egui::DragValue::new(&mut self.config.fixed_dt)
                                .range((1.0 / 240.0)..=(1.0 / 15.0))
                                .speed(0.001)
                                .suffix(" s"),
                        );
                        ui.end_row();
                    });
                ui.small("Edited values take effect when Reset is pressed.");

                ui.separator();
                ui.horizontal(|ui| {
                    ui.strong("Shapes");
                    if ui.button("+ Sphere").clicked() {
                        let mut sphere = SphereConfig::default();
                        sphere.position[0] = self.config.spheres.len() as f32 * 1.25;
                        self.config.spheres.push(sphere);
                        self.selected_sphere = self.config.spheres.len() - 1;
                        reset_requested = true;
                    }
                    if ui
                        .add_enabled(!self.config.spheres.is_empty(), egui::Button::new("Remove"))
                        .clicked()
                    {
                        self.config.spheres.remove(self.selected_sphere);
                        self.selected_sphere = self
                            .selected_sphere
                            .min(self.config.spheres.len().saturating_sub(1));
                        reset_requested = true;
                    }
                });

                ui.horizontal_wrapped(|ui| {
                    for index in 0..self.config.spheres.len() {
                        ui.selectable_value(
                            &mut self.selected_sphere,
                            index,
                            format!("Sphere {}", index + 1),
                        );
                    }
                });
                ui.small("Spheres collide using frictionless, mass-aware impulses.");

                egui::ScrollArea::vertical().show(ui, |ui| {
                    if let Some(sphere) = self.config.spheres.get_mut(self.selected_sphere) {
                        ui.add_space(4.0);
                        egui::Grid::new("sphere_settings")
                            .num_columns(2)
                            .show(ui, |ui| {
                                drag_row(ui, "Position X", &mut sphere.position[0], 0.1, " m");
                                drag_row(ui, "Position Y", &mut sphere.position[1], 0.1, " m");
                                drag_row(ui, "Velocity X", &mut sphere.velocity[0], 0.1, " m/s");
                                drag_row(ui, "Velocity Y", &mut sphere.velocity[1], 0.1, " m/s");
                                positive_row(ui, "Mass", &mut sphere.mass, 0.1, " kg");
                                positive_row(ui, "Radius", &mut sphere.radius, 0.05, " m");
                                ui.label("Restitution");
                                ui.add(
                                    egui::DragValue::new(&mut sphere.restitution)
                                        .range(0.0..=1.0)
                                        .speed(0.01),
                                );
                                ui.end_row();
                            });
                    } else {
                        ui.weak("Add a sphere to configure it.");
                    }

                    ui.separator();
                    ui.strong("Boundary planes");
                    ui.checkbox(&mut self.config.ground_enabled, "Ground");
                    ui.checkbox(&mut self.config.side_walls_enabled, "Left and right walls");
                    ui.checkbox(&mut self.config.ceiling_enabled, "Ceiling");
                    egui::Grid::new("boundary_settings")
                        .num_columns(2)
                        .show(ui, |ui| {
                            drag_row(
                                ui,
                                "Ground height",
                                &mut self.config.ground_height,
                                0.1,
                                " m",
                            );
                            positive_row(
                                ui,
                                "Wall half-width",
                                &mut self.config.wall_distance,
                                0.1,
                                " m",
                            );
                            positive_row(
                                ui,
                                "Ceiling above ground",
                                &mut self.config.ceiling_distance,
                                0.1,
                                " m",
                            );
                            ui.label("Restitution");
                            ui.add(
                                egui::DragValue::new(&mut self.config.boundary_restitution)
                                    .range(0.0..=1.0)
                                    .speed(0.01),
                            );
                            ui.end_row();
                        });
                    ui.small("Walls are placed at ± half-width from x=0.");

                    ui.separator();
                    ui.strong("Selected runtime state");
                    if let Some(handle) = self.simulation.sphere_handle(self.selected_sphere)
                        && let Some(body) = self.simulation.world().body(handle)
                    {
                        ui.monospace(format!(
                            "position  ({:>8.3}, {:>8.3}) m\nvelocity  ({:>8.3}, {:>8.3}) m/s",
                            body.position().x,
                            body.position().y,
                            body.velocity().x,
                            body.velocity().y,
                        ));
                    } else {
                        ui.weak("No corresponding runtime sphere. Press Reset.");
                    }
                });
            });

        if reset_requested {
            self.reset();
        } else if step_requested {
            self.simulation.step(self.config.fixed_dt);
        }
    }

    fn viewport(&mut self, root_ui: &mut egui::Ui) {
        egui::CentralPanel::default().show(root_ui, |ui| {
            let (response, painter) = ui.allocate_painter(ui.available_size(), Sense::drag());
            let rect = response.rect;
            painter.rect_filled(rect, 0.0, Color32::from_rgb(18, 22, 29));

            if response.dragged() {
                let delta = ui.input(|input| input.pointer.delta());
                self.camera_center.x -= delta.x / self.pixels_per_meter;
                self.camera_center.y += delta.y / self.pixels_per_meter;
            }

            if response.hovered() {
                let scroll = ui.input(|input| input.smooth_scroll_delta.y);
                if scroll != 0.0 {
                    self.pixels_per_meter =
                        (self.pixels_per_meter * (scroll * 0.002).exp()).clamp(15.0, 240.0);
                }
            }

            self.draw_grid(&painter, rect);
            self.draw_scene(&painter, rect);

            painter.text(
                rect.left_top() + Vec2::splat(10.0),
                egui::Align2::LEFT_TOP,
                "Drag to pan · Scroll to zoom · X/Y projection",
                egui::FontId::monospace(12.0),
                Color32::GRAY,
            );
        });
    }

    fn draw_grid(&self, painter: &egui::Painter, rect: Rect) {
        let spacing = if self.pixels_per_meter < 30.0 {
            5.0
        } else if self.pixels_per_meter > 140.0 {
            0.5
        } else {
            1.0
        };
        let half_width = rect.width() * 0.5 / self.pixels_per_meter;
        let half_height = rect.height() * 0.5 / self.pixels_per_meter;
        let min_x = ((self.camera_center.x - half_width) / spacing).floor() as i32;
        let max_x = ((self.camera_center.x + half_width) / spacing).ceil() as i32;
        let min_y = ((self.camera_center.y - half_height) / spacing).floor() as i32;
        let max_y = ((self.camera_center.y + half_height) / spacing).ceil() as i32;

        for index in min_x..=max_x {
            let x = index as f32 * spacing;
            let color = if x.abs() < f32::EPSILON {
                Color32::from_gray(100)
            } else {
                Color32::from_gray(42)
            };
            painter.line_segment(
                [
                    self.world_to_screen(rect, [x, self.camera_center.y - half_height]),
                    self.world_to_screen(rect, [x, self.camera_center.y + half_height]),
                ],
                Stroke::new(1.0, color),
            );
        }

        for index in min_y..=max_y {
            let y = index as f32 * spacing;
            let color = if y.abs() < f32::EPSILON {
                Color32::from_gray(100)
            } else {
                Color32::from_gray(42)
            };
            painter.line_segment(
                [
                    self.world_to_screen(rect, [self.camera_center.x - half_width, y]),
                    self.world_to_screen(rect, [self.camera_center.x + half_width, y]),
                ],
                Stroke::new(1.0, color),
            );
        }
    }

    fn draw_scene(&self, painter: &egui::Painter, rect: Rect) {
        self.draw_boundaries(painter, rect);

        for (index, handle) in (0..self.config.spheres.len()).filter_map(|index| {
            self.simulation
                .sphere_handle(index)
                .map(|handle| (index, handle))
        }) {
            let Some(body) = self.simulation.world().body(handle) else {
                continue;
            };
            let Some(CollisionShape::Sphere(shape)) = body.collision_shape() else {
                continue;
            };

            let center = self.world_to_screen(rect, [body.position().x, body.position().y]);
            let radius = shape.radius() * self.pixels_per_meter;
            let fill = if index == self.selected_sphere {
                Color32::from_rgb(90, 170, 245)
            } else {
                Color32::from_rgb(100, 125, 170)
            };
            painter.circle(center, radius, fill, Stroke::new(2.0, Color32::WHITE));

            let velocity = body.velocity();
            let velocity_tip =
                center + Vec2::new(velocity.x, -velocity.y) * (self.pixels_per_meter * 0.12);
            painter.arrow(
                center,
                velocity_tip - center,
                Stroke::new(2.0, Color32::YELLOW),
            );
            painter.text(
                center,
                egui::Align2::CENTER_CENTER,
                (index + 1).to_string(),
                egui::FontId::proportional(14.0),
                Color32::BLACK,
            );
        }

        for contact in self.simulation.world().contacts() {
            let point = self.world_to_screen(rect, [contact.point().x, contact.point().y]);
            painter.circle_filled(point, 5.0, Color32::LIGHT_RED);
            let normal = contact.normal();
            painter.arrow(
                point,
                Vec2::new(normal.x, -normal.y) * 30.0,
                Stroke::new(2.0, Color32::LIGHT_RED),
            );
        }
    }

    fn draw_boundaries(&self, painter: &egui::Painter, rect: Rect) {
        let mut boundaries = AxisBoundaries::default();
        for body in self.simulation.world().bodies() {
            if let Some(CollisionShape::Plane(plane)) = body.collision_shape() {
                let normal = plane.normal();
                boundaries.record(
                    Vec2::new(body.position().x, body.position().y),
                    Vec2::new(normal.x, normal.y),
                );
            }
        }

        let half_width = rect.width() * 0.5 / self.pixels_per_meter;
        let half_height = rect.height() * 0.5 / self.pixels_per_meter;
        let visible_min = self.camera_center - Vec2::new(half_width, half_height);
        let visible_max = self.camera_center + Vec2::new(half_width, half_height);

        for (start, end) in boundaries.segments(visible_min, visible_max) {
            painter.line_segment(
                [
                    self.world_to_screen(rect, [start.x, start.y]),
                    self.world_to_screen(rect, [end.x, end.y]),
                ],
                Stroke::new(3.0, Color32::from_rgb(100, 190, 115)),
            );
        }
    }

    fn world_to_screen(&self, rect: Rect, point: [f32; 2]) -> Pos2 {
        Pos2::new(
            rect.center().x + (point[0] - self.camera_center.x) * self.pixels_per_meter,
            rect.center().y - (point[1] - self.camera_center.y) * self.pixels_per_meter,
        )
    }
}

impl eframe::App for SandboxApp {
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let now = Instant::now();
        let frame_time = now.duration_since(self.last_frame).as_secs_f32();
        self.last_frame = now;

        if self.running {
            self.advance_real_time(frame_time);
            ctx.request_repaint();
        }
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.controls(ui);
        self.viewport(ui);
    }
}

fn drag_row(ui: &mut egui::Ui, label: &str, value: &mut f32, speed: f64, suffix: &str) {
    ui.label(label);
    ui.add(egui::DragValue::new(value).speed(speed).suffix(suffix));
    ui.end_row();
}

fn positive_row(ui: &mut egui::Ui, label: &str, value: &mut f32, speed: f64, suffix: &str) {
    ui.label(label);
    ui.add(
        egui::DragValue::new(value)
            .range(0.001..=f32::MAX)
            .speed(speed)
            .suffix(suffix),
    );
    ui.end_row();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enclosure_lines_stop_at_boundary_intersections() {
        let boundaries = AxisBoundaries {
            left: Some(-4.0),
            right: Some(4.0),
            ground: Some(0.0),
            ceiling: Some(6.0),
        };

        let segments = boundaries.segments(Vec2::splat(-10.0), Vec2::splat(10.0));

        assert_eq!(
            segments,
            vec![
                (Vec2::new(-4.0, 0.0), Vec2::new(4.0, 0.0)),
                (Vec2::new(-4.0, 6.0), Vec2::new(4.0, 6.0)),
                (Vec2::new(-4.0, 0.0), Vec2::new(-4.0, 6.0)),
                (Vec2::new(4.0, 0.0), Vec2::new(4.0, 6.0)),
            ]
        );
    }
}
