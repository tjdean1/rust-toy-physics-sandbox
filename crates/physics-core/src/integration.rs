use crate::RigidBody;

/// Advances one body using semi-implicit Euler integration.
pub(crate) fn semi_implicit_euler(body: &mut RigidBody, dt: f32) {
    if body.is_static() {
        body.clear_forces();
        body.clear_torques();
        return;
    }

    let acceleration = body.accumulated_force() * body.inverse_mass();
    let angular_acceleration = body.angular_acceleration();
    let velocity = body.velocity() + acceleration * dt;
    let angular_velocity = body.angular_velocity() + angular_acceleration * dt;
    let position = body.position() + velocity * dt;
    let orientation_delta = glam::Quat::from_scaled_axis(angular_velocity * dt);
    let orientation = orientation_delta * body.orientation();

    body.set_velocity(velocity);
    body.set_angular_velocity(angular_velocity);
    body.set_position(position);
    body.set_orientation(orientation);
    body.clear_forces();
    body.clear_torques();
}
