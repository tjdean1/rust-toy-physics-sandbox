use crate::RigidBody;

/// Advances one body using semi-implicit Euler integration.
pub(crate) fn semi_implicit_euler(body: &mut RigidBody, dt: f32) {
    if body.is_static() {
        body.clear_forces();
        return;
    }

    let acceleration = body.accumulated_force() * body.inverse_mass();
    let velocity = body.velocity() + acceleration * dt;
    let position = body.position() + velocity * dt;

    body.set_velocity(velocity);
    body.set_position(position);
    body.clear_forces();
}
