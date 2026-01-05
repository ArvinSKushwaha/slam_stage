use parking_lot::RwLock;
use std::{f32::consts::PI, sync::Arc};

use crate::{Lidar2D, sensors::{Sensor2D, TimeStamped}};

#[derive(Debug, Clone, Copy)]
pub struct Agent2DConfig {
    pub mass: f32,
    pub length: f32,
    pub width: f32,
    pub radius_tyre: f32,
    pub inertia_tyre: f32,
    pub torque_range: (f32, f32),
    pub beta_range: (f32, f32),
}

#[derive(Debug, Clone, Copy)]
pub struct Agent2DState {
    pub beta: f32,
    pub velocity: f32,
    pub torque: f32,
    pub position: glam::Vec2,
    pub heading: glam::Vec2,
}

#[derive(Debug, Clone)]
pub struct Agent2D {
    pub config: Agent2DConfig,
    pub state: Agent2DState,
    pub last_state: Option<Agent2DState>,
    pub sensors: Agent2DSensors,
}

#[derive(Debug)]
pub struct Agent2DSensors {
    pub lidar: Arc<RwLock<Lidar2D>>,
}

#[derive(Debug, Clone)]
pub struct Agent2DMeasurements {
    pub lidar: Option<TimeStamped<<Lidar2D as Sensor2D>::SensorType>>,
}

impl Clone for Agent2DSensors {
    fn clone(&self) -> Self {
        Self {
            lidar: Arc::clone(&self.lidar),
        }
    }
}

impl Default for Agent2DConfig {
    fn default() -> Self {
        Self {
            mass: 15.,
            length: 0.5,
            width: 0.25,
            radius_tyre: 0.33,
            inertia_tyre: 0.2,
            torque_range: (-100., 100.),
            beta_range: (-PI / 3., PI / 3.),
        }
    }
}

impl Agent2DConfig {
    fn with_scale(scale: f32) -> Self {
        let Self {
            mass,
            length,
            width,
            radius_tyre,
            inertia_tyre,
            torque_range,
            beta_range,
        } = Self::default();

        Self {
            mass: mass * scale.powi(2),
            length: length * scale,
            width: width * scale,
            radius_tyre: radius_tyre * scale,
            inertia_tyre: inertia_tyre * scale.powi(4),
            torque_range: (
                torque_range.0 * scale.powi(4),
                torque_range.1 * scale.powi(4),
            ),
            beta_range,
        }
    }
}

impl Default for Agent2DState {
    fn default() -> Self {
        Self {
            beta: 0.,
            velocity: 0.,
            torque: 0.,
            position: glam::Vec2::ZERO,
            heading: glam::Vec2::Y,
        }
    }
}

impl Default for Agent2D {
    fn default() -> Self {
        Agent2D {
            config: Default::default(),
            state: Agent2DState::default(),
            last_state: None,
            sensors: Agent2DSensors {
                lidar: Arc::new(RwLock::new(Lidar2D::default())),
            },
        }
    }
}

impl Agent2D {
    pub fn with_scale(scale: f32) -> Self {
        Self {
            config: Agent2DConfig::with_scale(scale),
            ..Default::default()
        }
    }

    pub fn update(&mut self, dt: f32) {
        let Agent2DConfig {
            mass,
            length,
            radius_tyre,
            inertia_tyre,
            ..
        } = self.config;
        let Agent2DState {
            beta,
            velocity,
            torque,
            heading,
            ..
        } = self.state;

        let (dbetadt, dvdt) = if let Some(last) = self.last_state {
            ((beta - last.beta) / dt, (velocity - last.velocity) / dt)
        } else {
            (0., 0.)
        };

        let tan_beta = beta.tan();
        let cos2_beta = 1. / (1. + tan_beta * tan_beta);

        let angular_velocity = (velocity) * tan_beta / length;
        let angular_acceleration =
            tan_beta / (length) * dvdt + (velocity) / (length * cos2_beta) * dbetadt;

        let acc = (radius_tyre) * (torque) / (2. * inertia_tyre + mass * radius_tyre * radius_tyre);

        let forward = heading;

        self.last_state = Some(self.state);

        self.state.position += forward * velocity * dt;
        self.state.velocity += acc * dt;
        self.state.heading =
            glam::Vec2::from_angle(angular_velocity * dt + angular_acceleration * dt * dt / 2.0)
                .rotate(heading)
                .normalize_or_zero();

        self.state.torque *= (0.01f32).powf(dt);
        self.state.beta *= (0.3f32).powf(dt);
    }
}
