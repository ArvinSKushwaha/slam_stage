use crate::{
    agent::{Agent2DConfig, Agent2DState},
    scene::{Scene2DState, SceneTime},
};

pub mod lidar;

#[derive(Debug, Clone, Copy)]
pub struct TimeStamped<T> {
    pub time: SceneTime,
    pub state: T,
}

pub trait Sensor2D {
    type SensorType;

    fn sense(
        &self,
        agent_config: Agent2DConfig,
        agent_state: Agent2DState,
        scene: Scene2DState,
    ) -> Option<TimeStamped<Self::SensorType>>;
}
