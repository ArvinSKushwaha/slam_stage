use std::sync::Arc;

use dashmap::DashMap;
use parking_lot::RwLock;

use crate::{
    Agent2D, Lidar2D,
    agent::{Agent2DConfig, Agent2DMeasurements, Agent2DState},
    scene::{AgentId, Scene2DState},
    sensors::{Sensor2D, TimeStamped},
};

#[derive(Default, Debug)]
pub struct Scene2DLoop {
    workers: DashMap<AgentId, AgentWorker>,
}

impl Scene2DLoop {
    pub fn contains_agent(&self, agent: AgentId) -> bool {
        self.workers.contains_key(&agent)
    }

    pub fn insert_agent(&self, agent_id: AgentId, agent: &Agent2D) {
        if !self.contains_agent(agent_id) {
            self.workers.insert(
                agent_id,
                AgentWorker {
                    lidar: SensorWorker {
                        lidar: Arc::clone(&agent.sensors.lidar),
                        worker: RwLock::new(None),
                        last_measurement: RwLock::new(None),
                    },
                },
            );
        }
    }

    pub fn update_state(
        &self,
        agent: AgentId,
        config: Agent2DConfig,
        state: Agent2DState,
        scene_state: Scene2DState,
    ) -> bool {
        if let Some(worker) = self.workers.get(&agent) {
            worker.update_state(config, state, scene_state);

            true
        } else {
            false
        }
    }

    pub fn query(&self, agent: AgentId) -> Option<Agent2DMeasurements> {
        Some(self.workers.get(&agent)?.query())
    }
}

#[derive(Debug)]
pub struct AgentWorker {
    lidar: SensorWorker<Lidar2D>,
}

impl AgentWorker {
    fn query(&self) -> Agent2DMeasurements {
        Agent2DMeasurements {
            lidar: self.lidar.last_measurement.read().clone(),
        }
    }

    fn update_state(&self, config: Agent2DConfig, state: Agent2DState, scene_state: Scene2DState) {
        self.lidar.update_state(config, state, scene_state);
    }
}

type Receiver<S> = flume::Receiver<TimeStamped<<S as Sensor2D>::SensorType>>;

#[derive(Debug)]
pub struct SensorWorker<S: Sensor2D> {
    lidar: Arc<RwLock<S>>,
    worker: RwLock<Option<Receiver<S>>>,
    last_measurement: RwLock<Option<TimeStamped<S::SensorType>>>,
}

impl<S: Sensor2D + Send + Sync + 'static> SensorWorker<S> {
    fn update_state(&self, config: Agent2DConfig, state: Agent2DState, scene_state: Scene2DState) where S::SensorType: Send + 'static {
        if let Some(rcv) = &*self.worker.read() {
            let rcvd = rcv.try_recv();

            if let Err(e) = rcvd
                && e == flume::TryRecvError::Empty
            {
                return;
            } else if let Ok(measurement) = rcvd {
                self.last_measurement.write().replace(measurement);
            }
        }

        let lidar = Arc::clone(&self.lidar);
        let (snd, rcv) = flume::bounded(1);
        rayon::spawn(move || {
            let measurement = lidar.read().sense(config, state, scene_state);
            if let Some(m) = measurement {
                let _ = snd.send(m);
            }
        });

        self.worker.write().replace(rcv);
    }
}
