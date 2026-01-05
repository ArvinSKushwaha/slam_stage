use std::sync::Arc;

use rayon::prelude::*;
use rustc_hash::FxHashMap;

use crate::{
    Agent2D,
    math::Box2D,
    scene::{occupancy_map::OccupancyMap, scene_loop::Scene2DLoop},
};

lazy_static::lazy_static! {
    pub static ref FUTURES_THREAD_POOL: futures::executor::ThreadPool = futures::executor::ThreadPool::new().unwrap();
}

pub mod occupancy_map;
pub mod scene_loop;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SceneTime(f32);

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
pub struct AgentId(u64);

#[derive(Debug, Clone)]
pub struct Scene2D {
    pub agents: FxHashMap<AgentId, Agent2D>,
    pub time: SceneTime,
    pub occupancy_map: Arc<OccupancyMap>,
    pub scene_loop: Arc<Scene2DLoop>,
}

#[derive(Debug)]
pub struct Scene2DState {
    pub time: SceneTime,
    pub occupancy_map: Arc<OccupancyMap>,
}

impl Clone for Scene2DState {
    fn clone(&self) -> Self {
        Self {
            time: self.time,
            occupancy_map: Arc::clone(&self.occupancy_map),
        }
    }
}

impl Scene2D {
    pub fn from_pixels(size: [usize; 2], pixels: &[u8]) -> Result<Self, Scene2DError> {
        // Invert because white is free space and black is occupied space.
        let pixels = pixels.iter().map(|&i| i <= 127).collect();
        let occupancy_map = OccupancyMap::from_pixels(glam::USizeVec2::from(size), pixels)?;

        let scene_loop = Arc::new(Scene2DLoop::default());

        Ok(Self {
            agents: FxHashMap::default(),
            time: SceneTime(0.),
            occupancy_map: Arc::new(occupancy_map),
            scene_loop,
        })
    }

    pub fn state(&self) -> Scene2DState {
        Scene2DState {
            time: self.time,
            occupancy_map: Arc::clone(&self.occupancy_map),
        }
    }

    pub fn update(&mut self, dt: f32) {
        self.time.0 += dt;
        let state = self.state();
        let scene_loop = Arc::clone(&self.scene_loop);

        self.agents.par_iter_mut().for_each_init(|| state.clone(), |state, (id, agent)| {
            agent.update(dt);
            scene_loop.update_state(*id, agent.config, agent.state, state.clone());
        });
    }

    pub fn add_agent(&mut self, agent: Agent2D) -> AgentId {
        let id = AgentId(self.agents.len() as u64);
        self.scene_loop.insert_agent(id, &agent);
        self.agents.insert(id, agent);

        id
    }

    #[inline]
    pub fn in_bounds_vec2(&self, loc: glam::Vec2) -> bool {
        self.occupancy_map.is_valid_vec2(loc)
    }

    #[inline]
    pub fn in_bounds(&self, loc: glam::USizeVec2) -> bool {
        self.occupancy_map.is_valid(loc)
    }

    #[inline]
    pub fn translate(&self, loc: glam::Vec2) -> glam::I64Vec2 {
        self.occupancy_map.translate(loc)
    }

    #[inline]
    pub fn get_box(&self, loc: glam::USizeVec2) -> Box2D {
        self.occupancy_map.get_box(loc)
    }

    #[inline]
    pub fn is_occupied_vec2(&self, loc: glam::Vec2) -> bool {
        self.occupancy_map.is_occupied_vec2(loc)
    }

    #[inline]
    pub fn is_occupied(&self, loc: glam::USizeVec2) -> bool {
        self.occupancy_map.is_occupied(loc)
    }
}

#[derive(thiserror::Error, Debug)]
pub enum Scene2DError {
    #[error("Pixel Size Mismatch: Got {0} pixels but have shape ({width}, {height})", width = .1[0], height = .1[1])]
    PixelSizeMismatch(usize, [usize; 2]),
}
