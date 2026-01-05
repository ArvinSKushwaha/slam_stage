use crate::{
    agent::{Agent2DConfig, Agent2DState},
    scene::Scene2DState,
    sensors::{Sensor2D, TimeStamped},
};
use rayon::prelude::*;
use zerocopy::{ByteEq, ByteHash, Immutable, IntoBytes};

#[derive(Debug, Clone, Default)]
pub struct Lidar2D {
    pub directions: Vec<glam::Vec2>,
}

impl Lidar2D {
    pub fn regular(n: usize) -> Lidar2D {
        let mut directions = Vec::with_capacity(n);
        for angle in (0..n).map(|i| std::f32::consts::TAU * ((i as f32 + 0.5) / n as f32)) {
            directions.push(glam::Vec2::from_angle(angle));
        }

        Lidar2D { directions }
    }

    pub fn set_regular(&mut self, n: usize) {
        self.directions.clear();
        for angle in (0..n).map(|i| std::f32::consts::TAU * ((i as f32 + 0.5) / n as f32)) {
            self.directions.push(glam::Vec2::from_angle(angle));
        }
    }

    pub fn update_directions(&mut self, directions: Vec<glam::Vec2>) {
        self.directions = directions;
    }
}

// #[inline]
// pub fn intersect_ray_line_segment(
//     pos: glam::Vec2,
//     dir: glam::Vec2,
//     LineSegment(start, end): &LineSegment,
// ) -> Option<f32> {
//     let shift = start - pos;
//     let disp = end - start;
//
//     let denom = dir.perp_dot(disp);
//
//     if denom.abs() < EPSILON {
//         None
//     } else {
//         let u_num = dir.perp_dot(shift);
//         let u = -u_num / denom;
//
//         if (0.0..=1.0).contains(&u) {
//             let t = shift.perp_dot(disp) / denom;
//
//             if t > EPSILON { Some(t) } else { None }
//         } else {
//             None
//         }
//     }
// }

/// Use the bit-representations in the hash-map since [glam::Vec2] is not typically hashable.
#[derive(Debug, Copy, Clone, ByteHash, ByteEq, Immutable, IntoBytes)]
pub struct HashVec2(pub glam::Vec2);

impl std::ops::Deref for HashVec2 {
    type Target = glam::Vec2;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl std::ops::DerefMut for HashVec2 {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Lidar2DSensed(pub Vec<glam::Vec2>);

impl Sensor2D for Lidar2D {
    type SensorType = Lidar2DSensed;

    // fn sense(&mut self, agent: &Agent2D, scene: &Scene2D) -> Self::SensorType {
    //     log::info!("Sensing surroundings with Lidar");
    //     let start = std::time::Instant::now();
    //
    //     let loc = scene.translate(agent.position);
    //
    //     if loc.cmplt(glam::I64Vec2::ZERO).any() || scene.is_occupied(loc.as_usizevec2()) {
    //         let output = Arc::new(Lidar2DSensed {
    //             position: agent.position,
    //             heading: agent.heading,
    //             measurements: self
    //                 .directions
    //                 .iter()
    //                 .map(|&dir| (HashVec2(dir), 0.0))
    //                 .collect(),
    //         });
    //
    //         self.last_sensed = Some(Arc::clone(&output));
    //         return output;
    //     }
    //
    //     let loc = loc.as_usizevec2();
    //
    //     let mut query_directions: Vec<_> = self
    //         .directions
    //         .iter()
    //         .map(|dir| (*dir, agent.heading.rotate(*dir)))
    //         .collect();
    //
    //     // Already visited
    //     let mut visited = FxHashSet::default();
    //     // The set of empty nodes to visit
    //     let mut vacuum_nodes = VecDeque::new();
    //     // The set of non-empty nodes to visit
    //     let mut query_nodes: VecDeque<glam::USizeVec2> = VecDeque::new();
    //
    //     let mut results = FxHashMap::default();
    //
    //     vacuum_nodes.push_back(loc);
    //     visited.insert(loc);
    //
    //     while !query_directions.is_empty() {
    //         query_nodes.iter().for_each(|&query_node| {
    //             let box2d = scene.get_box(query_node);
    //             query_directions.retain(|&(dir, local_dir)| {
    //                 if let Some(dist) = intersect_ray_box(agent.position, local_dir, box2d) {
    //                     results.insert(HashVec2(dir), dist);
    //                     false
    //                 } else {
    //                     true
    //                 }
    //             });
    //         });
    //
    //         query_nodes.clear();
    //
    //         // Sort neighbors into vacuum or query
    //         {
    //             for _ in 0..5 {
    //                 if let Some(vacuum_node) = vacuum_nodes.pop_front() {
    //                     let neighbor_left = vacuum_node - glam::USizeVec2::X;
    //                     let neighbor_right = vacuum_node + glam::USizeVec2::X;
    //                     let neighbor_up = vacuum_node - glam::USizeVec2::Y;
    //                     let neighbor_down = vacuum_node + glam::USizeVec2::Y;
    //
    //                     if vacuum_node.x > 0 {
    //                         add_relevant(
    //                             neighbor_left,
    //                             &mut vacuum_nodes,
    //                             &mut query_nodes,
    //                             scene,
    //                             &mut visited,
    //                         );
    //                     }
    //                     if vacuum_node.y > 0 {
    //                         add_relevant(
    //                             neighbor_up,
    //                             &mut vacuum_nodes,
    //                             &mut query_nodes,
    //                             scene,
    //                             &mut visited,
    //                         );
    //                     }
    //                     if scene.in_bounds(neighbor_right) {
    //                         add_relevant(
    //                             neighbor_right,
    //                             &mut vacuum_nodes,
    //                             &mut query_nodes,
    //                             scene,
    //                             &mut visited,
    //                         );
    //                     }
    //                     if scene.in_bounds(neighbor_down) {
    //                         add_relevant(
    //                             neighbor_down,
    //                             &mut vacuum_nodes,
    //                             &mut query_nodes,
    //                             scene,
    //                             &mut visited,
    //                         );
    //                     }
    //                 } else {
    //                     break;
    //                 }
    //             }
    //         }
    //     }
    //
    //     let sensed = Arc::new(Lidar2DSensed {
    //         position: agent.position,
    //         heading: agent.heading,
    //         measurements: results,
    //     });
    //
    //     self.last_sensed.replace(Arc::clone(&sensed));
    //     log::info!(
    //         "Sensing surroundings took {} ms",
    //         start.elapsed().as_millis()
    //     );
    //
    //     sensed
    // }

    fn sense(
        &self,
        _agent_config: Agent2DConfig,
        agent_state: Agent2DState,
        scene: Scene2DState,
    ) -> Option<TimeStamped<Self::SensorType>> {
        log::info!("Sensing surroundings with Lidar");
        let start = std::time::Instant::now();

        let loc = scene.occupancy_map.translate(agent_state.position);

        if loc.cmplt(glam::I64Vec2::ZERO).any()
            || scene.occupancy_map.is_occupied(loc.as_usizevec2())
        {
            return None;
        }

        let results: Vec<glam::Vec2> = self
            .directions
            .par_iter()
            .flat_map(|&dir| {
                let world_dir = agent_state.heading.rotate(dir);
                scene
                    .occupancy_map
                    .cast_rays(agent_state.position, world_dir)
                    .map(|i| world_dir * i + agent_state.position)
            })
            .collect();

        // log::debug!("{results:?}");

        let sensed = TimeStamped {
            time: scene.time,
            state: Lidar2DSensed(results),
        };

        log::info!(
            "Sensing surroundings took {} ms",
            start.elapsed().as_millis()
        );

        Some(sensed)
    }
}

// #[inline(always)]
// fn add_relevant(
//     neighbor: glam::USizeVec2,
//     vacuum_nodes: &mut VecDeque<glam::USizeVec2>,
//     query_nodes: &mut VecDeque<glam::USizeVec2>,
//     scene: &Scene2D,
//     visited: &mut FxHashSet<glam::USizeVec2>,
// ) {
//     if visited.contains(&neighbor) {
//         return;
//     }
//     {
//         if scene.is_occupied(neighbor) {
//             query_nodes.push_back(neighbor);
//         } else {
//             vacuum_nodes.push_back(neighbor);
//         }
//     }
//
//     visited.insert(neighbor);
// }
