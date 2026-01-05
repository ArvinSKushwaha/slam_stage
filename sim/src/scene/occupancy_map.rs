use std::collections::VecDeque;

use rustc_hash::FxHashSet;

use crate::{bvh::{BVH, Direction}, math::{Box2D, LineSegment, intersect_ray_box, intersect_ray_line_segment}, scene::Scene2DError};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ObjectTag(u64);

#[derive(Debug, Clone)]
pub struct OccupancyMap {
    pub size: glam::USizeVec2,
    pub pixels: Vec<bool>,
    pub objects: Vec<Option<ObjectTag>>,
    pub boundaries: Vec<LineSegment>,
    pub bvh: BVH,
}

#[inline]
fn boundary_direction(
    size: glam::USizeVec2,
    node: glam::USizeVec2,
    direction: Direction,
) -> LineSegment {
    let size = size.as_vec2();
    let node = node.as_vec2();

    let top_left = glam::vec2(node.x - size.x / 2., size.y / 2. - node.y);

    match direction {
        Direction::North => LineSegment(top_left, top_left + glam::Vec2::X),
        Direction::East => LineSegment(
            top_left + glam::Vec2::X,
            top_left + glam::Vec2::X + glam::Vec2::NEG_Y,
        ),
        Direction::South => LineSegment(
            top_left + glam::Vec2::X + glam::Vec2::NEG_Y,
            top_left + glam::Vec2::NEG_Y,
        ),
        Direction::West => LineSegment(top_left + glam::Vec2::NEG_Y, top_left),
    }
}

impl OccupancyMap {
    #[inline]
    pub fn is_valid_vec2(&self, loc: glam::Vec2) -> bool {
        loc.abs().cmplt(self.size.as_vec2() / 2.).all()
    }

    #[inline]
    pub fn is_valid(&self, loc: glam::USizeVec2) -> bool {
        loc.cmplt(self.size).all()
    }

    #[inline]
    pub fn translate(&self, loc: glam::Vec2) -> glam::I64Vec2 {
        const FLIP_HORIZONTAL: glam::Vec2 = glam::Vec2::new(-1., 1.);
        let origin_corner = (self.size.as_vec2() / 2.) * FLIP_HORIZONTAL;

        ((origin_corner - loc) * FLIP_HORIZONTAL)
            .floor()
            .as_i64vec2()
    }

    #[inline]
    pub fn get_box(&self, loc: glam::USizeVec2) -> Box2D {
        const FLIP_HORIZONTAL: glam::Vec2 = glam::Vec2::new(-1., 1.);

        let origin_corner = (self.size.as_vec2() / 2.) * FLIP_HORIZONTAL;
        let top_left = origin_corner - loc.as_vec2() * FLIP_HORIZONTAL;
        let bottom_right = top_left + glam::vec2(1., -1.);

        Box2D {
            min: top_left.min(bottom_right),
            max: top_left.max(bottom_right),
        }
    }

    #[inline]
    pub fn is_occupied_vec2(&self, loc: glam::Vec2) -> bool {
        if !self.is_valid_vec2(loc) {
            log::trace!("Out of bounds: {loc}");
            return true;
        }

        let loc = self.translate(loc).as_usizevec2();
        log::trace!("Checking Occupied: {loc}");
        self.pixels[loc[0] + loc[1] * self.size.x]
    }

    #[inline]
    pub fn is_occupied(&self, loc: glam::USizeVec2) -> bool {
        if self.is_valid(loc) {
            self.pixels[loc.x + loc.y * self.size.x]
        } else {
            true
        }
    }

    pub fn from_pixels(size: glam::USizeVec2, pixels: Vec<bool>) -> Result<OccupancyMap, Scene2DError> {
        let [width, height] = size.to_array();
        let expected_count = size[0] * size[1];
        let pixels_len = pixels.len();

        let mut objects = vec![None; pixels_len];
        let mut visited = FxHashSet::<glam::USizeVec2>::default();
        let mut boundaries = Vec::new();
        let mut tmp_nodes = vec![];

        let mut object_count = 0;

        for (i, &pixel) in pixels.iter().enumerate() {
            if !pixel || objects[i].is_some() {
                continue;
            }

            // Start DFS
            let object = ObjectTag(object_count);
            object_count += 1;

            objects[i] = Some(object);

            let loc = glam::usizevec2(i % width, i / width);
            tmp_nodes.push(loc);

            while let Some(node) = tmp_nodes.pop() {
                visited.insert(node);

                let mut try_add = |node: glam::USizeVec2| {
                    let k = node.x + node.y * width;

                    if visited.contains(&node) {
                        return false;
                    }

                    if !pixels[k] {
                        true
                    } else {
                        objects[k] = Some(object);
                        tmp_nodes.push(node);
                        false
                    }
                };

                if node.x > 0 && try_add(node - glam::USizeVec2::X) {
                    boundaries.push(boundary_direction(size, node, Direction::West));
                }

                if node.x < width - 1 && try_add(node + glam::USizeVec2::X) {
                    boundaries.push(boundary_direction(size, node, Direction::East));
                }

                if node.y > 0 && try_add(node - glam::USizeVec2::Y) {
                    boundaries.push(boundary_direction(size, node, Direction::North));
                }

                if node.y < height - 1 && try_add(node + glam::USizeVec2::Y) {
                    boundaries.push(boundary_direction(size, node, Direction::South));
                }
            }
        }

        let bvh = BVH::new(boundaries.iter());

        if expected_count == pixels_len {
            Ok(Self {
                pixels,
                size,
                objects,
                boundaries,
                bvh,
            })
        } else {
            Err(Scene2DError::PixelSizeMismatch(pixels_len, size.into()))
        }
    }

    pub fn cast_rays(&self, pos: glam::Vec2, dir: glam::Vec2) -> Option<f32> {
        let BVH { box_map, root } = &self.bvh;

        let mut queue = VecDeque::new();
        queue.push_back(*root);

        let mut min = f32::INFINITY;

        while let Some(node_id) = queue.pop_front() {
            let Some(node) = box_map.get(&node_id) else {
                continue;
            };

            if intersect_ray_box(pos, dir, node.rect).is_some() {
                if let Some(children) = &node.children {
                    for child in children {
                        queue.push_back(*child);
                    }
                }

                if let Some(elements) = &node.elements {
                    for &indices in elements {
                        if let Some(small) =
                            intersect_ray_line_segment(pos, dir, &self.boundaries[indices])
                        {
                            min = min.min(small);
                        }
                    }
                }
            }
        }

        if min != f32::INFINITY {
            Some(min)
        } else {
            None
        }
    }
}
