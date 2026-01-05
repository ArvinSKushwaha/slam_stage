#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Box2D {
    pub min: glam::Vec2,
    pub max: glam::Vec2,
}

impl Box2D {
    #[inline]
    pub fn size(&self) -> glam::Vec2 {
        self.max - self.min
    }

    #[inline]
    pub fn centroid(&self) -> glam::Vec2 {
        self.max.midpoint(self.min)
    }

    #[inline]
    pub fn contains(&self, point: glam::Vec2) -> bool {
        (point.cmple(self.max) & point.cmpge(self.min)).all()
    }

    #[inline]
    pub fn contains_box(&self, query: &Self) -> bool {
        self.contains(query.min) && self.contains(query.max)
    }

    #[inline]
    pub fn intersects(&self, query: &Self) -> bool {
        let min = self.min.max(query.min);
        let max = self.max.min(query.max);

        min.cmpge(max).any()
    }

    #[inline]
    pub fn encase(&self, other: &Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    pub fn split_vertical(&self) -> [Self; 2] {
        let midpoint = self.centroid();

        [
            Box2D {
                min: self.min,
                max: glam::vec2(self.max.x, midpoint.y),
            },
            Box2D {
                min: glam::vec2(self.min.x, midpoint.y),
                max: self.max,
            },
        ]
    }

    pub fn split_horizontal(&self) -> [Self; 2] {
        let midpoint = self.centroid();

        [
            Box2D {
                min: self.min,
                max: glam::vec2(midpoint.x, self.max.y),
            },
            Box2D {
                min: glam::vec2(midpoint.x, self.min.y),
                max: self.max,
            },
        ]
    }

    pub fn split(&self) -> [Self; 4] {
        let midpoint = self.centroid();

        [
            Box2D {
                min: midpoint,
                max: self.max,
            },
            Box2D {
                min: glam::vec2(midpoint.x, self.min.y),
                max: glam::vec2(self.max.x, midpoint.y),
            },
            Box2D {
                min: self.min,
                max: midpoint,
            },
            Box2D {
                min: glam::vec2(self.min.x, midpoint.y),
                max: glam::vec2(midpoint.x, self.max.y),
            },
        ]
    }
}

#[derive(Debug, Clone, Copy)]
pub struct LineSegment(pub glam::Vec2, pub glam::Vec2);

impl LineSegment {
    #[inline]
    pub const fn reverse(&self) -> Self {
        Self(self.1, self.0)
    }

    #[inline]
    pub fn midpoint(&self) -> glam::Vec2 {
        self.0.midpoint(self.1)
    }

    #[inline]
    pub fn get_box(&self) -> Box2D {
        Box2D {
            min: self.0.min(self.1),
            max: self.0.max(self.1),
        }
    }
}

#[inline]
pub fn intersect_ray_box(
    pos: glam::Vec2,
    dir: glam::Vec2,
    Box2D { min, max }: Box2D,
) -> Option<f32> {
    let center = (min + max) / 2.0;
    let half_extent = (max - min) / 2.0;
    let shifted_pos = pos - center;
    let m = 1.0 / dir;
    let n = m * shifted_pos;
    let k = m.abs() * half_extent;

    let t_n = (-n.x - k.x).max(-n.y - k.y);
    let t_f = (-n.x + k.x).min(-n.y + k.y);

    if t_n > t_f || t_f < f32::EPSILON {
        None
    } else if t_n < f32::EPSILON {
        Some(t_f)
    } else if t_n >= f32::EPSILON {
        Some(t_n)
    } else {
        None
    }
}

#[inline]
pub fn intersect_ray_line_segment(
    pos: glam::Vec2,
    dir: glam::Vec2,
    line_seg: &LineSegment,
) -> Option<f32> {
    let denom = dir.x * (line_seg.1.y - line_seg.0.y) - dir.y * (line_seg.1.x - line_seg.0.x);

    if denom.abs() < f32::EPSILON {
        None
    } else {
        let u_num = dir.x * (pos.y - line_seg.0.y) - dir.y * (pos.x - line_seg.0.x);
        let u = u_num / denom;

        if (0.0..=1.0).contains(&u) {
            let t = ((pos.x - line_seg.0.x) * (line_seg.0.y - line_seg.1.y)
                - (pos.y - line_seg.0.y) * (line_seg.0.x - line_seg.1.x))
                / denom;

            if t > f32::EPSILON { Some(t) } else { None }
        } else {
            None
        }
    }
}

#[cfg(test)]
mod test {
    use crate::math::{Box2D, intersect_ray_box};

    #[test]
    fn test_collisions() {
        assert_eq!(
            intersect_ray_box(
                glam::vec2(0., 0.),
                glam::vec2(0., 1.),
                Box2D {
                    min: glam::vec2(-0.25, 0.25),
                    max: glam::vec2(0.25, 0.75)
                }
            ),
            Some(0.25)
        );

        assert_eq!(
            intersect_ray_box(
                glam::vec2(0., 0.),
                glam::vec2(0., 1.),
                Box2D {
                    min: glam::vec2(-0.25, -0.25),
                    max: glam::vec2(0.25, 0.25)
                }
            ),
            Some(0.25)
        );

        assert_eq!(
            intersect_ray_box(
                glam::vec2(0., 0.),
                glam::vec2(0., 1.),
                Box2D {
                    min: glam::vec2(-0.25, -0.75),
                    max: glam::vec2(0.25, -0.25)
                }
            ),
            None
        );

        assert_eq!(
            intersect_ray_box(
                glam::vec2(0., 0.),
                glam::vec2(1., 0.),
                Box2D {
                    min: glam::vec2(0.25, -0.25),
                    max: glam::vec2(0.75, 0.25)
                }
            ),
            Some(0.25)
        );

        assert_eq!(
            intersect_ray_box(
                glam::vec2(0., 0.),
                glam::vec2(1., 0.),
                Box2D {
                    min: glam::vec2(-0.25, -0.25),
                    max: glam::vec2(0.25, 0.25)
                }
            ),
            Some(0.25)
        );

        assert_eq!(
            intersect_ray_box(
                glam::vec2(0., 0.),
                glam::vec2(1., 0.),
                Box2D {
                    min: glam::vec2(-0.75, -0.25),
                    max: glam::vec2(-0.25, 0.25)
                }
            ),
            None
        );
    }
}
