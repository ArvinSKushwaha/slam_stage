use crate::math::{Box2D, LineSegment};
use dashmap::DashMap;
use rayon::prelude::*;
use rustc_hash::FxBuildHasher;
use smallvec::{SmallVec, smallvec};
use std::{hash::BuildHasher, sync::atomic::AtomicU64};

const MAX_PRIMS_IN_NODE: usize = 16;

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
pub struct BVHNodeId(u64);

#[derive(Debug, Clone)]
pub struct BVH {
    pub box_map: DashMap<BVHNodeId, BVHNode, FxBuildHasher>,
    pub root: BVHNodeId,
}

#[derive(Debug, Clone)]
pub struct BVHNode {
    pub children: Option<SmallVec<[BVHNodeId; 2]>>,
    pub rect: Box2D,
    pub elements: Option<SmallVec<[usize; MAX_PRIMS_IN_NODE]>>,
}

fn embed_even_bits(x: u32) -> u64 {
    let mut x = x as u64;
    x = (x | (x << 16)) & 0x0000FFFF0000FFFF;
    x = (x | (x << 8)) & 0x00FF00FF00FF00FF;
    x = (x | (x << 4)) & 0x0F0F0F0F0F0F0F0F;
    x = (x | (x << 2)) & 0x3333333333333333;
    x = (x | (x << 1)) & 0x5555555555555555;

    x
}

// fn extract_even_bits(mut x: u64) -> u32 {
//     x &= 0x5555555555555555;
//     x = (x | (x >> 1)) & 0x3333333333333333;
//     x = (x | (x >> 2)) & 0x0F0F0F0F0F0F0F0F;
//     x = (x | (x >> 4)) & 0x00FF00FF00FF00FF;
//     x = (x | (x >> 8)) & 0x0000FFFF0000FFFF;
//     x = (x | (x >> 16)) & 0x00000000FFFFFFFF;
//
//     x as u32
// }

fn morton_encode(x: u32, y: u32) -> u64 {
    embed_even_bits(x) | (embed_even_bits(y) << 1)
}

// fn morton_decode(n: u64) -> (u32, u32) {
//     (extract_even_bits(n), extract_even_bits(n >> 1))
// }

#[derive(Debug, Clone, Copy)]
pub enum Direction {
    North,
    East,
    South,
    West,
}

impl BVH {
    pub fn new<'a>(segments: impl Iterator<Item = &'a LineSegment>) -> Self {
        let mut boxes = Vec::with_capacity(segments.size_hint().0);
        let mut bounding: Option<Box2D> = None;

        for (i, segment) in segments.enumerate() {
            let bx = segment.get_box();

            if let Some(b) = &mut bounding {
                *b = b.encase(&bx);
            } else {
                bounding = Some(bx);
            }

            boxes.push((i, bx, 0u64));
        }

        let Some(bounding) = bounding else {
            let box_map = DashMap::default();

            let node_id = BVHNodeId(0);
            let node = BVHNode {
                children: None,
                rect: Box2D {
                    min: glam::Vec2::ZERO,
                    max: glam::Vec2::ZERO,
                },
                elements: None,
            };

            box_map.insert(node_id, node);

            return BVH {
                box_map,
                root: node_id,
            };
        };

        boxes.par_iter_mut().for_each(|(_, bx, morton)| {
            bx.min = (bx.min - bounding.min) / (bounding.max - bounding.min);
            bx.max = (bx.max - bounding.min) / (bounding.max - bounding.min);

            let centroid = bx.centroid() * (1 << 20) as f32;
            *morton = morton_encode(centroid.x as u32, centroid.y as u32);
        });
        boxes.par_sort_unstable_by_key(|i| i.2);

        let mask = 0xFFFFFFFFFF000000u64;

        let mut start = 0;
        let mut end = 1;

        let mut treelets = Vec::new();

        while end <= boxes.len() {
            if end == boxes.len() || ((boxes[start].2 & mask) != (boxes[end].2 & mask)) {
                treelets.push(start..end);
                start = end;
            }
            end += 1;
        }

        let first_bit_index = mask.trailing_zeros() as i8;

        fn emit_lbvh<S: BuildHasher + Clone>(
            index: i8,
            range: std::ops::Range<usize>,
            boxes: &[(usize, Box2D, u64)],
            node_number: &AtomicU64,
            box_map: &DashMap<BVHNodeId, BVHNode, S>,
        ) -> (BVHNodeId, BVHNode) {
            // dbg!(index, &range, boxes, node_number, box_map);
            if index < 0 || range.len() <= MAX_PRIMS_IN_NODE {
                let rect = boxes[range.clone()]
                    .iter()
                    .map(|(_, bx, _)| bx)
                    .copied()
                    .reduce(|a_bx, b_bx| a_bx.encase(&b_bx));

                (
                    BVHNodeId(node_number.fetch_add(1, std::sync::atomic::Ordering::SeqCst)),
                    BVHNode {
                        elements: Some(boxes[range].iter().map(|&(i, _, _)| i).collect()),
                        rect: rect.unwrap_or(Box2D {
                            min: glam::Vec2::ZERO,
                            max: glam::Vec2::ZERO,
                        }),
                        children: None,
                    },
                )
            } else {
                let mask: u64 = 1 << index;
                if (boxes[range.start].2 & mask) == (boxes[range.end - 1].2 & mask) {
                    return emit_lbvh(index - 1, range, boxes, node_number, box_map);
                }

                let split = range.start
                    + boxes[range.clone()].partition_point(|(_, _, morton)| {
                        (morton & mask) == (boxes[range.start].2 & mask)
                    });

                let (id1, node1) =
                    emit_lbvh(index - 1, range.start..split, boxes, node_number, box_map);
                let (id2, node2) =
                    emit_lbvh(index - 1, split..range.end, boxes, node_number, box_map);

                let rect = node1.rect.encase(&node2.rect);

                box_map.insert(id1, node1);
                box_map.insert(id2, node2);

                (
                    BVHNodeId(node_number.fetch_add(1, std::sync::atomic::Ordering::SeqCst)),
                    BVHNode {
                        children: Some(smallvec![id1, id2]),
                        rect,
                        elements: None,
                    },
                )
            }
        }

        let box_map = DashMap::<BVHNodeId, BVHNode, FxBuildHasher>::default();
        let node_number = AtomicU64::new(0);
        let mut treelets = treelets
            .par_iter()
            .map(|i| {
                let (id, node) =
                    emit_lbvh(first_bit_index, i.clone(), &boxes, &node_number, &box_map);

                let rect = node.rect;
                box_map.insert(id, node);

                (id, rect)
            })
            .collect::<Vec<_>>();

        fn make_split_tree<S: BuildHasher + Clone>(
            depth: i8,
            horizontal: bool,
            range: std::ops::Range<usize>,
            boxes: &mut [(BVHNodeId, Box2D)],
            node_number: &AtomicU64,
            box_map: &DashMap<BVHNodeId, BVHNode, S>,
            bounding: Box2D,
        ) -> (BVHNodeId, BVHNode) {
            if depth < 0 || range.len() <= 2 {
                let rect = boxes[range.clone()]
                    .iter()
                    .map(|(_, bx)| bx)
                    .copied()
                    .reduce(|a_bx, b_bx| a_bx.encase(&b_bx));

                let children = boxes[range]
                    .iter()
                    .map(|&(id, _)| id)
                    .collect::<SmallVec<_>>();

                (
                    BVHNodeId(node_number.fetch_add(1, std::sync::atomic::Ordering::SeqCst)),
                    BVHNode {
                        children: if !children.is_empty() {
                            Some(children)
                        } else {
                            None
                        },
                        rect: rect.unwrap_or(Box2D {
                            min: glam::Vec2::ZERO,
                            max: glam::Vec2::ZERO,
                        }),
                        elements: None,
                    },
                )
            } else {
                let [first, second] = if horizontal {
                    bounding.split_horizontal()
                } else {
                    bounding.split_vertical()
                };

                boxes[range.clone()].sort_by_key(|(_, node)| !first.contains(node.centroid()));
                // dbg!(boxes.iter().map(|(id, node)| (id, node, node.centroid(), first.contains(node.centroid()))).collect::<Vec<_>>());

                let split = range.start
                    + boxes[range.clone()]
                        .partition_point(|(_, node)| first.contains(node.centroid()));

                // dbg!(
                //     horizontal,
                //     &range,
                //     &boxes,
                //     node_number,
                //     box_map,
                //     first,
                //     second,
                //     split
                // );
                // todo!();

                let (id1, node1) = make_split_tree(
                    depth - 1,
                    !horizontal,
                    range.start..split,
                    boxes,
                    node_number,
                    box_map,
                    first,
                );
                let (id2, node2) = make_split_tree(
                    depth - 1,
                    !horizontal,
                    split..range.end,
                    boxes,
                    node_number,
                    box_map,
                    second,
                );

                let rect = node1.rect.encase(&node2.rect);

                box_map.insert(id1, node1);
                box_map.insert(id2, node2);

                (
                    BVHNodeId(node_number.fetch_add(1, std::sync::atomic::Ordering::SeqCst)),
                    BVHNode {
                        children: Some(smallvec![id1, id2]),
                        rect,
                        elements: None,
                    },
                )
            }
        }

        let (id, node) = make_split_tree(
            5,
            true,
            0..treelets.len(),
            &mut treelets,
            &node_number,
            &box_map,
            Box2D {
                min: glam::vec2(0., 0.),
                max: glam::vec2(1., 1.),
            },
        );
        box_map.insert(id, node);

        box_map.iter_mut().for_each(|mut r| {
            let bx = &mut r.rect;
            bx.min = bx.min * (bounding.max - bounding.min) + bounding.min;
            bx.max = bx.max * (bounding.max - bounding.min) + bounding.min;
        });

        Self { box_map, root: id }
    }
}
