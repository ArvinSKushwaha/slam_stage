use std::ops::RangeInclusive;

use egui::{Color32, Rect, Shape, Ui};
use egui_plot::{PlotBounds, PlotGeometry, PlotItem, PlotItemBase, PlotPoint, PlotTransform};
use sim::agent::Agent2DMeasurements;

use crate::track_state::TrackState;

fn vec2_to_plotpoint(v: glam::Vec2) -> PlotPoint {
    v.as_dvec2().to_array().into()
}

impl PlotItem for TrackState {
    fn shapes(&self, ui: &Ui, transform: &PlotTransform, shapes: &mut Vec<Shape>) {
        // Track Image
        let image_screen_rect = {
            let size = self.track_texture.size_vec2();
            let left_top = PlotPoint::new(-0.5 * size.x as f64, -0.5 * size.y as f64);
            let right_bottom = PlotPoint::new(0.5 * size.x as f64, 0.5 * size.y as f64);

            let left_top_screen = transform.position_from_point(&left_top);
            let right_bottom_screen = transform.position_from_point(&right_bottom);
            Rect::from_two_pos(left_top_screen, right_bottom_screen)
        };
        egui::paint_texture_at(
            ui.painter(),
            image_screen_rect,
            &egui::ImageOptions {
                uv: Rect::from_min_max(egui::pos2(0., 0.), egui::pos2(1., 1.)),
                bg_fill: ui.style().visuals.window_fill,
                tint: Color32::WHITE,
                rotation: None,
                corner_radius: egui::CornerRadius::ZERO,
            },
            &(self.track_texture.id(), image_screen_rect.size()).into(),
        );

        for (id, agent) in &self.scene.agents {
            let agent_pos = transform
                .position_from_point(&PlotPoint::from(agent.state.position.as_dvec2().to_array()));

            // Agent direction
            {
                let agent_heading = transform.position_from_point(&PlotPoint::from(
                    (agent.state.position + agent.config.length * agent.state.heading)
                        .as_dvec2()
                        .to_array(),
                ));

                shapes.push(Shape::line_segment(
                    [agent_pos, agent_heading],
                    egui::Stroke::new(2.0, Color32::RED),
                ));
            }

            // Agent Body
            {
                let transform_scale = egui::vec2(
                    transform.dpos_dvalue_x() as _,
                    transform.dpos_dvalue_y() as _,
                )
                .abs();

                let flip_y = egui::vec2(1., -1.);
                let front: egui::Vec2 =
                    egui::Vec2::from(mint::Vector2::<f32>::from(agent.state.heading))
                        * transform_scale;
                let left = front.rot90();

                let center = agent_pos;
                let half_extent = egui::Vec2::new(agent.config.length, agent.config.width) * 0.5;

                shapes.push(Shape::convex_polygon(
                    vec![
                        center + (front * half_extent.x - left * half_extent.y) * flip_y,
                        center + (front * half_extent.x + left * half_extent.y) * flip_y,
                        center + (-front * half_extent.x + left * half_extent.y) * flip_y,
                        center + (-front * half_extent.x - left * half_extent.y) * flip_y,
                    ],
                    Color32::DARK_BLUE,
                    (0.0, Color32::TRANSPARENT),
                ));
            }

            // Lidar Measurements
            {
                if let Some(Agent2DMeasurements { lidar: Some(lidar) }) =
                    &self.scene.scene_loop.query(*id)
                {
                    for &point in &lidar.state.0 {
                        let agent_heading = transform.position_from_point(&vec2_to_plotpoint(point));
                        shapes.push(Shape::circle_filled(
                            agent_heading,
                            4.0,
                            Color32::from_white_alpha(70),
                        ));
                    }
                }
            }
        }

        // for segment_collection in self.scene.occupancy_map.boundaries.values() {
        //     for LineSegment(a, b) in segment_collection {
        //         let a = transform.position_from_point(&PlotPoint::from(a.as_dvec2().to_array()));
        //         let b = transform.position_from_point(&PlotPoint::from(b.as_dvec2().to_array()));
        //
        //         let dir = (b - a) * 0.2;
        //         let rot = egui::emath::Rot2::from_angle(0.3);
        //
        //         shapes.push(Shape::line_segment(
        //             [a, b],
        //             egui::Stroke::new(2.0, Color32::GOLD),
        //         ));
        //
        //         shapes.push(Shape::line_segment(
        //             [b - rot * dir, b],
        //             egui::Stroke::new(1.0, Color32::GOLD),
        //         ));
        //     }
        // }
    }

    fn initialize(&mut self, _x_range: RangeInclusive<f64>) {}

    fn color(&self) -> Color32 {
        Color32::TRANSPARENT
    }

    fn geometry(&self) -> PlotGeometry<'_> {
        PlotGeometry::None
    }

    fn bounds(&self) -> PlotBounds {
        let mut bounds = PlotBounds::NOTHING;
        let size = self.track_texture.size_vec2();
        let left_top = PlotPoint::new(-0.5 * size.x as f64, -0.5 * size.y as f64);
        let right_bottom = PlotPoint::new(0.5 * size.x as f64, 0.5 * size.y as f64);

        bounds.extend_with(&left_top);
        bounds.extend_with(&right_bottom);
        bounds
    }

    fn base(&self) -> &PlotItemBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PlotItemBase {
        &mut self.base
    }
}
