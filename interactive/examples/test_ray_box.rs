use eframe::{NativeOptions, egui};
use egui::{Color32, Slider};
use egui_plot::{Line, Plot, PlotPoint, PlotPoints, Polygon};
use sim::math::{Box2D, intersect_ray_box};

trait ShapeExt {
    fn circle(name: impl Into<String>, center: impl Into<PlotPoint>, radius: f64, n: usize)
    -> Self;
    fn rect(name: impl Into<String>, center: impl Into<PlotPoint>, extent: [f64; 2]) -> Self;
}

impl ShapeExt for Polygon<'_> {
    fn circle(
        name: impl Into<String>,
        center: impl Into<PlotPoint>,
        radius: f64,
        n: usize,
    ) -> Self {
        let center = center.into();

        let mut points = Vec::with_capacity(n);
        for angle in (0..n).map(|i| i as f64 * std::f64::consts::TAU / n as f64) {
            let (s, c) = angle.sin_cos();

            points.push([center.x + c * radius, center.y + s * radius]);
        }

        Polygon::new(name.into(), points)
    }

    fn rect(name: impl Into<String>, center: impl Into<PlotPoint>, half_extent: [f64; 2]) -> Self {
        let center = center.into();
        let half_extent = glam::DVec2::from_array(half_extent);

        let points = vec![
            [center.x - half_extent.x, center.y + half_extent.y],
            [center.x + half_extent.x, center.y + half_extent.y],
            [center.x + half_extent.x, center.y - half_extent.y],
            [center.x - half_extent.x, center.y - half_extent.y],
        ];

        Polygon::new(name.into(), points)
    }
}

fn main() -> eframe::Result {
    let mut pos = glam::Vec2::ZERO;
    let mut theta = 0.0;

    let mut cor1 = glam::vec2(-1., -1.);
    let mut cor2 = glam::vec2(1., 1.);

    eframe::run_simple_native(
        "Test Ray Box",
        NativeOptions::default(),
        move |ctx, _frame| {
            egui::Window::new("Config").show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("x:");
                    ui.add(Slider::new(&mut pos.x, (-10.)..=10.));
                    ui.label("y:");
                    ui.add(Slider::new(&mut pos.y, (-10.)..=10.));
                    ui.label("theta:");
                    ui.drag_angle(&mut theta);
                });

                ui.horizontal(|ui| {
                    ui.label("corner1.x:");
                    ui.add(Slider::new(&mut cor1.x, (-10.)..=10.));
                    ui.label("corner1.y:");
                    ui.add(Slider::new(&mut cor1.y, (-10.)..=10.));
                });

                ui.horizontal(|ui| {
                    ui.label("corner2.x:");
                    ui.add(Slider::new(&mut cor2.x, (-10.)..=10.));
                    ui.label("corner2.y:");
                    ui.add(Slider::new(&mut cor2.y, (-10.)..=10.));
                });
            });

            egui::CentralPanel::default().show(ctx, |ui| {
                Plot::new("display")
                    .width(ui.available_width())
                    .height(ui.available_height())
                    .default_x_bounds(-5., 5.)
                    .default_y_bounds(-5., 5.)
                    .data_aspect(1.0)
                    .show(ui, |plot_ui| {
                        let size = plot_ui
                            .plot_bounds()
                            .height()
                            .max(plot_ui.plot_bounds().width());

                        plot_ui.polygon(
                            Polygon::circle("point", pos.as_dvec2().to_array(), 0.05, 100)
                                .fill_color(Color32::RED)
                                .width(0.),
                        );

                        let (cor1, cor2) = (cor1.min(cor2), cor1.max(cor2));
                        plot_ui.polygon(
                            Polygon::rect(
                                "box",
                                ((cor1 + cor2) / 2.).as_dvec2().to_array(),
                                ((cor2 - cor1) / 2.).as_dvec2().to_array(),
                            )
                            .width(0.)
                            .fill_color(Color32::LIGHT_RED.linear_multiply(0.5)),
                        );

                        plot_ui.line(Line::new(
                            "ray_line",
                            PlotPoints::from_parametric_callback(
                                |t| {
                                    let out =
                                        pos.as_dvec2() + glam::DVec2::from_angle(theta as f64) * t;
                                    (out.x, out.y)
                                },
                                0.0..=size,
                                10,
                            ),
                        ));

                        let dir = glam::Vec2::from_angle(theta);
                        if let Some(dist) = intersect_ray_box(pos, dir, Box2D {min: cor1, max: cor2}) {
                            plot_ui.polygon(Polygon::circle(
                                "point",
                                (pos + dist * dir).as_dvec2().to_array(),
                                0.05,
                                100,
                            ).width(0.).fill_color(Color32::WHITE));
                        }
                    });
            });
        },
    )
}
