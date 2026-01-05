use std::collections::VecDeque;
use std::path::PathBuf;

use crate::track_file::{LidarFile, TrackFile};
use crate::track_state::{TrackLoadError, TrackRenderState, TrackState};
use eframe::egui::Color32;
use eframe::{CreationContext, egui};
use egui_file_dialog::FileDialog;
use sim::Agent2D;
use sim::math::Box2D;

pub struct App {
    durations: VecDeque<f32>,
    track_file: String,
    track_load_error: String,
    track_file_dialog: FileDialog,
    lidar_count: usize,
    track_state: Option<TrackState>,
    last_time: std::time::Instant,
    paused: bool,
}

impl App {
    pub fn new(cc: &CreationContext) -> anyhow::Result<Self> {
        let mut fonts = egui::FontDefinitions::default();
        egui_nerdfonts::add_to_fonts(&mut fonts, egui_nerdfonts::Variant::Regular);

        cc.egui_ctx.set_fonts(fonts);

        cc.egui_ctx
            .style_mut(|s| s.drag_value_text_style = egui::TextStyle::Monospace);

        let app = App {
            durations: VecDeque::new(),
            track_file: String::new(),
            track_load_error: String::new(),
            track_file_dialog: FileDialog::new(),
            lidar_count: 60,
            track_state: Default::default(),
            last_time: std::time::Instant::now(),
            paused: false,
        };

        Ok(app)
    }

    pub fn reset_track(&mut self) {
        log::info!("Resetting TrackState");
        self.track_state = None;
    }
    pub fn load_track_state(
        &mut self,
        track_render_state: TrackRenderState,
        ctx: &egui::Context,
    ) -> Result<(), TrackLoadError> {
        let path = &self.track_file;
        log::debug!("Loading {path:?}");
        let file = std::fs::File::open(path)?;

        let track_file: TrackFile = serde_yml::from_reader(file)?;

        let agents = track_file
            .agents
            .iter()
            .map(|f| {
                let mut agent = Agent2D::with_scale(f.scale);
                agent.state.position = f.position;
                agent.state.heading = f.heading;

                match f.lidar {
                    LidarFile::Count { count } => {
                        self.lidar_count = count;
                        agent.sensors.lidar.write_arc().set_regular(count);
                    }
                }

                agent
            })
            .collect::<Vec<_>>();

        let path = PathBuf::from(path).canonicalize()?;
        let image_path = if let Some(parent) = path.parent() {
            parent.join(&track_file.track)
        } else {
            PathBuf::from("/").join(&track_file.track)
        };

        let mut track_state = TrackState::load(
            image_path,
            track_file.threshold,
            track_render_state,
            agents,
            ctx,
        )?;

        if track_state.track_render_state.active.is_none() {
            track_state.track_render_state.active = track_state.scene.agents.keys().next().copied();
        }

        self.track_state = Some(track_state);
        self.last_time = std::time::Instant::now();

        Ok(())
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        catppuccin_egui::set_theme(ctx, catppuccin_egui::MOCHA);

        egui::Window::new("Config")
            .collapsible(true)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("File:");

                    if ui.button("Pick").clicked() {
                        self.track_file_dialog.pick_file();
                    }

                    self.track_file_dialog.update(ctx);

                    if let Some(path) = self.track_file_dialog.take_picked() {
                        self.track_file = path.to_string_lossy().to_string();
                    }

                    let file_edit = egui::TextEdit::singleline(&mut self.track_file)
                        .text_color(if self.track_load_error.is_empty() {
                            Color32::GREEN
                        } else {
                            Color32::RED
                        })
                        .code_editor()
                        .hint_text(egui::WidgetText::from("File Path").italics())
                        .show(ui);

                    if ctx.cumulative_pass_nr() == 0 {
                        file_edit.response.request_focus();
                    }

                    if file_edit.response.lost_focus()
                        && ui.input(|inp| inp.key_pressed(egui::Key::Enter))
                        || ui.button("Load").clicked()
                    {
                        self.reset_track();
                        if let Err(err) = self.load_track_state(TrackRenderState::default(), ctx) {
                            log::error!("{}", err);
                            self.track_load_error = format!("{err}");
                        } else {
                            self.track_load_error.clear();
                        }
                    }
                });

                if let Some(track_state) = &mut self.track_state
                    && let Some(agent) = &track_state.track_render_state.active
                {
                    ui.separator();

                    ui.horizontal(|ui| {
                        ui.label("Position");
                        ui.add_space(10.);
                        ui.add_sized(
                            [70., 20.],
                            egui::DragValue::new(
                                &mut track_state
                                    .scene
                                    .agents
                                    .get_mut(agent)
                                    .unwrap()
                                    .state
                                    .position
                                    .x,
                            )
                            .prefix("← x: ")
                            .suffix(" →"),
                        );
                        ui.add_sized(
                            [70., 20.],
                            egui::DragValue::new(
                                &mut track_state
                                    .scene
                                    .agents
                                    .get_mut(agent)
                                    .unwrap()
                                    .state
                                    .position
                                    .y,
                            )
                            .prefix("← y: ")
                            .suffix(" →"),
                        );
                    });

                    ui.horizontal(|ui| {
                        ui.label("Kinetics");
                        ui.add_space(10.);
                        ui.add_sized(
                            [70., 20.],
                            egui::DragValue::new(
                                &mut track_state
                                    .scene
                                    .agents
                                    .get_mut(agent)
                                    .unwrap()
                                    .state
                                    .velocity,
                            )
                            .prefix("← velocity: ")
                            .suffix(" →"),
                        );
                        ui.add_sized(
                            [70., 20.],
                            egui::DragValue::new(
                                &mut track_state
                                    .scene
                                    .agents
                                    .get_mut(agent)
                                    .unwrap()
                                    .state
                                    .torque,
                            )
                            .prefix("← torque: ")
                            .suffix(" →"),
                        );
                        ui.add_sized(
                            [70., 20.],
                            egui::DragValue::new(
                                &mut track_state.scene.agents.get_mut(agent).unwrap().state.beta,
                            )
                            .prefix("← beta: ")
                            .suffix(" →"),
                        );
                    });
                }
            });

        egui::TopBottomPanel::bottom("bottom").show(ctx, |ui| {
            ui.horizontal(|ui| {
                if ui
                    .add(
                        egui::Button::new(if self.paused { "" } else { "" })
                            .small()
                            .frame(false),
                    )
                    .clicked()
                {
                    self.paused = !self.paused;
                }
                ui.add_space(5.);

                ui.label("FPS:");

                let fps = if self.durations.len() > 5 {
                    (self.durations.iter().sum::<f32>() / self.durations.len() as f32).recip()
                } else {
                    0.0
                };

                ui.label(format!("{fps:.5}"));
                ui.add_space(50.);
                if !self.track_load_error.is_empty() {
                    ui.colored_label(Color32::RED, &self.track_load_error);
                }
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.style_mut().visuals.override_text_color = Some(Color32::from_white_alpha(70));
            let resp = egui_plot::Plot::new("main_plot")
                .show_x(false)
                .show_y(false)
                .width(ui.available_width())
                .height(ui.available_height())
                .data_aspect(1.0)
                .show(ui, |plot_ui| {
                    if let Some(track @ TrackState { .. }) = &self.track_state {
                        plot_ui.add(track.clone());
                    }
                });

            // Check if agent selected
            if resp.response.clicked() {
                let pointer = resp.response.interact_pointer_pos().unwrap();
                let pos = resp.transform.value_from_position(pointer);
                let pos = glam::vec2(pos.x as f32, pos.y as f32);

                if let Some(track_state) = &mut self.track_state {
                    track_state.track_render_state.active = None;

                    for (&id, agent) in &track_state.scene.agents {
                        let mut heading = agent.state.heading;
                        let agent_pos = agent.state.position;
                        let agent_size = glam::vec2(agent.config.length, agent.config.width);

                        heading.y *= -heading.y;
                        let body_view_pos = heading.rotate(pos - agent_pos);
                        let bbox = Box2D {
                            min: -agent_size / 2.,
                            max: agent_size / 2.,
                        };
                        if bbox.contains(body_view_pos) {
                            track_state.track_render_state.active = Some(id);
                            break;
                        }
                    }
                }
            }
        });

        ctx.request_repaint();
        if let Some(track_state) = &mut self.track_state {
            let dt = ctx.input(|i| i.unstable_dt);
            if !self.paused {
                track_state.scene.update(dt);
            }

            if ctx.input(|i| i.key_pressed(egui::Key::Space)) {
                self.paused = !self.paused;
            }

            if let Some(active) = &track_state.track_render_state.active {
                let Agent2D { config, state, .. } =
                    track_state.scene.agents.get_mut(active).unwrap();
                let config = &*config;

                let keys = ctx.input(|i| i.keys_down.clone());

                if keys.contains(&egui::Key::ArrowUp) {
                    state.torque += (config.torque_range.1 - config.torque_range.0) * dt * 0.2;
                }

                if keys.contains(&egui::Key::ArrowDown) {
                    state.torque -= (config.torque_range.1 - config.torque_range.0) * dt * 0.2;
                }

                if keys.contains(&egui::Key::ArrowLeft) {
                    state.beta += (config.beta_range.1 - config.beta_range.0) * dt * 0.2;
                }

                if keys.contains(&egui::Key::ArrowRight) {
                    state.beta -= (config.beta_range.1 - config.beta_range.0) * dt * 0.2;
                }

                state.torque = state
                    .torque
                    .clamp(config.torque_range.0, config.torque_range.1);
                state.beta = state.beta.clamp(config.beta_range.0, config.beta_range.1);
            }
        }

        if self.durations.len() > 100 {
            let _ = self.durations.pop_front();
        }

        self.durations
            .push_back(self.last_time.elapsed().as_secs_f32());

        self.last_time = std::time::Instant::now();
    }
}
