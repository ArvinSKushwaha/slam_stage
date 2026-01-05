use eframe::egui;
use egui_plot::PlotItemBase;
use rayon::prelude::*;
use sim::{Agent2D, Scene2D};
use std::time::Instant;

mod render;

#[derive(Default, Debug, Copy, Clone)]
pub struct TrackRenderState {}

#[derive(Clone)]
pub struct TrackState {
    base: PlotItemBase,
    pub(crate) track_texture: egui::TextureHandle,
    pub(crate) track_render_state: TrackRenderState,
    pub(crate) scene: Scene2D,
}

impl TrackState {
    pub fn new(
        image: &image::DynamicImage,
        threshold: u8,
        track_render_state: TrackRenderState,
        agents: Vec<Agent2D>,
        ctx: &egui::Context,
    ) -> Self {
        let start = Instant::now();

        let image = image.to_luma8();
        let size = [image.width(), image.height()];

        let mut data = image.into_vec();

        data.par_iter_mut().for_each(|p| {
            if *p <= threshold {
                *p = 0
            } else {
                *p = 255;
            }
        });

        log::info!("Image: Width: {}, Height: {}", size[0], size[1],);

        let mut scene = Scene2D::from_pixels([size[0] as _, size[1] as _], &data).unwrap();
        for agent in agents {
            scene.add_agent(agent);
        }

        let color_image = egui::ColorImage::from_rgba_unmultiplied(
            [size[0] as _, size[1] as _],
            &data
                .iter()
                .flat_map(|&i| [i, i, i, if i == 0 { 255 } else { 10 }])
                .collect::<Vec<_>>(),
        );
        let image_data = egui::ImageData::from(color_image);

        let texture_handle = ctx.load_texture(
            "track_texture",
            image_data,
            egui::TextureOptions {
                magnification: egui::TextureFilter::Nearest,
                minification: egui::TextureFilter::Linear,
                wrap_mode: egui::TextureWrapMode::ClampToEdge,
                mipmap_mode: Some(egui::TextureFilter::Nearest),
            },
        );

        log::trace!(
            "Took {} ms to load new texture",
            start.elapsed().as_millis()
        );

        TrackState {
            base: PlotItemBase::new("TrackState".into()),
            track_texture: texture_handle,
            track_render_state,
            scene,
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum TrackLoadError {
    #[error("IOError: {0}")]
    IOError(#[from] std::io::Error),
    #[error("ImageError: {0}")]
    ImageError(#[from] image::ImageError),
    #[error("Deserialize: {0}")]
    DeserializeError(#[from] serde_yml::Error),
}

impl TrackState {
    pub fn load(
        path: impl AsRef<std::path::Path>,
        threshold: u8,
        track_render_state: TrackRenderState,
        agents: Vec<Agent2D>,
        ctx: &egui::Context,
    ) -> Result<Self, TrackLoadError> {
        log::info!(
            "Loading Path: {path:?} with threshold {threshold}",
            path = path.as_ref()
        );

        let start = Instant::now();

        let image = image::ImageReader::open(path.as_ref())?.decode()?;

        log::trace!(
            "Took {} ms to load new image file",
            start.elapsed().as_millis()
        );

        Ok(TrackState::new(
            &image,
            threshold,
            track_render_state,
            agents,
            ctx,
        ))
    }
}
