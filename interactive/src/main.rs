mod app;
mod track_state;
mod track_file;

use eframe::run_native;

use crate::app::App;

pub fn main() -> anyhow::Result<()> {
    env_logger::init();

    if let Err(e) = run_native(
        "SceneSim Interactive",
        eframe::NativeOptions::default(),
        Box::new(|cc| Ok(Box::new(App::new(cc)?))),
    ) {
        anyhow::bail!("Error in `run_native`: {}", e);
    }

    Ok(())
}
