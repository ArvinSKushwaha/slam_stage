use std::time::Instant;

use kdam::BarExt;
use rand::{distr::slice::Choose, prelude::*};
use rayon::prelude::*;
use sim::{Agent2D, Lidar2D, Scene2D, sensors::Sensor2D};

fn main() -> anyhow::Result<()> {
    let track = image::open("./track1.png")?.to_luma8();
    let scene = Scene2D::from_pixels(
        [track.width() as usize, track.height() as usize],
        track.as_flat_samples().as_slice(),
    )?;

    let unoccupied: Vec<_> = scene
        .occupancy_map
        .pixels
        .par_iter()
        .enumerate()
        .flat_map(|(i, b)| if !b { Some(i) } else { None })
        .collect();

    println!(
        "Collected {} boundary segments.",
        scene.occupancy_map.boundaries.len()
    );

    let mut agent = Agent2D::default();

    let lidar = Lidar2D::regular(6000);
    let mut start = Instant::now();

    let mut rng = rand::rng();

    let mut tqdm = kdam::tqdm!(total = 1000);
    for _ in 0..tqdm.total {
        let agent_config = agent.config;
        let agent_state = agent.state;
        lidar.sense(agent_config, agent_state, scene.state());

        let &location = Choose::new(&unoccupied).unwrap().sample(&mut rng);
        let [width, _] = scene.occupancy_map.size.to_array();
        let random_box = scene.get_box(glam::usizevec2(location % width, location / width));

        let factor: glam::Vec2 = rng.random();

        agent.state.position = random_box.min * factor + (1.0 - factor) * random_box.max;
        agent.state.heading = glam::Vec2::from_angle(rng.random_range(0.0..std::f32::consts::TAU));

        tqdm.update(1).unwrap();
        tqdm.write(format!("Took {:>7} us", start.elapsed().as_micros())).unwrap();

        start = Instant::now();
    }

    Ok(())
}
