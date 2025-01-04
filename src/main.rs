use crate::world::World;
use kiss3d::{light::Light, window::Window};

mod block;
mod script;
mod world;

fn main() {
    let mut window = Window::new("PhysBlocks");
    window.set_light(Light::StickToCamera);
    let world = World::load(window.scene_mut(), "default.ron").unwrap();
    window.render_loop(world);
}
