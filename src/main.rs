use kiss3d::{
    camera::{ArcBall, Camera},
    light::Light,
    planar_camera::PlanarCamera,
    post_processing::PostProcessingEffect,
    scene::SceneNode,
    window::{State, Window},
};
use nalgebra::{Isometry3, Point3, Vector3};
use rapier3d::prelude::{
    BroadPhase, CCDSolver, ColliderBuilder, ColliderSet, ImpulseJointSet, IntegrationParameters,
    IslandManager, MultibodyJointSet, NarrowPhase, PhysicsPipeline, RigidBodyBuilder,
    RigidBodyHandle, RigidBodySet, RigidBodyType,
};

struct Block {
    body: RigidBodyHandle,
    node: SceneNode,
}

impl Block {
    fn new(
        world: &mut World,
        color: Vector3<f32>,
        position: Isometry3<f32>,
        size: Vector3<f32>,
        fixed: bool,
    ) -> Self {
        let body = RigidBodyBuilder::new(if fixed {
            RigidBodyType::Static
        } else {
            RigidBodyType::Dynamic
        })
        .position(position)
        .build();
        let body_handle = world.rigid_body_set.insert(body);
        let collider = ColliderBuilder::cuboid(size.x / 2.0, size.y / 2.0, size.z / 2.0).build();
        world
            .collider_set
            .insert_with_parent(collider, body_handle, &mut world.rigid_body_set);
        let mut node = world.scene.add_cube(size.x, size.y, size.z);
        node.set_color(color.x, color.y, color.z);
        node.set_local_transformation(position);
        Self {
            body: body_handle,
            node,
        }
    }

    fn update(&mut self, rigid_body_set: &RigidBodySet) {
        if let Some(body) = rigid_body_set.get(self.body) {
            self.node.set_local_transformation(*body.position());
        } else {
            eprintln!("Stale RigidBodyHandle!");
        }
    }
}

struct World {
    blocks: Vec<Block>,
    broad_phase: BroadPhase,
    camera: ArcBall,
    ccd_solver: CCDSolver,
    collider_set: ColliderSet,
    gravity: Vector3<f32>,
    impulse_joint_set: ImpulseJointSet,
    integration_parameters: IntegrationParameters,
    island_manager: IslandManager,
    multibody_joint_set: MultibodyJointSet,
    narrow_phase: NarrowPhase,
    physics_pipeline: PhysicsPipeline,
    rigid_body_set: RigidBodySet,
    scene: SceneNode,
}

impl World {
    fn add_block(
        &mut self,
        color: Vector3<f32>,
        position: Isometry3<f32>,
        size: Vector3<f32>,
        fixed: bool,
    ) {
        let block = Block::new(self, color, position, size, fixed);
        self.blocks.push(block);
    }

    fn new(root: &mut SceneNode) -> Self {
        let scene = SceneNode::new_empty();
        root.add_child(scene.clone());
        Self {
            blocks: Vec::new(),
            broad_phase: BroadPhase::default(),
            camera: ArcBall::new(Point3::new(64.0, 16.0, 0.0), Point3::origin()),
            ccd_solver: CCDSolver::new(),
            collider_set: ColliderSet::new(),
            gravity: Vector3::new(0.0, -9.81, 0.0),
            impulse_joint_set: ImpulseJointSet::new(),
            integration_parameters: IntegrationParameters::default(),
            island_manager: IslandManager::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            narrow_phase: NarrowPhase::new(),
            physics_pipeline: PhysicsPipeline::new(),
            rigid_body_set: RigidBodySet::new(),
            scene,
        }
    }
}

impl State for World {
    fn cameras_and_effect(
        &mut self,
    ) -> (
        Option<&mut dyn Camera>,
        Option<&mut dyn PlanarCamera>,
        Option<&mut dyn PostProcessingEffect>,
    ) {
        (Some(&mut self.camera), None, None)
    }

    fn step(&mut self, _window: &mut Window) {
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            &(),
            &(),
        );
        for block in &mut self.blocks {
            block.update(&self.rigid_body_set);
        }
    }
}

fn main() {
    let mut window = Window::new("PhysBlocks");
    let mut world = World::new(window.scene_mut());
    window.set_light(Light::StickToCamera);
    world.add_block(
        Vector3::new(0.0, 1.0, 0.0),
        Isometry3::new(Vector3::new(0.0, 0.0, 0.0), Vector3::zeros()),
        Vector3::new(64.0, 0.5, 64.0),
        true,
    );
    world.add_block(
        Vector3::new(0.5, 0.5, 0.5),
        Isometry3::new(Vector3::new(0.0, 1.0, 0.0), Vector3::zeros()),
        Vector3::new(2.0, 1.0, 4.0),
        false,
    );
    world.add_block(
        Vector3::new(0.5, 0.5, 0.5),
        Isometry3::new(Vector3::new(0.0, 1.0, 4.0), Vector3::zeros()),
        Vector3::new(2.0, 1.0, 4.0),
        false,
    );
    world.add_block(
        Vector3::new(0.5, 0.5, 0.5),
        Isometry3::new(Vector3::new(0.0, 2.0, 2.0), Vector3::zeros()),
        Vector3::new(2.0, 1.0, 4.0),
        false,
    );
    world.add_block(
        Vector3::new(0.5, 0.5, 0.5),
        Isometry3::new(Vector3::new(0.0, 64.0, 3.0), Vector3::zeros()),
        Vector3::new(2.0, 1.0, 4.0),
        false,
    );
    window.render_loop(world);
}
