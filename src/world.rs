use crate::block::{Block, BlockHandle};
use kiss3d::{
    camera::{ArcBall, Camera},
    planar_camera::PlanarCamera,
    post_processing::PostProcessingEffect,
    scene::SceneNode,
    window::{State, Window},
};
use nalgebra::Vector3;
use rapier3d::prelude::{
    BroadPhase, CCDSolver, ColliderBuilder, ColliderSet, DefaultBroadPhase, ImpulseJointSet,
    IntegrationParameters, IslandManager, MultibodyJointSet, NarrowPhase, PhysicsPipeline,
    QueryPipeline, RigidBodyBuilder, RigidBodySet, RigidBodyType,
};
use serde::{Deserialize, Serialize};
use std::{error::Error, fs::File, io::Read, path::Path};

pub struct World {
    blocks: Vec<BlockHandle>,
    broad_phase: Box<dyn BroadPhase>,
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
    query_pipeline: QueryPipeline,
    scene: SceneNode,
}

impl World {
    pub fn add_block(&mut self, block: &Block) -> BlockHandle {
        let body = RigidBodyBuilder::new(if block.fixed {
            RigidBodyType::Fixed
        } else {
            RigidBodyType::Dynamic
        })
        .position(block.position)
        .build();
        let body_handle = self.rigid_body_set.insert(body);
        let collider =
            ColliderBuilder::cuboid(block.size.x / 2.0, block.size.y / 2.0, block.size.z / 2.0)
                .build();
        self.collider_set
            .insert_with_parent(collider, body_handle, &mut self.rigid_body_set);
        let mut node = self
            .scene
            .add_cube(block.size.x, block.size.y, block.size.z);
        node.set_color(block.color.x, block.color.y, block.color.z);
        node.set_local_transformation(BlockHandle::kiss_isometry(block.position));
        let handle = BlockHandle {
            body: body_handle,
            node,
        };
        self.blocks.push(handle.clone());
        handle
    }

    pub fn load<P: AsRef<Path>>(root: &mut SceneNode, path: P) -> Result<Self, Box<dyn Error>> {
        let mut file = File::open(path)?;
        let mut buffer = String::new();
        file.read_to_string(&mut buffer)?;
        let archive: WorldArchive = ron::from_str(&buffer)?;
        let mut world = Self::new(root);
        for block in archive.blocks {
            world.add_block(&block);
        }
        Ok(world)
    }

    pub fn new(root: &mut SceneNode) -> Self {
        let scene = SceneNode::new_empty();
        root.add_child(scene.clone());
        Self {
            blocks: Vec::new(),
            broad_phase: Box::new(DefaultBroadPhase::new()),
            camera: ArcBall::new(
                kiss3d::nalgebra::Point3::new(64.0, 16.0, 0.0),
                kiss3d::nalgebra::Point3::origin(),
            ),
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
            query_pipeline: QueryPipeline::new(),
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
            self.broad_phase.as_mut(),
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &(),
            &(),
        );
        for block in &mut self.blocks {
            block.update(&self.rigid_body_set);
        }
    }
}

#[derive(Deserialize, Serialize)]
struct WorldArchive {
    blocks: Vec<Block>,
}
