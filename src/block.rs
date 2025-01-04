use kiss3d::scene::SceneNode;
use nalgebra::{Isometry3, Vector3};
use rapier3d::prelude::{RigidBodyHandle, RigidBodySet};
use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize)]
pub struct Block {
    #[serde(default = "Block::default_color")]
    pub color: Vector3<f32>,
    #[serde(default = "Block::default_fixed")]
    pub fixed: bool,
    #[serde(default = "Block::default_position")]
    pub position: Isometry3<f32>,
    #[serde(default = "Block::default_size")]
    pub size: Vector3<f32>,
}

impl Block {
    fn default_color() -> Vector3<f32> {
        Vector3::new(0.5, 0.5, 0.5)
    }

    fn default_fixed() -> bool {
        false
    }

    fn default_position() -> Isometry3<f32> {
        Isometry3::new(Vector3::zeros(), Vector3::zeros())
    }

    fn default_size() -> Vector3<f32> {
        Vector3::new(2.0, 1.0, 4.0)
    }
}

impl Default for Block {
    fn default() -> Self {
        Self {
            color: Self::default_color(),
            fixed: Self::default_fixed(),
            position: Self::default_position(),
            size: Self::default_size(),
        }
    }
}

#[derive(Clone)]
pub struct BlockHandle {
    pub(crate) body: RigidBodyHandle,
    pub(crate) node: SceneNode,
}

impl BlockHandle {
    pub(crate) fn kiss_isometry(isometry: Isometry3<f32>) -> kiss3d::nalgebra::Isometry3<f32> {
        // Cursed type interoperability between two versions of the same crate.
        // Send help. ~ahill
        kiss3d::nalgebra::Isometry3 {
            translation: kiss3d::nalgebra::Translation3::new(
                isometry.translation.x,
                isometry.translation.y,
                isometry.translation.z,
            ),
            rotation: kiss3d::nalgebra::Unit::new_normalize(kiss3d::nalgebra::Quaternion {
                coords: kiss3d::nalgebra::Vector4::new(
                    isometry.rotation.coords.x,
                    isometry.rotation.coords.y,
                    isometry.rotation.coords.z,
                    isometry.rotation.coords.w,
                ),
            }),
        }
    }

    pub fn update(&mut self, rigid_body_set: &RigidBodySet) {
        if let Some(body) = rigid_body_set.get(self.body) {
            self.node
                .set_local_transformation(Self::kiss_isometry(*body.position()));
        } else {
            eprintln!("Stale RigidBodyHandle!");
        }
    }
}
