//! Own the C model/data on one thread. All MuJoCo FFI and pointer access stay here.
use crate::{
    ffi,
    friction::Stribeck,
    protocol::{Command, Motor},
};
use anyhow::{Context, Result, ensure};
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeMap,
    ffi::{CStr, CString},
    path::Path,
    ptr::NonNull,
    slice,
};

pub const SIDES: [&str; 2] = ["right", "left"];
pub const STEP: f64 = 0.0005;
// Preserve the official gripper driver's numerical conversion exactly.
#[allow(clippy::approx_constant)]
pub const RADIUS: f64 = 0.044 / 1.0472;
pub type Pose = [f64; 8];

#[derive(Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Arms<T> {
    pub right: Option<T>,
    pub left: Option<T>,
}
impl<T> Arms<T> {
    fn entries(self) -> [Option<T>; 2] {
        [self.right, self.left]
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    #[serde(default)]
    pub poses: Arms<Pose>,
    #[serde(default)]
    pub offsets: Arms<Pose>,
    #[serde(default)]
    pub bodies: BTreeMap<String, BodyParameters>,
    #[serde(default)]
    pub joints: BTreeMap<String, JointParameters>,
    #[serde(default = "one")]
    pub friction_scale: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            poses: Arms::default(),
            offsets: Arms::default(),
            bodies: BTreeMap::new(),
            joints: BTreeMap::new(),
            friction_scale: 1.,
        }
    }
}

fn one() -> f64 {
    1.
}

#[derive(Deserialize, Serialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct BodyParameters {
    pub mass: f64,
    pub com: [f64; 3],
    pub inertia: [f64; 3],
}

/// Independent fixture overrides. Explicit values replace scaled defaults.
#[derive(Default, Deserialize, Serialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct JointParameters {
    /// Dry friction magnitude in Nm (arm hinge joints only).
    pub frictionloss: Option<f64>,
    /// Viscous damping in Nm*s/rad.
    pub damping: Option<f64>,
    /// Passive spring stiffness in Nm/rad; useful for model-mismatch tests.
    pub stiffness: Option<f64>,
    /// Passive spring rest angle in radians.
    pub springref: Option<f64>,
    /// Optional speed/angle/direction-dependent dry-friction envelope.
    pub stribeck: Option<Stribeck>,
}

struct VariableFriction {
    qpos: usize,
    dof: usize,
    sliding_nm: f64,
    law: Stribeck,
}

struct Spec(NonNull<ffi::mjSpec>);
impl Drop for Spec {
    fn drop(&mut self) {
        unsafe {
            ffi::mj_deleteSpec(self.0.as_ptr());
        }
    }
}
struct Model(NonNull<ffi::mjModel>);
impl Drop for Model {
    fn drop(&mut self) {
        unsafe {
            ffi::mj_deleteModel(self.0.as_ptr());
        }
    }
}
struct Data(NonNull<ffi::mjData>);
impl Drop for Data {
    fn drop(&mut self) {
        unsafe {
            ffi::mj_deleteData(self.0.as_ptr());
        }
    }
}

#[derive(Clone, Copy, Default)]
struct Index {
    qpos: [usize; 9],
    dof: [usize; 9],
    actuator: [usize; 8],
    ctrl: [usize; 8],
    force: [usize; 8],
}

#[derive(Serialize)]
pub struct Snapshot {
    pub right: [Motor; 8],
    pub left: [Motor; 8],
}

pub struct Physics {
    // Drop data before model. NonNull ownership keeps this !Send / !Sync;
    // no other thread may step/read the mutable MuJoCo world concurrently.
    data: Data,
    model: Model,
    index: [Index; 2],
    offsets: [Pose; 2],
    friction: Vec<VariableFriction>,
    joint_parameters: BTreeMap<String, JointParameters>,
    applied_torque: [[f64; 7]; 2],
    pub motors: [[Motor; 8]; 2],
}

pub fn model_sha256(path: &Path) -> Result<String> {
    use sha2::{Digest, Sha256};
    ensure!(
        unsafe { ffi::mj_version() } == ffi::mjVERSION_HEADER as i32,
        "MuJoCo runtime/header version mismatch"
    );
    let path = CString::new(path.as_os_str().as_encoded_bytes())?;
    let mut error = [0i8; 2048];
    let raw = unsafe {
        ffi::mj_loadXML(
            path.as_ptr(),
            std::ptr::null(),
            error.as_mut_ptr(),
            error.len() as i32,
        )
    };
    let model = Model(NonNull::new(raw).with_context(|| unsafe {
        CStr::from_ptr(error.as_ptr())
            .to_string_lossy()
            .into_owned()
    })?);
    // Compiled model includes referenced XML and meshes, independent of paths.
    let size = unsafe { ffi::mj_sizeModel(model.0.as_ptr()) };
    let mut bytes = vec![0u8; usize::try_from(size)?];
    unsafe {
        ffi::mj_saveModel(
            model.0.as_ptr(),
            std::ptr::null(),
            bytes.as_mut_ptr().cast(),
            i32::try_from(size)?,
        );
    }
    Ok(format!("{:x}", Sha256::digest(&bytes)))
}

impl Physics {
    pub fn load(path: &Path, config: Config) -> Result<Self> {
        // Check the ABI before accessing any generated struct fields.
        ensure!(
            unsafe { ffi::mj_version() } == ffi::mjVERSION_HEADER as i32,
            "MuJoCo runtime/header version mismatch; rebuild with matching MuJoCo headers and library"
        );
        let mut error = [0i8; 2048];
        let path = CString::new(path.as_os_str().as_encoded_bytes())?;
        let raw = unsafe {
            ffi::mj_parseXML(
                path.as_ptr(),
                std::ptr::null(),
                error.as_mut_ptr(),
                error.len() as i32,
            )
        };
        let spec = Spec(NonNull::new(raw).with_context(|| {
            unsafe { CStr::from_ptr(error.as_ptr()) }
                .to_string_lossy()
                .into_owned()
        })?);
        for side in SIDES {
            // Same two overlapping finger pairs as the original simulator.
            unsafe {
                let exclusion = ffi::mjs_addExclude(spec.0.as_ptr());
                ensure!(!exclusion.is_null(), "could not add gripper exclusion");
                ffi::mjs_setString(
                    (*exclusion).bodyname1,
                    CString::new(format!("openarm_{side}_right_finger"))?.as_ptr(),
                );
                ffi::mjs_setString(
                    (*exclusion).bodyname2,
                    CString::new(format!("openarm_{side}_left_finger"))?.as_ptr(),
                );
                let find = |name: &str| -> Result<*mut ffi::mjsActuator> {
                    let element = ffi::mjs_findElement(
                        spec.0.as_ptr(),
                        ffi::mjOBJ_ACTUATOR,
                        CString::new(name)?.as_ptr(),
                    );
                    ensure!(!element.is_null(), "missing actuator {name}");
                    NonNull::new(ffi::mjs_asActuator(element))
                        .map(|a| a.as_ptr())
                        .context("wrong element type")
                };
                let grip = find(&format!("{side}_finger1_ctrl"))?;
                (*grip).trntype = ffi::mjTRN_TENDON;
                ffi::mjs_setString(
                    (*grip).target,
                    CString::new(format!("split_{side}"))?.as_ptr(),
                );
                (*grip).gear[0] = -1. / RADIUS;
                (*grip).forcerange = (*find(&format!("{side}_joint7_ctrl"))?).forcerange;
                ensure!(
                    ffi::mjs_delete(
                        spec.0.as_ptr(),
                        (*find(&format!("{side}_finger2_ctrl"))?).element
                    ) == 0,
                    "could not remove second finger actuator"
                );
            }
        }
        let raw = unsafe { ffi::mj_compile(spec.0.as_ptr(), std::ptr::null()) };
        let model = Model(NonNull::new(raw).with_context(|| {
            unsafe { CStr::from_ptr(ffi::mjs_getError(spec.0.as_ptr())) }
                .to_string_lossy()
                .into_owned()
        })?);
        let mut index = [Index::default(); 2];
        // SAFETY: arrays below belong to the live model and use its declared
        // dimensions. Named IDs and control/output dimensions are checked first.
        unsafe {
            let m = &mut *model.0.as_ptr();
            m.opt.timestep = STEP;
            m.opt.integrator = ffi::mjINT_IMPLICITFAST as i32;
            slice::from_raw_parts_mut(m.jnt_solref, m.njnt as usize * ffi::mjNREF as usize)
                .chunks_mut(ffi::mjNREF as usize)
                .for_each(|a| a.copy_from_slice(&[0.002, 1.]));
            slice::from_raw_parts_mut(m.jnt_solimp, m.njnt as usize * ffi::mjNIMP as usize)
                .chunks_mut(ffi::mjNIMP as usize)
                .for_each(|a| a[..3].copy_from_slice(&[0.99, 0.999, 0.001]));
            slice::from_raw_parts_mut(m.actuator_gaintype, m.nactuator as usize)
                .fill(ffi::mjGAIN_FIXED as i32);
            slice::from_raw_parts_mut(m.actuator_biastype, m.nactuator as usize)
                .fill(ffi::mjBIAS_AFFINE as i32);
            slice::from_raw_parts_mut(
                m.actuator_gainprm,
                m.nactuator as usize * ffi::mjNGAIN as usize,
            )
            .chunks_mut(ffi::mjNGAIN as usize)
            .for_each(|a| {
                a.fill(0.);
                a[0] = 1.;
            });
            slice::from_raw_parts_mut(
                m.actuator_biasprm,
                m.nactuator as usize * ffi::mjNBIAS as usize,
            )
            .fill(0.);
            slice::from_raw_parts_mut(m.actuator_ctrllimited, m.nu as usize).fill(false);
            let id = |kind, name: String| -> Result<usize> {
                let value =
                    ffi::mj_name2id(model.0.as_ptr(), kind, CString::new(name.clone())?.as_ptr());
                ensure!(value >= 0, "missing model element {name}");
                Ok(value as usize)
            };
            for (side, name) in SIDES.iter().enumerate() {
                for joint in 0..9 {
                    let label = if joint < 7 {
                        format!("openarm_{name}_joint{}", joint + 1)
                    } else {
                        format!("openarm_{name}_finger_joint{}", joint - 6)
                    };
                    let j = id(ffi::mjOBJ_JOINT as i32, label)?;
                    ensure!(
                        matches!(
                            *m.jnt_type.add(j) as u32,
                            ffi::mjJNT_HINGE | ffi::mjJNT_SLIDE
                        ),
                        "motor joint must have one scalar degree of freedom"
                    );
                    index[side].qpos[joint] = *m.jnt_qposadr.add(j) as usize;
                    index[side].dof[joint] = *m.jnt_dofadr.add(j) as usize;
                    ensure!(
                        index[side].qpos[joint] < m.nq as usize
                            && index[side].dof[joint] < m.nv as usize,
                        "invalid joint address"
                    );
                }
                for joint in 0..8 {
                    let label = if joint < 7 {
                        format!("{name}_joint{}_ctrl", joint + 1)
                    } else {
                        format!("{name}_finger1_ctrl")
                    };
                    let a = id(ffi::mjOBJ_ACTUATOR as i32, label)?;
                    ensure!(
                        *m.actuator_ctrlnum.add(a) == 1 && *m.actuator_outnum.add(a) == 1,
                        "MIT requires scalar actuator control/force"
                    );
                    index[side].actuator[joint] = a;
                    index[side].ctrl[joint] = *m.actuator_ctrladr.add(a) as usize;
                    index[side].force[joint] = *m.actuator_outadr.add(a) as usize;
                    ensure!(
                        index[side].ctrl[joint] < m.nu as usize
                            && index[side].force[joint] < m.nout as usize,
                        "invalid actuator control/output address"
                    );
                }
            }
        }
        let data = Data(
            NonNull::new(unsafe { ffi::mj_makeData(model.0.as_ptr()) })
                .context("could not allocate MuJoCo data")?,
        );
        let mut friction = Vec::new();
        let mut joint_parameters = BTreeMap::new();
        // Experimental assembly changes live only in this simulator's model.
        // Never edit the upstream XML or the estimator's nominal description.
        unsafe {
            let m = &mut *model.0.as_ptr();
            ensure!(
                config.friction_scale.is_finite() && config.friction_scale >= 0.,
                "invalid friction scale"
            );
            for (name, body) in &config.bodies {
                let id = ffi::mj_name2id(
                    model.0.as_ptr(),
                    ffi::mjOBJ_BODY as i32,
                    CString::new(name.as_str())?.as_ptr(),
                );
                ensure!(
                    id > 0
                        && (name.starts_with("openarm_right_")
                            || name.starts_with("openarm_left_")),
                    "invalid perturbed arm body {name}"
                );
                ensure!(
                    body.mass.is_finite()
                        && body.mass > 0.
                        && body.com.iter().all(|x| x.is_finite())
                        && body.inertia.iter().all(|x| x.is_finite() && *x > 0.),
                    "invalid inertial parameters for {name}"
                );
                let sum: f64 = body.inertia.iter().sum();
                ensure!(
                    body.inertia.iter().all(|x| 2. * x <= sum + 1e-12),
                    "inertia triangle inequality for {name}"
                );
                let id = id as usize;
                *m.body_mass.add(id) = body.mass;
                slice::from_raw_parts_mut(m.body_ipos.add(3 * id), 3).copy_from_slice(&body.com);
                slice::from_raw_parts_mut(m.body_inertia.add(3 * id), 3)
                    .copy_from_slice(&body.inertia);
            }
            for ix in &index {
                for dof in &ix.dof[..7] {
                    *m.dof_frictionloss.add(*dof) *= config.friction_scale;
                    *m.dof_damping.add(*dof) *= config.friction_scale;
                }
            }
            for (name, parameters) in &config.joints {
                let id = ffi::mj_name2id(
                    model.0.as_ptr(),
                    ffi::mjOBJ_JOINT as i32,
                    CString::new(name.as_str())?.as_ptr(),
                );
                ensure!(id >= 0, "missing perturbed joint {name}");
                let joint = id as usize;
                let dof = *m.jnt_dofadr.add(joint) as usize;
                let qpos = *m.jnt_qposadr.add(joint) as usize;
                ensure!(
                    *m.jnt_type.add(joint) as u32 == ffi::mjJNT_HINGE
                        && index.iter().any(|ix| ix.dof[..7].contains(&dof)),
                    "only arm hinge joints can be perturbed: {name}"
                );
                for (value, field, address) in [
                    (
                        parameters.frictionloss,
                        "frictionloss",
                        m.dof_frictionloss.add(dof),
                    ),
                    (parameters.damping, "damping", m.dof_damping.add(dof)),
                    (
                        parameters.stiffness,
                        "stiffness",
                        m.jnt_stiffness.add(joint),
                    ),
                ] {
                    if let Some(value) = value {
                        ensure!(
                            value.is_finite() && value >= 0.,
                            "invalid {field} for {name}"
                        );
                        *address = value;
                    }
                }
                if let Some(value) = parameters.springref {
                    ensure!(value.is_finite(), "invalid springref for {name}");
                    *m.qpos_spring.add(qpos) = value;
                }
            }
            for (side, ix) in index.iter().enumerate() {
                for j in 0..7 {
                    let name = format!("openarm_{}_joint{}", SIDES[side], j + 1);
                    let id = ffi::mj_name2id(
                        model.0.as_ptr(),
                        ffi::mjOBJ_JOINT as i32,
                        CString::new(name.as_str())?.as_ptr(),
                    ) as usize;
                    let sliding_nm = *m.dof_frictionloss.add(ix.dof[j]);
                    let damping = *m.dof_damping.add(ix.dof[j]);
                    ensure!(
                        sliding_nm.is_finite() && damping.is_finite(),
                        "friction scaling overflow for {name}"
                    );
                    let law = config.joints.get(&name).and_then(|p| p.stribeck.clone());
                    if let Some(law) = &law {
                        law.validate(sliding_nm)
                            .with_context(|| format!("invalid friction for {name}"))?;
                        // Default soft friction can creep across the Stribeck
                        // band under a sub-breakaway load on small wrist inertia.
                        // Enhanced joints use a firmer, still regularized native
                        // constraint. Basic joints keep their upstream settings.
                        slice::from_raw_parts_mut(
                            m.dof_solref.add(ix.dof[j] * ffi::mjNREF as usize),
                            ffi::mjNREF as usize,
                        )
                        .copy_from_slice(&[0.002, 1.]);
                        slice::from_raw_parts_mut(
                            m.dof_solimp.add(ix.dof[j] * ffi::mjNIMP as usize),
                            ffi::mjNIMP as usize,
                        )
                        .copy_from_slice(&[0.999, 0.999, 0.001, 0.5, 2.]);
                        friction.push(VariableFriction {
                            qpos: ix.qpos[j],
                            dof: ix.dof[j],
                            sliding_nm,
                            law: law.clone(),
                        });
                    }
                    joint_parameters.insert(
                        name,
                        JointParameters {
                            frictionloss: Some(sliding_nm),
                            damping: Some(damping),
                            stiffness: Some(*m.jnt_stiffness.add(id)),
                            springref: Some(*m.qpos_spring.add(ix.qpos[j])),
                            stribeck: law,
                        },
                    );
                }
            }
            ffi::mj_setConst(model.0.as_ptr(), data.0.as_ptr());
        }
        let offsets = config.offsets.entries().map(|a| a.unwrap_or([0.; 8]));
        let mut world = Self {
            data,
            model,
            index,
            offsets,
            friction,
            joint_parameters,
            applied_torque: [[0.; 7]; 2],
            motors: std::array::from_fn(|_| std::array::from_fn(|i| Motor::new(i + 1))),
        };
        world.reset(config.poses)?;
        Ok(world)
    }

    pub fn reset(&mut self, poses: Arms<Pose>) -> Result<()> {
        let mut zero = [0.; 8];
        zero[7] = -10f64.to_radians();
        let poses = poses.entries().map(|a| a.unwrap_or(zero));
        ensure!(
            poses
                .iter()
                .chain(&self.offsets)
                .flatten()
                .all(|v| v.is_finite()),
            "poses/offsets must be finite"
        );
        unsafe {
            ffi::mj_resetData(self.model.0.as_ptr(), self.data.0.as_ptr());
            let d = self.data.0.as_mut();
            for (side, pose) in poses.iter().enumerate() {
                for (j, q) in pose[..7].iter().enumerate() {
                    *d.qpos.add(self.index[side].qpos[j]) = *q;
                }
                for j in 7..9 {
                    *d.qpos.add(self.index[side].qpos[j]) = -pose[7] * RADIUS;
                }
                self.motors[side] = std::array::from_fn(|i| Motor::new(i + 1));
            }
        }
        self.applied_torque = [[0.; 7]; 2];
        self.configure();
        self.configure_friction();
        unsafe {
            ffi::mj_forward(self.model.0.as_ptr(), self.data.0.as_ptr());
        }
        self.feedback();
        Ok(())
    }

    fn configure(&mut self) {
        unsafe {
            let (m, d) = (self.model.0.as_mut(), self.data.0.as_mut());
            for side in 0..2 {
                for j in 0..8 {
                    let motor = self.motors[side][j];
                    let a = self.index[side].actuator[j];
                    let c = if motor.status == 1 {
                        motor.command
                    } else {
                        Command::default()
                    };
                    *d.ctrl.add(self.index[side].ctrl[j]) =
                        c.kp * (c.q - self.offsets[side][j]) + c.kd * c.dq + c.tau;
                    *m.actuator_biasprm.add(a * ffi::mjNBIAS as usize + 1) = -c.kp;
                    *m.actuator_biasprm.add(a * ffi::mjNBIAS as usize + 2) = -c.kd;
                }
            }
        }
    }

    fn configure_friction(&mut self) {
        // Freeze a positive friction bound over this implicit step. MuJoCo's
        // native constraint chooses the opposing force and handles stiction;
        // no explicit sign(v) torque, hidden integrator or bristle state is added.
        unsafe {
            let (m, d) = (self.model.0.as_mut(), self.data.0.as_ref());
            for joint in &self.friction {
                *m.dof_frictionloss.add(joint.dof) = joint.law.bound_nm(
                    joint.sliding_nm,
                    *d.qpos.add(joint.qpos),
                    *d.qvel.add(joint.dof),
                );
            }
        }
    }

    pub fn step(&mut self, count: u64) -> Result<()> {
        self.configure();
        unsafe {
            for _ in 0..count {
                self.configure_friction();
                ffi::mj_step(self.model.0.as_ptr(), self.data.0.as_ptr());
            }
            let d = self.data.0.as_ref();
            ensure!(
                d.warning.iter().all(|w| w.number == 0)
                    && slice::from_raw_parts(d.qpos, self.model.0.as_ref().nq as usize)
                        .iter()
                        .all(|q| q.is_finite()),
                "MuJoCo numerical warning; simulation stopped"
            );
        }
        self.feedback();
        Ok(())
    }

    fn feedback(&mut self) {
        unsafe {
            let d = self.data.0.as_ref();
            for side in 0..2 {
                for j in 0..8 {
                    let ix = self.index[side];
                    let (q, dq) = if j < 7 {
                        (*d.qpos.add(ix.qpos[j]), *d.qvel.add(ix.dof[j]))
                    } else {
                        (
                            -(*d.qpos.add(ix.qpos[7]) + *d.qpos.add(ix.qpos[8])) / (2. * RADIUS),
                            -(*d.qvel.add(ix.dof[7]) + *d.qvel.add(ix.dof[8])) / (2. * RADIUS),
                        )
                    };
                    let motor = &mut self.motors[side][j];
                    motor.q = q + self.offsets[side][j];
                    motor.dq = dq;
                    motor.torque = if motor.status == 1 {
                        *d.actuator_force.add(ix.force[j])
                    } else {
                        0.
                    };
                }
            }
        }
    }

    pub fn time(&self) -> f64 {
        unsafe { self.data.0.as_ref().time }
    }
    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            right: self.motors[0],
            left: self.motors[1],
        }
    }
    pub fn version() -> String {
        unsafe { CStr::from_ptr(ffi::mj_versionString()) }
            .to_string_lossy()
            .into_owned()
    }

    pub fn push(&mut self, forces: [[f64; 7]; 2]) -> Result<()> {
        ensure!(
            forces.iter().flatten().all(|v| v.is_finite()),
            "invalid applied torque"
        );
        unsafe {
            let d = self.data.0.as_mut();
            for (side, arm) in forces.iter().enumerate() {
                for (j, force) in arm.iter().enumerate() {
                    *d.qfrc_applied.add(self.index[side].dof[j]) = *force;
                }
            }
        }
        self.applied_torque = forces;
        Ok(())
    }

    pub fn parameters(&self) -> serde_json::Value {
        serde_json::json!({
            "friction_model": "mujoco-dry-viscous-with-optional-stribeck-v1",
            "joints": self.joint_parameters,
            "applied_torque_nm": {"right": self.applied_torque[0], "left": self.applied_torque[1]},
        })
    }

    pub fn configuration(&self) -> serde_json::Value {
        // Fixed plant identity excludes pose, elapsed time and transient pushes.
        let m = unsafe { self.model.0.as_ref() };
        serde_json::json!({
            "friction_model": "mujoco-dry-viscous-with-optional-stribeck-v1",
            "mujoco_version": Self::version(), "timestep_s": m.opt.timestep,
            "integrator": "implicitfast", "gravity_m_s2": m.opt.gravity,
            "joint_stop_solref": [0.002, 1.], "joint_stop_solimp": [0.99,0.999,0.001,0.5,2.],
            "enhanced_friction_solref": [0.002,1.],
            "enhanced_friction_solimp": [0.999,0.999,0.001,0.5,2.],
            "gripper_radius_m": RADIUS,
            "joints": self.joint_parameters, "bodies": self.body_parameters(),
            "encoder_offsets_rad": {"right": self.offsets[0], "left": self.offsets[1]},
        })
    }

    fn body_parameters(&self) -> BTreeMap<String, BodyParameters> {
        unsafe {
            let m = self.model.0.as_ref();
            (1..m.nbody as usize)
                .filter_map(|i| {
                    let name = CStr::from_ptr(ffi::mj_id2name(
                        self.model.0.as_ptr(),
                        ffi::mjOBJ_BODY as i32,
                        i as i32,
                    ))
                    .to_string_lossy();
                    if !name.starts_with("openarm_right_") && !name.starts_with("openarm_left_") {
                        return None;
                    }
                    Some((
                        name.into_owned(),
                        BodyParameters {
                            mass: *m.body_mass.add(i),
                            com: slice::from_raw_parts(m.body_ipos.add(3 * i), 3)
                                .try_into()
                                .unwrap(),
                            inertia: slice::from_raw_parts(m.body_inertia.add(3 * i), 3)
                                .try_into()
                                .unwrap(),
                        },
                    ))
                })
                .collect()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn world(config: Config) -> Physics {
        Physics::load(std::path::Path::new(openarm_test_model::SCENE), config).unwrap()
    }

    #[test]
    fn gravity_and_holding() {
        let mut pose = [0.; 8];
        pose[3] = 45f64.to_radians();
        pose[7] = -0.2;
        let mut p = world(Config {
            poses: Arms {
                right: Some(pose),
                left: Some(pose),
            },
            ..Config::default()
        });
        p.step(200).unwrap();
        assert!((p.motors[0][3].q - pose[3]).abs() > 0.001);
        p.reset(Arms {
            right: Some(pose),
            left: Some(pose),
        })
        .unwrap();
        for arm in &mut p.motors {
            for m in arm {
                m.status = 1;
                m.command = Command {
                    kp: 150.,
                    kd: 2.,
                    q: m.q,
                    ..Command::default()
                };
            }
        }
        p.step(1000).unwrap();
        assert!((p.motors[0][3].q - pose[3]).abs() < 0.08);
        assert!(p.motors[0][3].torque.abs() > 0.1);
    }

    #[test]
    fn gripper_transmission_loaded_stop_and_no_teleport() {
        let mut p = world(Config::default());
        p.motors[0][7].status = 1;
        p.motors[0][7].command = Command {
            kp: 10.,
            kd: 0.9,
            q: -0.5,
            ..Command::default()
        };
        p.step(1).unwrap();
        assert!((p.motors[0][7].q + 0.5).abs() > 0.1);
        p.step(1000).unwrap();
        assert!((p.motors[0][7].q + 0.5).abs() < 0.01);
        unsafe {
            for i in &p.index[0].qpos[7..] {
                assert!((*p.data.0.as_ref().qpos.add(*i) - 0.5 * RADIUS).abs() < 0.001);
            }
        }
        p.motors[0][7].command = Command {
            kp: 45.,
            kd: 1.2,
            q: 0.1,
            ..Command::default()
        };
        p.step(2000).unwrap();
        let m = p.motors[0][7];
        assert!(m.dq.abs() < 0.01 && m.torque > 0.3 && m.q > 0. && m.q < 0.2f64.to_radians());
    }

    #[test]
    fn joint_target_is_physics_driven_and_force_is_bounded() {
        let mut p = world(Config::default());
        p.motors[0][6].status = 1;
        p.motors[0][6].command = Command {
            kp: 10.,
            kd: 0.5,
            q: 0.3,
            ..Command::default()
        };
        p.step(1).unwrap();
        assert!(p.motors[0][6].q < 0.1);
        p.step(1000).unwrap();
        assert!((p.motors[0][6].q - 0.3).abs() < 0.05);
        assert!(p.motors[1][6].q.abs() < 0.05);
        p.motors[0][6].command.q = 10.;
        p.step(1).unwrap();
        assert!((p.motors[0][6].torque - 7.).abs() < 1e-6);
        p.motors[0][6].status = 0;
        p.step(1).unwrap();
        assert_eq!(p.motors[0][6].torque, 0.);
    }

    #[test]
    fn encoder_bias_and_reset() {
        let mut p = world(Config {
            offsets: Arms {
                right: Some([0.01; 8]),
                left: None,
            },
            ..Config::default()
        });
        assert_eq!(p.motors[0][6].q, 0.01);
        p.motors[0][6].status = 1;
        p.motors[0][6].command = Command {
            kp: 30.,
            kd: 0.8,
            q: 0.3,
            ..Command::default()
        };
        p.step(1000).unwrap();
        assert!((p.motors[0][6].q - 0.3).abs() < 0.02);
        p.motors[0][6].silent = true;
        p.reset(Arms::default()).unwrap();
        assert!(
            !p.motors[0][6].silent && p.motors[0][6].status == 0 && p.motors[0][6].command.kp == 0.
        );
    }

    #[test]
    fn per_joint_passive_parameters_override_scaling_and_survive_reset() {
        let parameters = JointParameters {
            frictionloss: Some(0.23),
            damping: Some(0.61),
            stiffness: Some(0.47),
            springref: Some(0.19),
            ..JointParameters::default()
        };
        let mut p = world(Config {
            friction_scale: 0.5,
            joints: BTreeMap::from([("openarm_right_joint4".into(), parameters)]),
            ..Config::default()
        });
        p.reset(Arms::default()).unwrap();
        unsafe {
            let (m, d) = (p.model.0.as_ref(), p.data.0.as_ref());
            let dof = p.index[0].dof[3];
            assert_eq!(*m.dof_frictionloss.add(dof), 0.23);
            assert_eq!(*m.dof_damping.add(dof), 0.61);
            assert!((*d.qfrc_passive.add(dof) - 0.47 * 0.19).abs() < 1e-12);
            assert!((*m.dof_frictionloss.add(p.index[1].dof[3]) - 0.05).abs() < 1e-12);
        }
    }

    #[test]
    fn reject_invalid_joint_parameters() {
        let path = std::path::Path::new(openarm_test_model::SCENE);
        for (name, value) in [
            ("openarm_right_joint4", -0.1),
            ("openarm_right_finger_joint1", 0.1),
        ] {
            let config = Config {
                joints: BTreeMap::from([(
                    name.into(),
                    JointParameters {
                        frictionloss: Some(value),
                        ..JointParameters::default()
                    },
                )]),
                ..Config::default()
            };
            assert!(Physics::load(path, config).is_err());
        }
    }

    fn friction_world(law: Option<Stribeck>, step: f64) -> Physics {
        let mut p = world(Config {
            friction_scale: 0.,
            joints: BTreeMap::from([(
                "openarm_right_joint7".into(),
                JointParameters {
                    frictionloss: Some(0.2),
                    damping: Some(0.05),
                    stribeck: law,
                    ..JointParameters::default()
                },
            )]),
            ..Config::default()
        });
        // Isolate passive dissipation from gravity, contacts and joint stops.
        // This remains the full coupled robot mass matrix, not a signal delay.
        unsafe {
            let m = p.model.0.as_mut();
            m.opt.gravity.fill(0.);
            m.opt.timestep = step;
            m.opt.disableflags |= (ffi::mjDSBL_CONTACT | ffi::mjDSBL_LIMIT) as i32;
            ffi::mj_forward(p.model.0.as_ptr(), p.data.0.as_ptr());
        }
        p
    }

    fn test_law() -> Stribeck {
        Stribeck {
            breakaway_nm: 0.6,
            velocity_rad_s: 0.04,
            direction_asymmetry: 0.,
            angle: None,
        }
    }

    #[test]
    fn enhanced_friction_opposes_slip_and_dissipates_energy_in_coupled_physics() {
        for sign in [-1., 1.] {
            let law = Stribeck {
                direction_asymmetry: 0.25,
                angle: Some(crate::friction::AngleModulation {
                    amplitude: 0.3,
                    harmonic: 2,
                    phase_rad: 0.4,
                }),
                ..test_law()
            };
            let mut p = friction_world(Some(law), STEP);
            let dof = p.index[0].dof[6];
            unsafe {
                *p.data.0.as_mut().qvel.add(dof) = sign * 2.;
                ffi::mj_forward(p.model.0.as_ptr(), p.data.0.as_ptr());
                ffi::mj_energyVel(p.model.0.as_ptr(), p.data.0.as_ptr());
            }
            let initial_energy = unsafe { p.data.0.as_ref().energy[1] };
            let mut friction_work = 0.;
            for _ in 0..1000 {
                let before = p.motors[0][6].q;
                p.step(1).unwrap();
                unsafe {
                    let d = p.data.0.as_ref();
                    let force = *d.qfrc_constraint.add(dof);
                    friction_work += force * (p.motors[0][6].q - before);
                    assert!(force.abs() <= *p.model.0.as_ref().dof_frictionloss.add(dof) + 1e-10);
                    ffi::mj_energyVel(p.model.0.as_ptr(), p.data.0.as_ptr());
                    assert!(p.data.0.as_ref().energy[1] <= initial_energy * 1.0001);
                }
            }
            // Other unresisted joints can retain energy transferred through the
            // coupled mass matrix; this test asserts dissipation, not all-arm rest.
            assert!(friction_work < 0.);
            assert!(unsafe { p.data.0.as_ref().energy[1] } < initial_energy);
        }
    }

    #[test]
    fn breakaway_resists_small_push_and_batched_steps_are_deterministic() {
        let mut basic = friction_world(None, STEP);
        let mut rich = friction_world(Some(test_law()), STEP);
        let mut repeated = friction_world(Some(test_law()), STEP);
        let mut force = [[0.; 7]; 2];
        force[0][6] = 0.3; // Above sliding friction, below enhanced breakaway.
        for p in [&mut basic, &mut rich, &mut repeated] {
            p.push(force).unwrap();
        }
        basic.step(400).unwrap();
        rich.step(400).unwrap();
        for _ in 0..400 {
            repeated.step(1).unwrap();
        }
        assert!(
            basic.motors[0][6].q > 5. * rich.motors[0][6].q,
            "basic={} enhanced={}",
            basic.motors[0][6].q,
            rich.motors[0][6].q
        );
        assert_eq!(rich.motors[0][6].q, repeated.motors[0][6].q);
        assert_eq!(rich.motors[0][6].dq, repeated.motors[0][6].dq);
        assert_eq!(rich.motors[0][6].torque, 0.); // External push is not motor torque.
        force[0][6] = 0.9;
        rich.push(force).unwrap();
        let before = rich.motors[0][6].q;
        rich.step(200).unwrap();
        assert!(rich.motors[0][6].q - before > 0.01);
    }

    #[test]
    fn enhanced_friction_is_stable_under_time_step_variation() {
        let mut endpoints = Vec::new();
        for step in [0.00025, STEP, 0.001] {
            let mut p = friction_world(
                Some(Stribeck {
                    direction_asymmetry: -0.2,
                    angle: Some(crate::friction::AngleModulation {
                        amplitude: 0.25,
                        harmonic: 2,
                        phase_rad: -0.3,
                    }),
                    ..test_law()
                }),
                step,
            );
            for torque in [0.2, 0.9, 0., -0.9, 0.] {
                let mut force = [[0.; 7]; 2];
                force[0][6] = torque;
                p.push(force).unwrap();
                p.step((0.1 / step).round() as u64).unwrap();
            }
            endpoints.push(p.motors[0][6].q);
            assert!(p.motors[0][6].dq.is_finite());
        }
        for q in &endpoints[1..] {
            // Severe square torque pulses and stick/slip are first-order here;
            // require agreement within 5% over the fourfold step-size range.
            assert!(
                (q - endpoints[0]).abs() < 0.05 * endpoints[0].abs(),
                "step-sensitive endpoints {endpoints:?}"
            );
        }
    }

    #[test]
    fn reset_retains_resistance_but_clears_external_push_atomically() {
        let mut p = friction_world(Some(test_law()), STEP);
        let mut force = [[0.; 7]; 2];
        force[0][6] = 0.75;
        p.push(force).unwrap();
        let previous = p.parameters();
        force[1][0] = f64::NAN;
        assert!(p.push(force).is_err());
        assert_eq!(p.parameters(), previous);
        p.step(100).unwrap();
        p.reset(Arms::default()).unwrap();
        assert_eq!(p.applied_torque, [[0.; 7]; 2]);
        assert_eq!(p.parameters()["joints"], previous["joints"]);
        unsafe {
            assert_eq!(*p.data.0.as_ref().qfrc_applied.add(p.index[0].dof[6]), 0.);
            assert_eq!(
                *p.model.0.as_ref().dof_frictionloss.add(p.index[0].dof[6]),
                0.6
            );
        }
    }

    #[test]
    fn arms_share_contact_world() {
        let right = [-26.625f64, 1.814, -56.701, 121.279, 0., 0., 0., -10.].map(f64::to_radians);
        let left = [-69.152f64, -4.847, -72.3, 115.939, 0., 0., 0., -10.].map(f64::to_radians);
        let p = world(Config {
            poses: Arms {
                right: Some(right),
                left: Some(left),
            },
            ..Config::default()
        });
        let hit = unsafe {
            slice::from_raw_parts(p.data.0.as_ref().contact, p.data.0.as_ref().ncon as usize)
                .iter()
                .any(|c| {
                    let a = CStr::from_ptr(ffi::mj_id2name(
                        p.model.0.as_ptr(),
                        ffi::mjOBJ_GEOM as i32,
                        c.geom[0],
                    ))
                    .to_string_lossy();
                    let b = CStr::from_ptr(ffi::mj_id2name(
                        p.model.0.as_ptr(),
                        ffi::mjOBJ_GEOM as i32,
                        c.geom[1],
                    ))
                    .to_string_lossy();
                    c.dist < -0.001
                        && c.efc_address >= 0
                        && ((a.contains("openarm_left") && b.contains("openarm_right"))
                            || (a.contains("openarm_right") && b.contains("openarm_left")))
                })
        };
        assert!(hit);
    }
}
