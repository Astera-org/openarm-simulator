//! MuJoCo model editing, owned model/data allocations and bounded array views.
use anyhow::{Context, Result, ensure};
use mujoco_sys as ffi;
use std::{
    ffi::{CStr, CString},
    path::Path,
    ptr::NonNull,
    rc::Rc,
    slice,
    time::Duration,
};
mod scene;
pub use scene::{AppliedForces, Body};

pub const NREF: usize = ffi::mjNREF as usize;
pub const NIMP: usize = ffi::mjNIMP as usize;
pub const NGAIN: usize = ffi::mjNGAIN as usize;
pub const NBIAS: usize = ffi::mjNBIAS as usize;

pub use mujoco_core::{
    ActuatorIndex, BodyIndex, GeomIndex, Integrator, JointIndex, JointKind, Object, ObjectIndex,
    SiteIndex,
};

fn object_kind(kind: Object) -> i32 {
    (match kind {
        Object::Body => ffi::mjOBJ_BODY,
        Object::Joint => ffi::mjOBJ_JOINT,
        Object::Actuator => ffi::mjOBJ_ACTUATOR,
        Object::Geom => ffi::mjOBJ_GEOM,
        Object::Site => ffi::mjOBJ_SITE,
    }) as i32
}

pub fn version() -> String {
    unsafe { CStr::from_ptr(ffi::mj_versionString()) }
        .to_string_lossy()
        .into_owned()
}
fn check_version() -> Result<()> {
    scene::install_callback();
    ensure!(
        unsafe { ffi::mj_version() } == ffi::mjVERSION_HEADER as i32,
        "MuJoCo runtime/header version mismatch"
    );
    Ok(())
}

pub struct Spec(NonNull<ffi::mjSpec>);
impl Drop for Spec {
    fn drop(&mut self) {
        unsafe {
            ffi::mj_deleteSpec(self.0.as_ptr());
        }
    }
}
impl Spec {
    pub fn from_xml(path: &Path) -> Result<Self> {
        check_version()?;
        let path = CString::new(path.as_os_str().as_encoded_bytes())?;
        let mut error = [0; 2048];
        let raw = unsafe {
            ffi::mj_parseXML(
                path.as_ptr(),
                std::ptr::null(),
                error.as_mut_ptr(),
                error.len() as i32,
            )
        };
        Ok(Self(NonNull::new(raw).with_context(|| {
            unsafe { CStr::from_ptr(error.as_ptr()) }
                .to_string_lossy()
                .into_owned()
        })?))
    }
    /// Parse an in-memory MJCF or URDF document.
    /// MuJoCo expects UTF-8 XML. Embedded NUL bytes are rejected.
    pub fn from_xml_bytes(xml: &[u8]) -> Result<Self> {
        check_version()?;
        let xml = CString::new(xml)?;
        let mut error = [0; 2048];
        let raw = unsafe {
            ffi::mj_parseXMLString(
                xml.as_ptr(),
                std::ptr::null(),
                error.as_mut_ptr(),
                error.len() as i32,
            )
        };
        Ok(Self(NonNull::new(raw).with_context(|| {
            unsafe { CStr::from_ptr(error.as_ptr()) }
                .to_string_lossy()
                .into_owned()
        })?))
    }
    pub fn compile(&mut self) -> Result<Model> {
        let raw = unsafe { ffi::mj_compile(self.0.as_ptr(), std::ptr::null()) };
        Ok(Model {
            raw: NonNull::new(raw).with_context(|| {
                unsafe { CStr::from_ptr(ffi::mjs_getError(self.0.as_ptr())) }
                    .to_string_lossy()
                    .into_owned()
            })?,
            id: Rc::new(()),
        })
    }
}

/// Owns an `mjModel`; its allocation layout cannot be changed.
/// Operations on data panic if it was allocated for a different model.
pub struct Model {
    raw: NonNull<ffi::mjModel>,
    // The token outlives this model when data still exists, so reusing the native
    // model's address cannot accidentally validate an incompatible allocation.
    id: Rc<()>,
}
impl Drop for Model {
    fn drop(&mut self) {
        unsafe {
            ffi::mj_deleteModel(self.raw.as_ptr());
        }
    }
}
impl Model {
    pub fn from_xml(path: &Path) -> Result<Self> {
        Spec::from_xml(path)?.compile()
    }
    /// Build a model from an in-memory MJCF or URDF document.
    /// MuJoCo expects UTF-8 XML. Embedded NUL bytes are rejected.
    pub fn from_xml_bytes(xml: &[u8]) -> Result<Self> {
        Spec::from_xml_bytes(xml)?.compile()
    }
    pub fn id<I: ObjectIndex>(&self, name: &str) -> Result<I> {
        let text = CString::new(name)?;
        let id =
            unsafe { ffi::mj_name2id(self.raw.as_ptr(), object_kind(I::OBJECT), text.as_ptr()) };
        ensure!(id >= 0, "missing model element {name}");
        Ok(I::from(id as usize))
    }
    pub fn name<I: ObjectIndex>(&self, id: I) -> Option<&str> {
        let id = i32::try_from(id.into()).ok()?;
        let ptr = unsafe { ffi::mj_id2name(self.raw.as_ptr(), object_kind(I::OBJECT), id) };
        if ptr.is_null() {
            None
        } else {
            unsafe { CStr::from_ptr(ptr) }.to_str().ok()
        }
    }
    pub fn set_timestep(&mut self, timestep: Duration) -> Result<()> {
        ensure!(!timestep.is_zero(), "timestep must be positive");
        unsafe {
            self.raw.as_mut().opt.timestep = timestep.as_secs_f64();
        }
        Ok(())
    }
    pub fn gravity(&self) -> [f64; 3] {
        unsafe { self.raw.as_ref().opt.gravity }
    }
    pub fn set_gravity(&mut self, gravity: [f64; 3]) -> Result<()> {
        ensure!(gravity.iter().all(|v| v.is_finite()), "invalid gravity");
        unsafe {
            self.raw.as_mut().opt.gravity = gravity;
        }
        Ok(())
    }
    pub fn disable_contacts_and_limits(&mut self) {
        unsafe {
            self.raw.as_mut().opt.disableflags |= (ffi::mjDSBL_CONTACT | ffi::mjDSBL_LIMIT) as i32;
        }
    }
    /// Configure a stateless scalar actuator for an affine force law.
    pub fn use_affine_actuator(&mut self, actuator: ActuatorIndex) -> Result<()> {
        let actuator = actuator.0;
        let m = unsafe { self.raw.as_mut() };
        ensure!(actuator < m.nactuator as usize, "invalid actuator index");
        unsafe {
            ensure!(
                *m.actuator_ctrlnum.add(actuator) == 1
                    && *m.actuator_outnum.add(actuator) == 1
                    && *m.actuator_actadr.add(actuator) == -1
                    && *m.actuator_plugin.add(actuator) == -1,
                "motor binding requires a stateless scalar actuator without a plugin"
            );
            *m.actuator_gaintype.add(actuator) = ffi::mjGAIN_FIXED as i32;
            *m.actuator_biastype.add(actuator) = ffi::mjBIAS_AFFINE as i32;
            let gain = slice_mut(m.actuator_gainprm.add(actuator * NGAIN), NGAIN);
            gain.fill(0.);
            gain[0] = 1.;
            slice_mut(m.actuator_biasprm.add(actuator * NBIAS), NBIAS).fill(0.);
            *m.actuator_ctrllimited
                .add(*m.actuator_ctrladr.add(actuator) as usize) = false;
        }
        Ok(())
    }

    /// Keep all bodies active when applying custom forces.
    pub fn disable_sleep(&mut self) {
        unsafe {
            self.raw.as_mut().opt.enableflags &= !(ffi::mjENBL_SLEEP as i32);
        }
    }

    /// Panics if the joint index is out of range.
    pub fn joint_kind(&self, index: JointIndex) -> JointKind {
        let model = unsafe { self.raw.as_ref() };
        assert!(index.0 < model.njnt as usize);
        match unsafe { *model.jnt_type.add(index.0) } {
            x if x == ffi::mjJNT_FREE as i32 => JointKind::Free,
            x if x == ffi::mjJNT_BALL as i32 => JointKind::Ball,
            x if x == ffi::mjJNT_SLIDE as i32 => JointKind::Slide,
            x if x == ffi::mjJNT_HINGE as i32 => JointKind::Hinge,
            kind => panic!("unknown MuJoCo joint kind: {kind}"),
        }
    }

    pub fn integrator(&self) -> Integrator {
        match unsafe { self.raw.as_ref().opt.integrator } {
            x if x == ffi::mjINT_EULER as i32 => Integrator::Euler,
            x if x == ffi::mjINT_RK4 as i32 => Integrator::Rk4,
            x if x == ffi::mjINT_IMPLICIT as i32 => Integrator::Implicit,
            x if x == ffi::mjINT_IMPLICITFAST as i32 => Integrator::ImplicitFast,
            x if x == ffi::mjINT_DISCRETE as i32 => Integrator::Discrete,
            integrator => panic!("unknown MuJoCo integrator: {integrator}"),
        }
    }
}

// Zero-sized MuJoCo arrays may be null. All other pointers/lengths below come
// from the live engine-owned allocation; callers cannot edit layout metadata.
unsafe fn slice_ref<'a, T>(ptr: *const T, len: usize) -> &'a [T] {
    if len == 0 {
        &[]
    } else {
        unsafe { slice::from_raw_parts(ptr, len) }
    }
}
unsafe fn slice_mut<'a, T>(ptr: *mut T, len: usize) -> &'a mut [T] {
    if len == 0 {
        &mut []
    } else {
        unsafe { slice::from_raw_parts_mut(ptr, len) }
    }
}

pub struct ModelRef<'a> {
    pub nq: usize,
    pub nv: usize,
    pub nu: usize,
    pub nout: usize,
    pub nbody: usize,
    pub jnt_qposadr: &'a [i32],
    pub jnt_dofadr: &'a [i32],
    pub actuator_ctrlnum: &'a [i32],
    pub actuator_outnum: &'a [i32],
    pub actuator_ctrladr: &'a [i32],
    pub actuator_outadr: &'a [i32],
    pub jnt_solref: &'a [f64],
    pub jnt_solimp: &'a [f64],
    pub actuator_biasprm: &'a [f64],
    pub body_mass: &'a [f64],
    pub body_ipos: &'a [f64],
    pub body_inertia: &'a [f64],
    pub dof_frictionloss: &'a [f64],
    pub dof_damping: &'a [f64],
    pub jnt_stiffness: &'a [f64],
    pub qpos_spring: &'a [f64],
    pub dof_solref: &'a [f64],
    pub dof_solimp: &'a [f64],
}
impl Model {
    pub fn view(&self) -> ModelRef<'_> {
        unsafe {
            let m = self.raw.as_ref();
            ModelRef {
                nq: m.nq as usize,
                nv: m.nv as usize,
                nu: m.nu as usize,
                nout: m.nout as usize,
                nbody: m.nbody as usize,
                jnt_qposadr: slice_ref(m.jnt_qposadr, (m.njnt) as usize),
                jnt_dofadr: slice_ref(m.jnt_dofadr, (m.njnt) as usize),
                actuator_ctrlnum: slice_ref(m.actuator_ctrlnum, (m.nactuator) as usize),
                actuator_outnum: slice_ref(m.actuator_outnum, (m.nactuator) as usize),
                actuator_ctrladr: slice_ref(m.actuator_ctrladr, (m.nactuator) as usize),
                actuator_outadr: slice_ref(m.actuator_outadr, (m.nactuator) as usize),
                jnt_solref: slice_ref(m.jnt_solref, m.njnt as usize * NREF),
                jnt_solimp: slice_ref(m.jnt_solimp, m.njnt as usize * NIMP),
                actuator_biasprm: slice_ref(m.actuator_biasprm, m.nactuator as usize * NBIAS),
                body_mass: slice_ref(m.body_mass, (m.nbody) as usize),
                body_ipos: slice_ref(m.body_ipos, m.nbody as usize * 3),
                body_inertia: slice_ref(m.body_inertia, m.nbody as usize * 3),
                dof_frictionloss: slice_ref(m.dof_frictionloss, (m.nv) as usize),
                dof_damping: slice_ref(m.dof_damping, (m.nv) as usize),
                jnt_stiffness: slice_ref(m.jnt_stiffness, (m.njnt) as usize),
                qpos_spring: slice_ref(m.qpos_spring, (m.nq) as usize),
                dof_solref: slice_ref(m.dof_solref, m.nv as usize * NREF),
                dof_solimp: slice_ref(m.dof_solimp, m.nv as usize * NIMP),
            }
        }
    }
}

pub struct ModelMut<'a> {
    pub nq: usize,
    pub nv: usize,
    pub nu: usize,
    pub nout: usize,
    pub nbody: usize,
    pub jnt_qposadr: &'a [i32],
    pub jnt_dofadr: &'a [i32],
    pub actuator_ctrlnum: &'a [i32],
    pub actuator_outnum: &'a [i32],
    pub actuator_ctrladr: &'a [i32],
    pub actuator_outadr: &'a [i32],
    pub jnt_solref: &'a mut [f64],
    pub jnt_solimp: &'a mut [f64],
    pub actuator_biasprm: &'a mut [f64],
    pub body_mass: &'a mut [f64],
    pub body_ipos: &'a mut [f64],
    pub body_inertia: &'a mut [f64],
    pub dof_frictionloss: &'a mut [f64],
    pub dof_damping: &'a mut [f64],
    pub jnt_stiffness: &'a mut [f64],
    pub qpos_spring: &'a mut [f64],
    pub dof_solref: &'a mut [f64],
    pub dof_solimp: &'a mut [f64],
}
impl Model {
    pub fn view_mut(&mut self) -> ModelMut<'_> {
        unsafe {
            let m = self.raw.as_ref();
            ModelMut {
                nq: m.nq as usize,
                nv: m.nv as usize,
                nu: m.nu as usize,
                nout: m.nout as usize,
                nbody: m.nbody as usize,
                jnt_qposadr: slice_ref(m.jnt_qposadr, (m.njnt) as usize),
                jnt_dofadr: slice_ref(m.jnt_dofadr, (m.njnt) as usize),
                actuator_ctrlnum: slice_ref(m.actuator_ctrlnum, (m.nactuator) as usize),
                actuator_outnum: slice_ref(m.actuator_outnum, (m.nactuator) as usize),
                actuator_ctrladr: slice_ref(m.actuator_ctrladr, (m.nactuator) as usize),
                actuator_outadr: slice_ref(m.actuator_outadr, (m.nactuator) as usize),
                jnt_solref: slice_mut(m.jnt_solref, m.njnt as usize * NREF),
                jnt_solimp: slice_mut(m.jnt_solimp, m.njnt as usize * NIMP),
                actuator_biasprm: slice_mut(m.actuator_biasprm, m.nactuator as usize * NBIAS),
                body_mass: slice_mut(m.body_mass, (m.nbody) as usize),
                body_ipos: slice_mut(m.body_ipos, m.nbody as usize * 3),
                body_inertia: slice_mut(m.body_inertia, m.nbody as usize * 3),
                dof_frictionloss: slice_mut(m.dof_frictionloss, (m.nv) as usize),
                dof_damping: slice_mut(m.dof_damping, (m.nv) as usize),
                jnt_stiffness: slice_mut(m.jnt_stiffness, (m.njnt) as usize),
                qpos_spring: slice_mut(m.qpos_spring, (m.nq) as usize),
                dof_solref: slice_mut(m.dof_solref, m.nv as usize * NREF),
                dof_solimp: slice_mut(m.dof_solimp, m.nv as usize * NIMP),
            }
        }
    }
}

pub struct DataRef<'a> {
    pub actuator_length: &'a [f64],
    pub actuator_velocity: &'a [f64],
    pub qpos: &'a [f64],
    pub qvel: &'a [f64],
    pub ctrl: &'a [f64],
    pub actuator_force: &'a [f64],
    pub qfrc_applied: &'a [f64],
    pub qfrc_passive: &'a [f64],
    pub qfrc_constraint: &'a [f64],
}

pub struct DataMut<'a> {
    pub qpos: &'a mut [f64],
    pub qvel: &'a mut [f64],
    pub ctrl: &'a mut [f64],
    pub actuator_force: &'a mut [f64],
    pub qfrc_applied: &'a mut [f64],
    pub qfrc_passive: &'a mut [f64],
    pub qfrc_constraint: &'a mut [f64],
}

/// Owns an `mjData`, independently of the model used to allocate it.
/// Multiple data allocations can be used with one model. Borrow rules exclude
/// array views while MuJoCo updates the data.
pub struct Data {
    raw: NonNull<ffi::mjData>,
    model_id: Rc<()>,
    // mjData does not store these immutable array lengths.
    nq: usize,
    nv: usize,
    nu: usize,
    nout: usize,
}
impl Drop for Data {
    fn drop(&mut self) {
        unsafe {
            ffi::mj_deleteData(self.raw.as_ptr());
        }
    }
}
impl Data {
    pub fn new(model: &Model) -> Result<Self> {
        let raw = NonNull::new(unsafe { ffi::mj_makeData(model.raw.as_ptr()) })
            .context("could not allocate MuJoCo data")?;
        let m = model.view();
        Ok(Self {
            raw,
            model_id: Rc::clone(&model.id),
            nq: m.nq,
            nv: m.nv,
            nu: m.nu,
            nout: m.nout,
        })
    }
    pub fn warnings(&self) -> &[ffi::mjWarningStat] {
        unsafe { &self.raw.as_ref().warning }
    }
    pub fn contacts(&self) -> &[ffi::mjContact] {
        unsafe {
            let d = self.raw.as_ref();
            slice_ref(d.contact, d.ncon as usize)
        }
    }
    pub fn view(&self) -> DataRef<'_> {
        unsafe {
            let d = self.raw.as_ref();
            DataRef {
                actuator_length: slice_ref(d.actuator_length, self.nout),
                actuator_velocity: slice_ref(d.actuator_velocity, self.nout),
                qpos: slice_ref(d.qpos, self.nq),
                qvel: slice_ref(d.qvel, self.nv),
                ctrl: slice_ref(d.ctrl, self.nu),
                actuator_force: slice_ref(d.actuator_force, self.nout),
                qfrc_applied: slice_ref(d.qfrc_applied, self.nv),
                qfrc_passive: slice_ref(d.qfrc_passive, self.nv),
                qfrc_constraint: slice_ref(d.qfrc_constraint, self.nv),
            }
        }
    }
    pub fn view_mut(&mut self) -> DataMut<'_> {
        unsafe {
            let d = self.raw.as_ref();
            DataMut {
                qpos: slice_mut(d.qpos, self.nq),
                qvel: slice_mut(d.qvel, self.nv),
                ctrl: slice_mut(d.ctrl, self.nu),
                actuator_force: slice_mut(d.actuator_force, self.nout),
                qfrc_applied: slice_mut(d.qfrc_applied, self.nv),
                qfrc_passive: slice_mut(d.qfrc_passive, self.nv),
                qfrc_constraint: slice_mut(d.qfrc_constraint, self.nv),
            }
        }
    }
}

impl Model {
    fn check_data(&self, data: &Data) {
        assert!(
            Rc::ptr_eq(&self.id, &data.model_id),
            "MuJoCo data belongs to a different model"
        );
    }
    pub fn reset_data(&self, data: &mut Data) {
        self.check_data(data);
        unsafe {
            ffi::mj_resetData(self.raw.as_ptr(), data.raw.as_ptr());
        }
    }
    pub fn forward(&self, data: &mut Data) {
        self.check_data(data);
        unsafe {
            ffi::mj_forward(self.raw.as_ptr(), data.raw.as_ptr());
        }
    }
    pub fn set_constants(&mut self, data: &mut Data) {
        self.check_data(data);
        unsafe {
            ffi::mj_setConst(self.raw.as_ptr(), data.raw.as_ptr());
        }
    }
    pub fn step(&self, data: &mut Data) {
        self.check_data(data);
        unsafe {
            ffi::mj_step(self.raw.as_ptr(), data.raw.as_ptr());
        }
    }
    pub fn kinetic_energy(&self, data: &mut Data) -> f64 {
        self.check_data(data);
        unsafe {
            ffi::mj_energyVel(self.raw.as_ptr(), data.raw.as_ptr());
            data.raw.as_ref().energy[1]
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn memory_loading_preserves_diagnostics_and_rejects_embedded_nul() {
        let error = Spec::from_xml_bytes(b"<mujoco>\n<worldbody invalid='1'/>\n</mujoco>")
            .err()
            .unwrap()
            .to_string();
        assert!(
            error.contains("unrecognized attribute: 'invalid'"),
            "{error}"
        );
        assert!(error.contains("line 2"), "{error}");
        let error = Model::from_xml_bytes(
            b"<mujoco>\n<worldbody><body name='broken'><joint/></body></worldbody>\n</mujoco>",
        )
        .err()
        .unwrap()
        .to_string();
        assert!(error.contains("mass and inertia"), "{error}");
        assert!(error.contains("'broken'"), "{error}");
        assert!(error.contains("line 2"), "{error}");
        assert!(Spec::from_xml_bytes(b"<mujoco/>\0trailing content").is_err());
    }

    #[test]
    fn file_loading_resolves_relative_includes() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("scene.xml");
        std::fs::write(&path, "<mujoco><include file='body.xml'/></mujoco>").unwrap();
        std::fs::write(
            dir.path().join("body.xml"),
            "<mujoco><worldbody><body name='included'/></worldbody></mujoco>",
        )
        .unwrap();
        let model = Model::from_xml(&path).unwrap();
        assert_eq!(model.id::<BodyIndex>("included").unwrap(), BodyIndex(1));
    }

    #[test]
    fn joint_kinds_and_integrators_match_compiled_model() {
        let joints = [
            ("free", JointKind::Free),
            ("ball", JointKind::Ball),
            ("slide", JointKind::Slide),
            ("hinge", JointKind::Hinge),
        ];
        let bodies: String = joints.iter().map(|(name, _)| format!(
            "<body><joint name='{name}' type='{name}'/><geom type='sphere' size='.1'/></body>"
        )).collect();
        for (name, expected) in [
            ("Euler", Integrator::Euler),
            ("RK4", Integrator::Rk4),
            ("implicit", Integrator::Implicit),
            ("implicitfast", Integrator::ImplicitFast),
            ("discrete", Integrator::Discrete),
        ] {
            let model = Model::from_xml_bytes(
                format!(
                    "<mujoco><option integrator='{name}'/><worldbody>{bodies}</worldbody></mujoco>"
                )
                .as_bytes(),
            )
            .unwrap();
            assert_eq!(model.integrator(), expected);
            for (name, kind) in joints {
                assert_eq!(model.joint_kind(model.id(name).unwrap()), kind);
            }
        }
    }

    #[test]
    fn empty_arrays_names_and_independent_data() {
        let mut model = Model::from_xml_bytes(b"<mujoco><worldbody><body name='ball'><joint name='slide' type='slide'/><geom type='sphere' size='.1'/><site name='tip' pos='.2 0 0'/></body></worldbody></mujoco>").unwrap();
        assert!(model.id::<JointIndex>("missing").is_err());
        assert_eq!(model.name(BodyIndex(usize::MAX)), None);
        assert!(model.view().actuator_biasprm.is_empty());
        assert_eq!(
            model.names::<SiteIndex>().collect::<Vec<_>>(),
            [("tip", SiteIndex(0))]
        );
        model.set_timestep(Duration::from_millis(1)).unwrap();
        let mut first = Data::new(&model).unwrap();
        let mut second = Data::new(&model).unwrap();
        first.view_mut().qvel[0] = 1.;
        model.step(&mut first);
        assert!(first.warnings().iter().all(|w| w.number == 0));
        assert_ne!(first.view().qpos[0], 0.);
        model.forward(&mut first);
        let body = model.body(&first, model.id::<BodyIndex>("ball").unwrap());
        assert_eq!(
            body.point_position([0.2, 0., 0.]),
            [0.2, 0., first.view().qpos[0]]
        );
        assert_eq!(
            body.point_velocity([0.2, 0., 0.]),
            [0., 0., first.view().qvel[0]]
        );
        assert_eq!(*body.orientation(), [1., 0., 0., 0.]);
        assert_eq!(*body.position(), [0., 0., first.view().qpos[0]]);
        assert_eq!(second.view().qpos[0], 0.);
        assert!(second.view().ctrl.is_empty());
        assert!(second.view_mut().actuator_force.is_empty());
        model.set_gravity([0.; 3]).unwrap();
        second.view_mut().qvel[0] = -1.;
        model.step(&mut second);
        assert!(second.view().qpos[0] < 0.);
        model.reset_data(&mut first);
        assert_eq!(first.view().qvel[0], 0.);
        assert_eq!(second.view().qvel[0], -1.);
        drop(model);
        // Data owns its arrays and can be read, edited and freed after the model.
        first.view_mut().qpos[0] = 2.;
        assert_eq!(first.view().qpos[0], 2.);
    }

    #[test]
    fn reject_mismatched_data_before_calling_mujoco() {
        let original = Model::from_xml_bytes(b"<mujoco/>").unwrap();
        let mut data = Data::new(&original).unwrap();
        drop(original);
        let mut other = Model::from_xml_bytes(b"<mujoco/>").unwrap();
        let operations: [fn(&mut Model, &mut Data); 6] = [
            |m, d| m.reset_data(d),
            |m, d| m.forward(d),
            |m, d| m.set_constants(d),
            |m, d| m.step(d),
            |m, d| {
                m.body(d, BodyIndex(0));
            },
            |m, d| {
                m.kinetic_energy(d);
            },
        ];
        for operation in operations {
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                operation(&mut other, &mut data);
            }));
            let panic = result.expect_err("accepted data from a different model");
            assert_eq!(
                panic.downcast_ref::<&str>(),
                Some(&"MuJoCo data belongs to a different model")
            );
        }
    }
}
