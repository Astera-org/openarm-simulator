//! MuJoCo model editing, owned model/data allocations and bounded array views.
use anyhow::{Context, Result, ensure};
use mujoco_sys_rs as ffi;
use std::{
    ffi::{CStr, CString},
    path::Path,
    ptr::NonNull,
    rc::Rc,
    slice,
    time::Duration,
};

pub const NREF: usize = ffi::mjNREF as usize;
pub const NIMP: usize = ffi::mjNIMP as usize;
pub const NGAIN: usize = ffi::mjNGAIN as usize;
pub const NBIAS: usize = ffi::mjNBIAS as usize;
pub const JOINT_HINGE: i32 = ffi::mjJNT_HINGE as i32;
pub const JOINT_SLIDE: i32 = ffi::mjJNT_SLIDE as i32;

#[derive(Clone, Copy)]
pub enum Object {
    Body,
    Joint,
    Actuator,
    Geom,
}
impl Object {
    fn raw(self) -> i32 {
        (match self {
            Self::Body => ffi::mjOBJ_BODY,
            Self::Joint => ffi::mjOBJ_JOINT,
            Self::Actuator => ffi::mjOBJ_ACTUATOR,
            Self::Geom => ffi::mjOBJ_GEOM,
        }) as i32
    }
}

pub fn version() -> String {
    unsafe { CStr::from_ptr(ffi::mj_versionString()) }
        .to_string_lossy()
        .into_owned()
}
fn check_version() -> Result<()> {
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
    pub fn exclude_contact(&mut self, first: &str, second: &str) -> Result<()> {
        let (first, second) = (CString::new(first)?, CString::new(second)?);
        unsafe {
            let exclusion = NonNull::new(ffi::mjs_addExclude(self.0.as_ptr()))
                .context("could not add contact exclusion")?;
            ffi::mjs_setString(exclusion.as_ref().bodyname1, first.as_ptr());
            ffi::mjs_setString(exclusion.as_ref().bodyname2, second.as_ptr());
        }
        Ok(())
    }
    fn actuator(&self, name: &str) -> Result<NonNull<ffi::mjsActuator>> {
        let name = CString::new(name)?;
        unsafe {
            let element = ffi::mjs_findElement(self.0.as_ptr(), ffi::mjOBJ_ACTUATOR, name.as_ptr());
            ensure!(!element.is_null(), "missing actuator {name:?}");
            NonNull::new(ffi::mjs_asActuator(element)).context("wrong element type")
        }
    }
    pub fn actuator_force_range(&self, name: &str) -> Result<[f64; 2]> {
        Ok(unsafe { self.actuator(name)?.as_ref().forcerange })
    }
    pub fn set_actuator_tendon(
        &mut self,
        name: &str,
        tendon: &str,
        gear: f64,
        force_range: [f64; 2],
    ) -> Result<()> {
        let tendon = CString::new(tendon)?;
        unsafe {
            let mut actuator = self.actuator(name)?;
            let a = actuator.as_mut();
            a.trntype = ffi::mjTRN_TENDON;
            ffi::mjs_setString(a.target, tendon.as_ptr());
            a.gear[0] = gear;
            a.forcerange = force_range;
        }
        Ok(())
    }
    pub fn remove_actuator(&mut self, name: &str) -> Result<()> {
        unsafe {
            let actuator = self.actuator(name)?;
            ensure!(
                ffi::mjs_delete(self.0.as_ptr(), actuator.as_ref().element) == 0,
                "could not remove actuator {name}"
            );
        }
        Ok(())
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
    pub fn id(&self, kind: Object, name: &str) -> Result<usize> {
        let text = CString::new(name)?;
        let id = unsafe { ffi::mj_name2id(self.raw.as_ptr(), kind.raw(), text.as_ptr()) };
        ensure!(id >= 0, "missing model element {name}");
        Ok(id as usize)
    }
    pub fn name(&self, kind: Object, id: usize) -> Option<&str> {
        let id = i32::try_from(id).ok()?;
        let ptr = unsafe { ffi::mj_id2name(self.raw.as_ptr(), kind.raw(), id) };
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
    pub fn use_implicit_fast(&mut self) {
        unsafe {
            self.raw.as_mut().opt.integrator = ffi::mjINT_IMPLICITFAST as i32;
        }
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
    pub fn use_affine_actuators(&mut self) {
        unsafe {
            let m = self.raw.as_mut();
            slice_mut(m.actuator_gaintype, m.nactuator as usize).fill(ffi::mjGAIN_FIXED as i32);
            slice_mut(m.actuator_biastype, m.nactuator as usize).fill(ffi::mjBIAS_AFFINE as i32);
            slice_mut(m.actuator_gainprm, m.nactuator as usize * NGAIN)
                .chunks_mut(NGAIN)
                .for_each(|a| {
                    a.fill(0.);
                    a[0] = 1.;
                });
            slice_mut(m.actuator_biasprm, m.nactuator as usize * NBIAS).fill(0.);
            slice_mut(m.actuator_ctrllimited, m.nu as usize).fill(false);
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
    pub jnt_type: &'a [i32],
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
                jnt_type: slice_ref(m.jnt_type, (m.njnt) as usize),
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
    pub jnt_type: &'a [i32],
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
                jnt_type: slice_ref(m.jnt_type, (m.njnt) as usize),
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
    fn empty_arrays_names_and_independent_data() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("scene.xml");
        std::fs::write(&path, "<mujoco><worldbody><body name='ball'><joint name='slide' type='slide'/><geom type='sphere' size='.1'/></body></worldbody></mujoco>").unwrap();
        let mut model = Model::from_xml(&path).unwrap();
        assert!(model.id(Object::Joint, "missing").is_err());
        assert_eq!(model.name(Object::Body, usize::MAX), None);
        assert!(model.view().actuator_biasprm.is_empty());
        model.set_timestep(Duration::from_millis(1)).unwrap();
        let mut first = Data::new(&model).unwrap();
        let mut second = Data::new(&model).unwrap();
        first.view_mut().qvel[0] = 1.;
        model.step(&mut first);
        assert!(first.warnings().iter().all(|w| w.number == 0));
        assert_ne!(first.view().qpos[0], 0.);
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
        std::fs::write(&path, "not xml").unwrap();
        assert!(Spec::from_xml(&path).is_err());
    }

    #[test]
    fn reject_mismatched_data_before_calling_mujoco() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("scene.xml");
        std::fs::write(&path, "<mujoco/>").unwrap();
        let original = Model::from_xml(&path).unwrap();
        let mut data = Data::new(&original).unwrap();
        drop(original);
        let mut other = Model::from_xml(&path).unwrap();
        let operations: [fn(&mut Model, &mut Data); 5] = [
            |m, d| m.reset_data(d),
            |m, d| m.forward(d),
            |m, d| m.set_constants(d),
            |m, d| m.step(d),
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
