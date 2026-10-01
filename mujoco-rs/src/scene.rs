use super::*;

/// Borrow a site's current kinematics. Quaternions use wxyz order.
pub struct Site<'a> {
    model: &'a ffi::mjModel,
    data: &'a ffi::mjData,
    index: usize,
}
impl<'a> Site<'a> {
    fn new(model: &'a ffi::mjModel, data: &'a ffi::mjData, index: usize) -> Self {
        assert!(index < model.nsite as usize);
        Self { model, data, index }
    }

    pub fn body(&self) -> BodyIndex {
        unsafe { BodyIndex(*self.model.site_bodyid.add(self.index) as usize) }
    }

    pub fn position(&self) -> &[f64; 3] {
        unsafe {
            slice_ref(self.data.site_xpos.add(3 * self.index), 3)
                .try_into()
                .unwrap()
        }
    }

    pub fn orientation(&self) -> [f64; 4] {
        let mut quaternion = [0.; 4];
        unsafe {
            ffi::mju_mat2Quat(
                quaternion.as_mut_ptr(),
                self.data.site_xmat.add(9 * self.index),
            );
        }
        quaternion
    }

    /// Read the site's linear velocity in world axes.
    pub fn velocity(&self) -> [f64; 3] {
        let mut velocity = [0.; 6];
        unsafe {
            ffi::mj_objectVelocity(
                self.model,
                self.data,
                ffi::mjOBJ_SITE as i32,
                self.index as i32,
                velocity.as_mut_ptr(),
                0,
            );
        }
        velocity[3..].try_into().unwrap()
    }
}

/// Borrow a body's world pose and centre of mass. Quaternions use wxyz order.
pub struct Body<'a> {
    data: &'a ffi::mjData,
    index: usize,
}
impl Body<'_> {
    pub fn position(&self) -> &[f64; 3] {
        unsafe {
            slice_ref(self.data.xpos.add(3 * self.index), 3)
                .try_into()
                .unwrap()
        }
    }

    pub fn orientation(&self) -> &[f64; 4] {
        unsafe {
            slice_ref(self.data.xquat.add(4 * self.index), 4)
                .try_into()
                .unwrap()
        }
    }

    pub fn com(&self) -> &[f64; 3] {
        unsafe {
            slice_ref(self.data.xipos.add(3 * self.index), 3)
                .try_into()
                .unwrap()
        }
    }
}

impl Model {
    pub fn count<I: ObjectIndex>(&self) -> usize {
        let m = unsafe { self.raw.as_ref() };
        (match I::OBJECT {
            Object::Body => m.nbody,
            Object::Joint => m.njnt,
            Object::Actuator => m.nactuator,
            Object::Geom => m.ngeom,
            Object::Site => m.nsite,
        }) as usize
    }

    /// Resolve named objects to their indices. Unnamed objects are omitted.
    pub fn names<I: ObjectIndex>(&self) -> impl Iterator<Item = (&str, I)> {
        (0..self.count::<I>())
            .map(I::from)
            .filter_map(move |index| self.name(index).map(|name| (name, index)))
    }

    /// Observe a body after forward dynamics. Panics if the index is out of range.
    pub fn body<'a>(&'a self, data: &'a Data, index: BodyIndex) -> Body<'a> {
        self.check_data(data);
        assert!(index.0 < self.count::<BodyIndex>());
        Body {
            data: unsafe { data.raw.as_ref() },
            index: index.0,
        }
    }

    /// Observe a site after forward dynamics. Panics if the index is out of range.
    pub fn site<'a>(&'a self, data: &'a Data, index: SiteIndex) -> Site<'a> {
        self.check_data(data);
        unsafe { Site::new(self.raw.as_ref(), data.raw.as_ref(), index.0) }
    }
}

/// Access to current kinematics and additive forces during an applied-force callback.
pub struct AppliedForces<'a> {
    model: *const ffi::mjModel,
    data: *mut ffi::mjData,
    _borrow: std::marker::PhantomData<&'a mut Data>,
}
impl AppliedForces<'_> {
    /// Observe a site at the current dynamics evaluation. Panics if the index is out of range.
    pub fn site(&self, index: SiteIndex) -> Site<'_> {
        unsafe { Site::new(&*self.model, &*self.data, index.0) }
    }

    /// Set joint-space loads for this dynamics evaluation.
    pub fn generalized(&mut self) -> &mut [f64] {
        unsafe { slice_mut((*self.data).qfrc_applied, (*self.model).nv as usize) }
    }

    /// Add a force and torque at a point on a body, all expressed in world axes.
    pub fn apply(&mut self, body: BodyIndex, point: [f64; 3], force: [f64; 3], torque: [f64; 3]) {
        unsafe {
            assert!(body.0 < (*self.model).nbody as usize);
            ffi::mj_applyFT(
                self.model,
                self.data,
                force.as_ptr(),
                torque.as_ptr(),
                point.as_ptr(),
                body.0 as i32,
                (*self.data).qfrc_applied,
            );
        }
    }
}

// MuJoCo's callback pointer is global. Install one dispatcher; each synchronous
// evaluation borrows its callback on the calling thread, without a global lock.
#[derive(Clone, Copy)]
struct Active {
    model: *const ffi::mjModel,
    data: *mut ffi::mjData,
    context: *mut (),
    call: unsafe fn(*mut (), &mut AppliedForces<'_>),
}
thread_local! {
    static ACTIVE: std::cell::Cell<Option<Active>> = const { std::cell::Cell::new(None) };
}
unsafe extern "C" fn dispatch(model: *const ffi::mjModel, data: *mut ffi::mjData) {
    ACTIVE.with(|slot| {
        if let Some(active) = slot.get().filter(|a| a.model == model && a.data == data) {
            let mut forces = AppliedForces {
                model,
                data,
                _borrow: std::marker::PhantomData,
            };
            unsafe { (active.call)(active.context, &mut forces) };
        }
    });
}

impl Model {
    /// Step with additional applied forces, including every RK4 stage.
    /// Callback forces are not differentiated by the implicit integrators.
    pub fn step_with_forces(&self, data: &mut Data, callback: impl FnMut(&mut AppliedForces<'_>)) {
        self.with_forces(data, callback, ffi::mj_step);
    }

    /// Evaluate dynamics with additional applied forces without advancing time.
    pub fn forward_with_forces(
        &self,
        data: &mut Data,
        callback: impl FnMut(&mut AppliedForces<'_>),
    ) {
        self.with_forces(data, callback, ffi::mj_forward);
    }

    fn with_forces<F: FnMut(&mut AppliedForces<'_>)>(
        &self,
        data: &mut Data,
        callback: F,
        operation: unsafe extern "C" fn(*const ffi::mjModel, *mut ffi::mjData),
    ) {
        self.check_data(data);
        struct Invocation<F> {
            callback: F,
            panic: Option<Box<dyn std::any::Any + Send>>,
        }
        unsafe fn call<F: FnMut(&mut AppliedForces<'_>)>(
            context: *mut (),
            forces: &mut AppliedForces<'_>,
        ) {
            let invocation = unsafe { &mut *context.cast::<Invocation<F>>() };
            if invocation.panic.is_none() {
                invocation.panic = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    (invocation.callback)(forces)
                }))
                .err();
            }
        }
        struct Clear;
        impl Drop for Clear {
            fn drop(&mut self) {
                ACTIVE.with(|slot| slot.set(None));
            }
        }
        let mut invocation = Invocation {
            callback,
            panic: None,
        };
        ACTIVE.with(|slot| {
            assert!(slot.get().is_none(), "nested applied-force evaluation");
            slot.set(Some(Active {
                model: self.raw.as_ptr(),
                data: data.raw.as_ptr(),
                context: (&mut invocation as *mut Invocation<F>).cast(),
                call: call::<F>,
            }));
        });
        let guard = Clear;
        unsafe {
            operation(self.raw.as_ptr(), data.raw.as_ptr());
        }
        drop(guard);
        if let Some(panic) = invocation.panic {
            std::panic::resume_unwind(panic);
        }
    }
}

pub(super) fn install_callback() {
    static INSTALL: std::sync::Once = std::sync::Once::new();
    INSTALL.call_once(|| unsafe {
        let existing = ffi::mjcb_control;
        assert!(
            existing.is_none(),
            "MuJoCo control callback already installed"
        );
        ffi::mjcb_control = Some(dispatch);
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn force_callbacks_are_scoped_and_panics_do_not_cross_ffi() {
        let threads: Vec<_> = (1..=2).map(|force| std::thread::spawn(move || {
            let model = Model::from_xml_bytes(b"<mujoco><option gravity='0 0 0' integrator='RK4'/><worldbody><body><joint type='slide' axis='1 0 0'/><geom type='sphere' size='.1' mass='1'/></body></worldbody></mujoco>").unwrap();
            let mut data = Data::new(&model).unwrap();
            let mut calls = 0;
            model.step_with_forces(&mut data, |context| {
                calls += 1;
                context.generalized()[0] = force as f64;
            });
            assert_eq!(calls, 4);
            assert!((data.view().qvel[0] - 0.002 * force as f64).abs() < 1e-12);
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                model.forward_with_forces(&mut data, |_| panic!("callback panic"));
            }));
            assert!(result.is_err());
            model.forward_with_forces(&mut data, |context| context.generalized().fill(0.));
            assert_eq!(data.view().qfrc_applied, &[0.]);
            model.step(&mut data);
            assert!((data.view().qvel[0] - 0.002 * force as f64).abs() < 1e-12);
        })).collect();
        for thread in threads {
            thread.join().unwrap();
        }
    }
}
