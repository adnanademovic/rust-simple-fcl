#![deny(warnings)]

#[cfg(test)]
extern crate assert;
pub extern crate nalgebra;
pub extern crate simple_fcl_sys as raw;

pub type Vec3f = nalgebra::Vector3<f32>;
pub type Rotation3f = nalgebra::Rotation3<f32>;
pub type Translation3f = nalgebra::Translation3<f32>;

pub struct ModelBuilder {
    model_ptr: raw::fcl_model_t,
    freed: bool,
}

impl Default for ModelBuilder {
    fn default() -> Self {
        Self {
            model_ptr: unsafe {
                let val = raw::fcl_model_new();
                raw::fcl_model_begin(val);
                val
            },
            freed: false,
        }
    }
}

impl Drop for ModelBuilder {
    fn drop(&mut self) {
        if !self.freed {
            unsafe { raw::fcl_model_free(self.model_ptr) }
        }
    }
}

impl ModelBuilder {
    pub fn add_triangle(&mut self, p0: Vec3f, p1: Vec3f, p2: Vec3f) {
        unsafe {
            raw::fcl_model_add_triangle(
                self.model_ptr,
                p0.as_slice().as_ptr(),
                p1.as_slice().as_ptr(),
                p2.as_slice().as_ptr(),
            )
        }
    }

    pub fn build(mut self) -> Model {
        unsafe {
            raw::fcl_model_end(self.model_ptr);
        }
        self.freed = true;
        Model {
            model_ptr: self.model_ptr,
        }
    }
}

pub struct Model {
    model_ptr: raw::fcl_model_t,
}

impl Drop for Model {
    fn drop(&mut self) {
        unsafe { raw::fcl_model_free(self.model_ptr) }
    }
}

pub fn collide(
    model_a: &Model,
    rotation_a: &Rotation3f,
    translation_a: &Translation3f,
    model_b: &Model,
    rotation_b: &Rotation3f,
    translation_b: &Translation3f,
) -> bool {
    unsafe {
        raw::fcl_collide(
            model_a.model_ptr,
            rotation_a.matrix().as_slice().as_ptr(),
            translation_a.vector.as_slice().as_ptr(),
            model_b.model_ptr,
            rotation_b.matrix().as_slice().as_ptr(),
            translation_b.vector.as_slice().as_ptr(),
        ) != 0
    }
}

#[derive(Clone, Debug, Default)]
pub struct DistanceOptions {
    absolute_error: f64,
    relative_error: f64,
}

pub fn distance(
    model_a: &Model,
    rotation_a: &Rotation3f,
    translation_a: &Translation3f,
    model_b: &Model,
    rotation_b: &Rotation3f,
    translation_b: &Translation3f,
    options: &DistanceOptions,
) -> Option<f64> {
    let mut success = 0;
    let mut distance = 0.0;
    let mut points = [0.0; 6];
    unsafe {
        raw::fcl_distance(
            model_a.model_ptr,
            rotation_a.matrix().as_slice().as_ptr(),
            translation_a.vector.as_slice().as_ptr(),
            model_b.model_ptr,
            rotation_b.matrix().as_slice().as_ptr(),
            translation_b.vector.as_slice().as_ptr(),
            0,
            options.relative_error,
            options.absolute_error,
            &mut success,
            &mut distance,
            points[0..3].as_mut_ptr(),
            points[3..6].as_mut_ptr(),
        );
    }
    if success != 0 {
        Some(distance)
    } else {
        None
    }
}

pub struct DistancePoints {
    pub distance: f64,
    pub point_a: Vec3f,
    pub point_b: Vec3f,
}

pub fn distance_points(
    model_a: &Model,
    rotation_a: &Rotation3f,
    translation_a: &Translation3f,
    model_b: &Model,
    rotation_b: &Rotation3f,
    translation_b: &Translation3f,
    options: &DistanceOptions,
) -> Option<DistancePoints> {
    let mut success = 0;
    let mut distance = 0.0;
    let mut points = [0.0f32; 6];
    unsafe {
        raw::fcl_distance(
            model_a.model_ptr,
            rotation_a.matrix().as_slice().as_ptr(),
            translation_a.vector.as_slice().as_ptr(),
            model_b.model_ptr,
            rotation_b.matrix().as_slice().as_ptr(),
            translation_b.vector.as_slice().as_ptr(),
            1,
            options.relative_error,
            options.absolute_error,
            &mut success,
            &mut distance,
            points[0..3].as_mut_ptr(),
            points[3..6].as_mut_ptr(),
        );
    }
    if success != 0 {
        Some(DistancePoints {
            distance,
            point_a: Vec3f::from_column_slice(&points[0..3]),
            point_b: Vec3f::from_column_slice(&points[3..6]),
        })
    } else {
        None
    }
}

#[cfg(test)]
mod tests;
