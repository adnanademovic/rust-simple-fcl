//! Library for using FCL to do collision and distance checks between meshes
//! # Examples
//!
//! ```rust
//! let mut model_builder_a = simple_fcl::ModelBuilder::default();
//! model_builder_a.add_triangle(
//!     simple_fcl::Vec3f::new(10.0, 0.0, 0.0),
//!     simple_fcl::Vec3f::new(0.0, 0.0, 1.0),
//!     simple_fcl::Vec3f::new(0.0, 0.0, -1.0),
//! );
//! let model_a = model_builder_a.build();
//!
//! let mut model_builder_b = simple_fcl::ModelBuilder::default();
//! model_builder_b.add_triangle(
//!     simple_fcl::Vec3f::new(5.0, -10.0, -10.0),
//!     simple_fcl::Vec3f::new(5.0, -10.0, 10.0),
//!     simple_fcl::Vec3f::new(5.0, 10.0, 0.0),
//! );
//! let model_b = model_builder_b.build();
//!
//! let rotation_none = simple_fcl::Rotation3f::identity();
//! let translation_none = simple_fcl::Translation3f::new(0.0, 0.0, 0.0);
//!
//! assert!(simple_fcl::collide(
//!     &model_a,
//!     &rotation_none,
//!     &translation_none,
//!     &model_b,
//!     &rotation_none,
//!     &translation_none,
//! ));
//!
//! assert!(simple_fcl::distance(
//!     &model_a,
//!     &rotation_none,
//!     &translation_none,
//!     &model_b,
//!     &rotation_none,
//!     &translation_none,
//!     &simple_fcl::DistanceOptions::default(),
//! ).is_none());
//! ```
#![deny(warnings)]
#![deny(missing_docs)]

#[cfg(test)]
extern crate assert;
pub extern crate nalgebra;
pub extern crate simple_fcl_sys as raw;

/// Holds data for one point
pub type Vec3f = nalgebra::Vector3<f32>;

/// Holds data for rotations
pub type Rotation3f = nalgebra::Rotation3<f32>;

/// Holds data for translations
pub type Translation3f = nalgebra::Translation3<f32>;

/// Builds models out of passed triangles.
pub struct ModelBuilder {
    model_ptr: raw::fcl_model_t,
    freed: bool,
}

impl ModelBuilder {
    /// Create a new model builder.
    ///
    /// This is just an alias for `default`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
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
    /// Add a triangle to the built model.
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

    /// Consume the model builder and get the finished model.
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

/// Model created by model builder, used for collision and distance checks.
pub struct Model {
    model_ptr: raw::fcl_model_t,
}

impl Drop for Model {
    fn drop(&mut self) {
        unsafe { raw::fcl_model_free(self.model_ptr) }
    }
}

/// Perform collision check between two models transformed in space.
///
/// Each model has a translation and rotation applied to it.
/// So the model center is moved by the translation, while the model is rotated
/// around its center.
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

/// Options for distance check.
///
/// Sets the maximum allowed distance underestimate.
///
/// For distance `D`, with absolute error `a` and relative error `r`, we can
/// get a distance measurement of anywhere between `max(D - a, D * (1 - r))`
/// and `D`.
#[derive(Clone, Debug, Default)]
pub struct DistanceOptions {
    /// Represents and absolute distance error
    pub absolute_error: f64,
    /// Represents the relative distance error as a fraction of the measured
    /// distance, within the [0, 1] range.
    pub relative_error: f64,
}

/// Perform distance check between two models transformed in space.
///
/// Each model has a translation and rotation applied to it.
/// So the model center is moved by the translation, while the model is rotated
/// around its center.
///
/// The returned value is `None` in case of collision, otherwise it's the
/// distance.
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

/// Structure that holds the distance between objects, together with the points
/// between which this distance is achieved.
///
/// The points don't have to be unique, so this is just one single instance of
/// them. If the models had surfaces that were parallel to each other, the
/// number of point pairs would be infinite.
pub struct DistancePoints {
    /// Distance between the two models
    pub distance: f64,
    /// Point on the first model that is closest to the second model
    pub point_a: Vec3f,
    /// Point on the second model that is closest to the first model
    pub point_b: Vec3f,
}

/// Perform distance check between two models transformed in space.
///
/// Each model has a translation and rotation applied to it.
/// So the model center is moved by the translation, while the model is rotated
/// around its center.
///
/// The returned value is `None` in case of collision, otherwise it's the
/// distance together with the points between which the distance was achieved.
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
