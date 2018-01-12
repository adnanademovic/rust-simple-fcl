use super::*;

use assert::close;

fn make_model(coords: &[Vec3f]) -> Model {
    let mut mb = ModelBuilder::default();
    coords
        .chunks(3)
        .for_each(|v| mb.add_triangle(v[0], v[1], v[2]));
    mb.build()
}


#[test]
fn trivial_collision() {
    let model_a = make_model(&[
        Vec3f::new(10.0, 0.0, 0.0),
        Vec3f::new(0.0, 0.0, 1.0),
        Vec3f::new(0.0, 0.0, -1.0),
    ]);
    let model_b = make_model(&[
        Vec3f::new(5.0, -10.0, -10.0),
        Vec3f::new(5.0, -10.0, 10.0),
        Vec3f::new(5.0, 10.0, 0.0),
    ]);

    let rotation_none = Rotation3f::identity();
    let translation = Translation3f::new(0.0, 0.0, 0.0);

    assert!(collide(
        &model_a,
        &rotation_none,
        &translation,
        &model_b,
        &rotation_none,
        &translation
    ));

    assert!(!collide(
        &model_a,
        &Rotation3f::from_euler_angles(0.0, 0.0, ::std::f32::consts::PI / 2.0),
        &translation,
        &model_b,
        &rotation_none,
        &translation
    ));
}

#[test]
fn rotation_orientates_correctly() {
    let model_a = make_model(&[
        Vec3f::new(10.0, 0.0, 0.0),
        Vec3f::new(0.0, 0.0, 1.0),
        Vec3f::new(0.0, 0.0, -1.0),
    ]);
    let model_b = make_model(&[
        Vec3f::new(5.0, 1.0, -10.0),
        Vec3f::new(5.0, 1.0, 10.0),
        Vec3f::new(5.0, 10.0, 0.0),
    ]);

    let rotation_none = Rotation3f::identity();
    let translation = Translation3f::new(0.0, 0.0, 0.0);

    assert!(!collide(
        &model_a,
        &rotation_none,
        &translation,
        &model_b,
        &rotation_none,
        &translation
    ));

    assert!(collide(
        &model_a,
        &Rotation3f::from_euler_angles(0.0, 0.0, ::std::f32::consts::PI / 6.0),
        &translation,
        &model_b,
        &rotation_none,
        &translation
    ));

    assert!(!collide(
        &model_a,
        &Rotation3f::from_euler_angles(0.0, 0.0, -::std::f32::consts::PI / 6.0),
        &translation,
        &model_b,
        &rotation_none,
        &translation
    ));
}

#[test]
fn translation_moves_correctly() {
    let model_a = make_model(&[
        Vec3f::new(10.0, 0.0, 0.0),
        Vec3f::new(0.0, 0.0, 1.0),
        Vec3f::new(0.0, 0.0, -1.0),
    ]);
    let model_b = make_model(&[
        Vec3f::new(5.0, 1.0, -10.0),
        Vec3f::new(5.0, 1.0, 10.0),
        Vec3f::new(5.0, 10.0, 0.0),
    ]);

    let rotation_none = Rotation3f::identity();
    let translation_none = Translation3f::new(0.0, 0.0, 0.0);

    assert!(!collide(
        &model_a,
        &rotation_none,
        &translation_none,
        &model_b,
        &rotation_none,
        &translation_none
    ));

    assert!(collide(
        &model_a,
        &rotation_none,
        &Translation3f::new(0.0, 2.0, 0.0),
        &model_b,
        &rotation_none,
        &translation_none
    ));

    assert!(collide(
        &model_a,
        &rotation_none,
        &Translation3f::new(0.0, 5.0, 0.0),
        &model_b,
        &rotation_none,
        &translation_none
    ));

    assert!(!collide(
        &model_a,
        &rotation_none,
        &Translation3f::new(0.0, 12.0, 0.0),
        &model_b,
        &rotation_none,
        &translation_none
    ));
}

#[test]
fn trivial_distance() {
    let model_a = make_model(&[
        Vec3f::new(10.0, 0.0, 0.0),
        Vec3f::new(0.0, 0.0, 1.0),
        Vec3f::new(0.0, 0.0, -1.0),
    ]);
    let model_b = make_model(&[
        Vec3f::new(5.0, -10.0, -10.0),
        Vec3f::new(5.0, -10.0, 10.0),
        Vec3f::new(5.0, 10.0, 0.0),
    ]);

    let rotation_none = Rotation3f::identity();
    let translation = Translation3f::new(0.0, 0.0, 0.0);

    assert!(
        distance(
            &model_a,
            &rotation_none,
            &translation,
            &model_b,
            &rotation_none,
            &translation,
            &DistanceOptions::default(),
        ).is_none()
    );

    close(
        distance(
            &model_a,
            &Rotation3f::from_euler_angles(0.0, 0.0, ::std::f32::consts::PI / 2.0),
            &translation,
            &model_b,
            &rotation_none,
            &translation,
            &DistanceOptions::default(),
        ).unwrap(),
        5.0,
        0.001,
    );
}

#[test]
fn distance_is_right_with_translation() {
    let model_a = make_model(&[
        Vec3f::new(10.0, 0.0, 0.0),
        Vec3f::new(0.0, 0.0, 1.0),
        Vec3f::new(0.0, 0.0, -1.0),
    ]);
    let model_b = make_model(&[
        Vec3f::new(5.0, 1.0, -10.0),
        Vec3f::new(5.0, 1.0, 10.0),
        Vec3f::new(5.0, 10.0, 0.0),
    ]);

    let rotation_none = Rotation3f::identity();
    let translation_none = Translation3f::new(0.0, 0.0, 0.0);

    close(
        distance(
            &model_a,
            &rotation_none,
            &translation_none,
            &model_b,
            &rotation_none,
            &translation_none,
            &DistanceOptions::default(),
        ).unwrap(),
        1.0,
        0.001,
    );

    assert!(
        distance(
            &model_a,
            &rotation_none,
            &Translation3f::new(0.0, 2.0, 0.0),
            &model_b,
            &rotation_none,
            &translation_none,
            &DistanceOptions::default(),
        ).is_none()
    );

    assert!(
        distance(
            &model_a,
            &rotation_none,
            &Translation3f::new(0.0, 5.0, 0.0),
            &model_b,
            &rotation_none,
            &translation_none,
            &DistanceOptions::default(),
        ).is_none()
    );

    close(
        distance(
            &model_a,
            &rotation_none,
            &Translation3f::new(0.0, 12.0, 0.0),
            &model_b,
            &rotation_none,
            &translation_none,
            &DistanceOptions::default(),
        ).unwrap(),
        2.0,
        0.001,
    );
}
