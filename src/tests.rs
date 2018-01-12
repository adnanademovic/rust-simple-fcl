use super::*;

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
