use nalgebra::base::{Matrix4, RowVector4, Vector3};

enum Axis {
  X,
  Y,
  Z,
}

fn generate_rotation_matrix(axis: Axis, angle: f32) -> Matrix4<f32> {
  match axis {
    Axis::X => Matrix4::from_rows(&[
      RowVector4::new(1., 0., 0., 0.),
      RowVector4::new(0., f32::cos(angle), -f32::sin(angle), 0.),
      RowVector4::new(0., f32::sin(angle), f32::cos(angle), 0.),
      RowVector4::new(0., 0., 0., 1.),
    ]),
    Axis::Y => Matrix4::from_rows(&[
      RowVector4::new(f32::cos(angle), 0., f32::sin(angle), 0.),
      RowVector4::new(0., 1., 0., 0.),
      RowVector4::new(-f32::sin(angle), 0., f32::cos(angle), 0.),
      RowVector4::new(0., 0., 0., 1.),
    ]),
    Axis::Z => Matrix4::from_rows(&[
      RowVector4::new(f32::cos(angle), -f32::sin(angle), 0., 0.),
      RowVector4::new(f32::sin(angle), f32::cos(angle), 0., 0.),
      RowVector4::new(0., 0., 1., 0.),
      RowVector4::new(0., 0., 0., 1.),
    ]),
  }
}

fn r_x(roll: f32) -> Matrix4<f32> {
  generate_rotation_matrix(Axis::X, roll)
}

fn r_y(pitch: f32) -> Matrix4<f32> {
  generate_rotation_matrix(Axis::Y, pitch)
}

fn r_z(yaw: f32) -> Matrix4<f32> {
  generate_rotation_matrix(Axis::Z, yaw)
}

pub fn r_xyz(roll: f32, pitch: f32, yaw: f32) -> Matrix4<f32> {
  r_x(roll) * r_y(pitch) * r_z(yaw)
}

fn rt_matrix(rotation: Vector3<f32>, translation: Vector3<f32>) -> Matrix4<f32> {
  let roll = rotation[0];
  let pitch = rotation[1];
  let yaw = rotation[2];

  let x = translation[0];
  let y = translation[1];
  let z = translation[2];

  let t = Matrix4::new(1., 0., 0., x, 0., 1., 0., y, 0., 0., 1., z, 0., 0., 0., 1.);
  let r = r_xyz(roll, pitch, yaw);

  r * t
}

#[test]
fn test_r_xyz() {
  assert_eq!(r_xyz(0., 0., 0.), Matrix4::identity());
}
