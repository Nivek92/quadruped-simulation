use nalgebra::base::{Matrix4, RowVector4, Vector4};

pub struct RobotConfiguration {
  pub l: f32,
  pub w: f32,
  pub l1: f32,
  pub l2: f32,
  pub l3: f32,
  pub l4: f32,
}

impl RobotConfiguration {
  pub fn new(l: f32, w: f32, l1: f32, l2: f32, l3: f32, l4: f32) -> Self {
    RobotConfiguration {
      l,
      w,
      l1,
      l2,
      l3,
      l4,
    }
  }
}

enum Axis {
  X,
  Y,
  Z,
}

fn generate_rotation_matrix(axis: Axis, angle: f32) -> Matrix4<f32> {
  let c = f32::cos(angle);
  let s = f32::sin(angle);

  match axis {
    Axis::X => Matrix4::from_rows(&[
      RowVector4::new(1., 0., 0., 0.),
      RowVector4::new(0., c, -s, 0.),
      RowVector4::new(0., s, c, 0.),
      RowVector4::new(0., 0., 0., 1.),
    ]),
    Axis::Y => Matrix4::from_rows(&[
      RowVector4::new(c, 0., s, 0.),
      RowVector4::new(0., 1., 0., 0.),
      RowVector4::new(-s, 0., c, 0.),
      RowVector4::new(0., 0., 0., 1.),
    ]),
    Axis::Z => Matrix4::from_rows(&[
      RowVector4::new(c, -s, 0., 0.),
      RowVector4::new(s, c, 0., 0.),
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

fn r_xyz(roll: f32, pitch: f32, yaw: f32) -> Matrix4<f32> {
  r_x(roll) * r_y(pitch) * r_z(yaw)
}

fn rt_matrix(rotation: [f32; 3], translation: [f32; 3]) -> Matrix4<f32> {
  let roll = rotation[0];
  let pitch = rotation[1];
  let yaw = rotation[2];

  let x = translation[0];
  let y = translation[1];
  let z = translation[2];

  let t = Matrix4::new(0., 0., 0., x, 0., 0., 0., y, 0., 0., 0., z, 0., 0., 0., 0.);
  let r = r_xyz(roll, pitch, yaw);

  t + r
}

pub fn body_ik(
  rotation: [f32; 3],
  translation: [f32; 3],
  robot_configuration: &RobotConfiguration,
) -> [Matrix4<f32>; 5] {
  let l = robot_configuration.l;
  let w = robot_configuration.w;

  let tm = rt_matrix(rotation, translation);
  let tlf = tm * rt_matrix([0., std::f32::consts::FRAC_PI_2, 0.], [l / 2., 0., w / 2.]);
  let trf = tm * rt_matrix([0., std::f32::consts::FRAC_PI_2, 0.], [l / 2., 0., -w / 2.]);
  let tlb = tm * rt_matrix([0., std::f32::consts::FRAC_PI_2, 0.], [-l / 2., 0., w / 2.]);
  let trb = tm
    * rt_matrix(
      [0., std::f32::consts::FRAC_PI_2, 0.],
      [-l / 2., 0., -w / 2.],
    );

  [tlf, trf, tlb, trb, tm]
}

pub fn leg_ik(target: Vector4<f32>, robot_configuration: &RobotConfiguration) -> [f32; 3] {
  let x = target[0];
  let y = target[1];
  let z = target[2];

  let l1 = robot_configuration.l1;
  let l2 = robot_configuration.l2;
  let l3 = robot_configuration.l3;
  let l4 = robot_configuration.l4;

  let f = f32::sqrt(f32::powi(x, 2) + f32::powi(y, 2) - f32::powi(l1, 2)); // length of shoulder point to target point in xy only
  let g = f - l2; // length to reach the point in xy
  let h = f32::sqrt(f32::powi(g, 2) + f32::powi(z, 2)); // length to reach the points in xyz

  let d = (f32::powi(h, 2) - f32::powi(l3, 2) - f32::powi(l4, 2)) / (2. * l3 * l4);
  let theta1 = -(y).atan2(x) - f.atan2(-l1);
  let theta3 = f32::acos(d);
  let theta2 = z.atan2(g) - (l4 * f32::sin(theta3)).atan2(l3 + l4 * f32::cos(theta3));

  [theta1, theta2, theta3]
}

pub fn leg_fk(theta: [f32; 3], robot_configuration: &RobotConfiguration) -> [Vector4<f32>; 5] {
  let theta1 = theta[0];
  let theta2 = theta[1];
  let theta3 = theta[2];
  let theta23 = theta2 + theta3;

  let l1 = robot_configuration.l1;
  let l2 = robot_configuration.l2;
  let l3 = robot_configuration.l3;
  let l4 = robot_configuration.l4;

  let t0 = Vector4::new(0., 0., 0., 1.);
  let t1 = t0 + Vector4::new(-l1 * f32::cos(theta1), l1 * f32::sin(theta1), 0., 0.);
  let t2 = t1 + Vector4::new(-l2 * f32::sin(theta1), -l2 * f32::cos(theta1), 0., 0.);
  let t3 = t2
    + Vector4::new(
      -l3 * f32::sin(theta1) * f32::cos(theta2),
      -l3 * f32::cos(theta1) * f32::cos(theta2),
      l3 * f32::sin(theta2),
      0.,
    );
  let t4 = t3
    + Vector4::new(
      -l4 * f32::sin(theta1) * f32::cos(theta23),
      -l4 * f32::cos(theta1) * f32::cos(theta23),
      l4 * f32::sin(theta23),
      0.,
    );

  [t0, t1, t2, t3, t4]
}
