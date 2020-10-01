#![feature(clamp)]
use nalgebra::base::{Matrix4, RowVector4, Vector3, Vector4};

fn generate_binomial_coefficients(n: usize) -> Vec<u16> {
  if n == 0 {
    return vec![1];
  } else if n == 1 {
    return vec![1, 1];
  } else {
    let elements = generate_binomial_coefficients(n - 1);

    let mut res = vec![1];

    for i in 0..elements.len() - 1 {
      res.push(elements[i] + elements[i + 1]);
    }

    res.push(1);

    res
  }
}

pub fn bezier(t: f32, points: &Vec<Vector3<f32>>) -> Vector4<f32> {
  let n = points.len();

  let t = f32::clamp(t, 0., 1.);
  let nt = 1. - t;

  let coefficients = generate_binomial_coefficients(n - 1);

  let bez = |idx: usize| {
    let mut sum = 0.;
    for i in 0..n {
      let k = n as i32 - 1;
      let j = i as i32;
      sum += coefficients[i] as f32 * f32::powi(nt, k - j) * f32::powi(t, j) * points[i][idx];
    }

    sum
  };

  let mut res = Vector4::zeros();

  res[0] = bez(0);
  res[1] = bez(1);
  res[2] = bez(2);
  res[3] = 1.;

  res
}

fn swing_phase(
  t: f32,
  step_size: f32,
  velocity: f32,
  lateral_fraction: f32,
  clearance_height: f32,
) -> Vector4<f32> {
  let c = f32::cos(lateral_fraction);
  let s = f32::sin(lateral_fraction);

  let h = [
    -step_size, // Ctrl Point 0, half of stride len
    -step_size * 1.4,
    -step_size * 1.5,
    -step_size * 1.5,
    -step_size * 1.5,
    0.,
    0.,
    0.,
    step_size * 1.5,
    step_size * 1.5,
    step_size * 1.4,
    step_size,
  ];

  let v = [
    0.,
    0.,
    clearance_height * 0.9,
    clearance_height * 0.9,
    clearance_height * 0.9,
    clearance_height * 0.9,
    clearance_height * 0.9,
    clearance_height * 1.1,
    clearance_height * 1.1,
    clearance_height * 1.1,
    0.,
    0.,
  ];

  let mut points: Vec<Vector3<f32>> = vec![];

  for i in 0..12 {
    points.push(Vector3::new(
      c * velocity * h[i],
      velocity * v[i],
      s * velocity * h[i],
    ));
  }

  bezier(t, &points)
}

fn stance_phase(
  t: f32,
  step_size: f32,
  velocity: f32,
  lateral_fraction: f32,
  penetration_depth: f32,
) -> Vector4<f32> {
  let c = f32::cos(lateral_fraction);
  let s = f32::sin(lateral_fraction);
  let step = step_size * (1. - 2. * t);
  let x = c * step * velocity;
  let z = s * step * velocity;
  let y = -penetration_depth * f32::cos((std::f32::consts::PI * (x + z)) / (2. * step_size));

  Vector4::new(x, y, z, 1.)
}

pub fn trajectory(
  t: f32,
  step_size: f32,
  velocity: f32,
  lateral_fraction: f32,
  clearance_height: f32,
  penetration_depth: f32,
  swing_duration: f32,
) -> Vector4<f32> {
  let t = f32::clamp(t, 0., 1.);

  if t > swing_duration {
    return stance_phase(
      (t - swing_duration) / (1. - swing_duration), // scale t to stance duration
      step_size,
      velocity,
      lateral_fraction,
      penetration_depth,
    );
  }

  swing_phase(
    t / swing_duration, // scale t to swing duration
    step_size,
    velocity,
    lateral_fraction,
    clearance_height,
  )
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

enum Gait {
  Idle,
  Walk,
  Trot,
}

impl Gait {
  fn offsets(&self) -> [f32; 4] {
    match self {
      Gait::Idle => [0., 0., 0., 0.],
      Gait::Walk => [0., 0.5, 0.25, 0.75],
      Gait::Trot => [0., 0., 0.5, 0.5],
    }
  }
}

pub struct Quadruped {
  timer: [f32; 4],
  gait: Gait,
  velocity: f32,
  cycle_duration: f32,
  l: f32,
  w: f32,
  l1: f32,
  l2: f32,
  l3: f32,
  l4: f32,
  update_rate: f32,
  body_orientation: [f32; 3],
  body_position: [f32; 3],
  thetas: [[f32; 3]; 4],
}

impl Quadruped {
  pub fn new(l: f32, w: f32, l1: f32, l2: f32, l3: f32, l4: f32) -> Self {
    Quadruped {
      timer: [0., 0., 0., 0.],
      gait: Gait::Idle,
      velocity: 0.,
      cycle_duration: 1.5,
      l,
      w,
      l1,
      l2,
      l3,
      l4,
      update_rate: 60.,
      body_orientation: [0., 0., 0.],
      body_position: [0., 0., 0.],
      thetas: [[0.; 3]; 4],
    }
  }

  fn get_targets(&mut self) -> [Vector4<f32>; 4] {
    let [t_lf, t_rf, t_lb, t_rb] = self.timer;
    let l = self.l;

    let target_lf =
      trajectory(t_lf, 65., 1., 0., 50., 10., 0.5) + Vector4::new(l / 2., -200., 100., 0.);
    let target_rf =
      trajectory(t_rf, 65., 1., 0., 50., 10., 0.5) + Vector4::new(l / 2., -200., -100., 0.);
    let target_lb =
      trajectory(t_lb, 65., 1., 0., 50., 10., 0.5) + Vector4::new(-l / 2., -200., 100., 0.);
    let target_rb =
      trajectory(t_rb, 65., 1., 0., 50., 10., 0.5) + Vector4::new(-l / 2., -200., -100., 0.);

    [target_lf, target_rf, target_lb, target_rb]
  }

  fn change_gait(&mut self, gait: Gait) {
    self.timer = gait.offsets();
    self.gait = gait;
  }

  fn shoulder_transformations(&self) -> [Matrix4<f32>; 4] {
    let l = self.l;
    let w = self.w;

    let tm = rt_matrix(self.body_orientation, self.body_position);
    let tlf = tm * rt_matrix([0., std::f32::consts::FRAC_PI_2, 0.], [l / 2., 0., w / 2.]);
    let trf = tm * rt_matrix([0., std::f32::consts::FRAC_PI_2, 0.], [l / 2., 0., -w / 2.]);
    let tlb = tm * rt_matrix([0., std::f32::consts::FRAC_PI_2, 0.], [-l / 2., 0., w / 2.]);
    let trb = tm
      * rt_matrix(
        [0., std::f32::consts::FRAC_PI_2, 0.],
        [-l / 2., 0., -w / 2.],
      );

    [tlf, trf, tlb, trb]
  }

  fn leg_ik(&self, target: Vector4<f32>) -> [f32; 3] {
    let x = target[0];
    let y = target[1];
    let z = target[2];

    let l1 = self.l1;
    let l2 = self.l2;
    let l3 = self.l3;
    let l4 = self.l4;

    let f = f32::sqrt(f32::powi(x, 2) + f32::powi(y, 2) - f32::powi(l1, 2)); // length of shoulder point to target point in xy only
    let g = f - l2; // length to reach the point in xy
    let h = f32::sqrt(f32::powi(g, 2) + f32::powi(z, 2)); // length to reach the points in xyz

    let d = (f32::powi(h, 2) - f32::powi(l3, 2) - f32::powi(l4, 2)) / (2. * l3 * l4);
    let theta1 = -(y).atan2(x) - f.atan2(-l1);
    let theta3 = f32::acos(d);
    let theta2 = z.atan2(g) - (l4 * f32::sin(theta3)).atan2(l3 + l4 * f32::cos(theta3));

    [theta1, theta2, theta3]
  }

  fn legs_ik(&self, targets: [Vector4<f32>; 4]) -> [[f32; 3]; 4] {
    let mut angles = [[0.; 3]; 4];

    for i in 0..4 {
      angles[i] = self.leg_ik(targets[i]);
    }

    angles
  }

  fn leg_fk(&self, theta: [f32; 3]) -> [Vector4<f32>; 5] {
    let theta1 = theta[0];
    let theta2 = theta[1];
    let theta3 = theta[2];
    let theta23 = theta2 + theta3;

    let l1 = self.l1;
    let l2 = self.l2;
    let l3 = self.l3;
    let l4 = self.l4;

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

    [t0, t1, t2, t3, t4] // shoulder, coxa, femur, tibia, foot tip
  }

  fn legs_fk(&self) -> [[Vector4<f32>; 5]; 4] {
    let mut legs = [[Vector4::zeros(); 5]; 4];

    for i in 0..4 {
      legs[i] = self.leg_fk(self.thetas[i]);
    }

    legs
  }

  fn update_timer(&mut self) {
    self.timer[0] += 1. / self.update_rate / self.cycle_duration;
    self.timer[1] += 1. / self.update_rate / self.cycle_duration;
    self.timer[2] += 1. / self.update_rate / self.cycle_duration;
    self.timer[3] += 1. / self.update_rate / self.cycle_duration;

    if self.timer[0] > 1. {
      self.timer[0] = 0.;
    }
    if self.timer[1] > 1. {
      self.timer[1] = 0.;
    }
    if self.timer[2] > 1. {
      self.timer[2] = 0.;
    }
    if self.timer[3] > 1. {
      self.timer[3] = 0.;
    }
  }

  fn update_legs(&mut self) {
    match self.gait {
      Gait::Idle => (),
      _ => {
        // invert x
        let inv_x = Matrix4::new(
          -1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
        );
        let mut targets = self.get_targets();
        let [tlf, trf, tlb, trb] = self.shoulder_transformations();

        targets[0] = tlf.try_inverse().unwrap() * targets[0];
        targets[1] = inv_x * trf.try_inverse().unwrap() * targets[1];
        targets[2] = tlb.try_inverse().unwrap() * targets[2];
        targets[3] = inv_x * trb.try_inverse().unwrap() * targets[3];

        let angles = self.legs_ik(targets);

        self.thetas = angles;
      }
    }
  }

  pub fn set_velocity(&mut self, velocity: f32) {
    self.velocity = velocity;
  }

  pub fn set_body_orientation(&mut self, orientation: [f32; 3]) {
    self.body_orientation = orientation;
  }

  pub fn set_body_position(&mut self, position: [f32; 3]) {
    self.body_position = position;
  }

  pub fn update(&mut self) -> [Vec<Vector4<f32>>; 5] {
    let fp = Vector4::new(0., 0., 0., 1.);

    // invert x
    let inv_x = Matrix4::new(
      -1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
    );

    self.update_legs();
    self.update_timer();

    let [tlf, trf, tlb, trb] = self.shoulder_transformations();

    let body: Vec<Vector4<f32>> = vec![tlf, trf, tlb, trb].iter().map(|x| x * fp).collect();

    let leg_lf: Vec<Vector4<f32>> = self
      .leg_fk(self.thetas[0])
      .iter()
      .map(|x| tlf * x)
      .collect();

    let leg_rf: Vec<Vector4<f32>> = self
      .leg_fk(self.thetas[1])
      .iter()
      .map(|x| trf * inv_x * x)
      .collect();

    let leg_lb: Vec<Vector4<f32>> = self
      .leg_fk(self.thetas[2])
      .iter()
      .map(|x| tlb * x)
      .collect();

    let leg_rb: Vec<Vector4<f32>> = self
      .leg_fk(self.thetas[3])
      .iter()
      .map(|x| trb * inv_x * x)
      .collect();

    [body, leg_lf, leg_rf, leg_lb, leg_rb]
  }

  pub fn walk(&mut self) {
    self.change_gait(Gait::Walk);
  }

  pub fn trot(&mut self) {
    self.change_gait(Gait::Trot);
  }
}
