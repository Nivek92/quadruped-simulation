use nalgebra::base::{Vector3, Vector4};

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
