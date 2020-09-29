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

pub fn sample_bezier(sample_size: u16, segments: &Vec<Vec<Vector3<f32>>>) -> Vec<Vector4<f32>> {
  let sample_size = f32::ceil(sample_size as f32 / segments.len() as f32);
  let sample_rate = 1. / sample_size;
  let mut samples = vec![];

  for points in segments {
    for i in 0..sample_size as usize {
      let sample = bezier(i as f32 * sample_rate, &points);
      samples.push(sample);
    }
  }

  samples
}

pub fn generate_bezier_segments() -> Vec<Vec<Vector3<f32>>> {
  let p0 = Vector3::new(-63.5, -200., 0.);
  let p1 = Vector3::new(-83.75, -200., 0.);
  let p2 = Vector3::new(-103.75, -160., 0.);
  let p3 = Vector3::new(-103.75, -160., 0.);
  let p4 = Vector3::new(-103.75, -160., 0.);
  let p5 = Vector3::new(0., -160., 0.);
  let p6 = Vector3::new(0., -160., 0.);
  let p7 = Vector3::new(0., -140., 0.);
  let p8 = Vector3::new(103.75, -140., 0.);
  let p9 = Vector3::new(103.75, -140., 0.);
  let p10 = Vector3::new(83.75, -200., 0.);
  let p11 = Vector3::new(63.5, -200., 0.);

  vec![vec![p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11]]
}

pub fn swing_phase(
  t: f32,
  step_size: f32,
  clearance_height: f32,
  velocity: f32,
  lateral_fraction: f32,
) -> Vector4<f32> {
  let step_size = step_size / 2.;
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

pub fn stance_phase(
  t: f32,
  step_size: f32,
  penetration_depth: f32,
  velocity: f32,
  lateral_fraction: f32,
) -> Vector4<f32> {
  let c = f32::cos(lateral_fraction);
  let s = f32::sin(lateral_fraction);

  let x = c * step_size * (1. - 2. * t) * velocity;
  let z = s * step_size * (1. - 2. * t) * velocity;
  let y = -penetration_depth * f32::cos(std::f32::consts::PI * (x + z) / 2. * step_size);

  Vector4::new(x, y, z, 1.)
}
