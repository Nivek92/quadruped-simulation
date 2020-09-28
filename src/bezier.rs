use nalgebra::base::Vector3;

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

fn bezier(t: f32, points: &Vec<Vector3<f32>>) -> Vector3<f32> {
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

  let mut res = Vector3::zeros();

  res[0] = bez(0);
  res[1] = bez(1);
  res[2] = bez(2);

  res
}

pub fn sample_bezier(sample_size: u16, points: &Vec<Vector3<f32>>) -> Vec<Vector3<f32>> {
  let sample_rate = 1. / sample_size as f32;
  let mut samples = vec![];

  for i in 0..sample_size {
    let sample = bezier(i as f32 * sample_rate, &points);
    samples.push(sample);
  }

  samples
}

pub fn generate_bezier_segments() -> Vec<Vec<Vector3<f32>>> {
  let p0 = Vector3::new(60.5, -200., -64.);
  let p1 = Vector3::new(60.5, -200., -64.);
  let p2 = Vector3::new(60.5, -160., -90.);
  let p3 = Vector3::new(60.5, -160., -90.);
  let p4 = Vector3::new(60.5, -160., -90.);
  let p5 = Vector3::new(60.5, -160., 0.);
  let p6 = Vector3::new(60.5, -160., 0.);
  let p7 = Vector3::new(60.5, -140., 0.);
  let p8 = Vector3::new(60.5, -140., 90.);
  let p9 = Vector3::new(60.5, -140., 90.);
  let p10 = Vector3::new(60.5, -200., 64.);
  let p11 = Vector3::new(60.5, -200., 64.);
  vec![vec![p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11]]
}