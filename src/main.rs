use kiss3d::light::Light;
use kiss3d::window::Window;

use nalgebra::base::{Matrix4, Vector4};
use nalgebra::geometry::Point3;

use quadruped_simulation::bezier::{generate_bezier_segments, sample_bezier};
use quadruped_simulation::geometry::{body_ik, leg_fk, leg_ik, RobotConfiguration};

fn step(
  body_orientation: [f32; 3],
  body_position: [f32; 3],
  targets: [Vector4<f32>; 4],
  robot_configuration: &RobotConfiguration,
) -> [Vec<Vector4<f32>>; 5] {
  let fp = Vector4::new(0., 0., 0., 1.);

  let ix = Matrix4::new(
    -1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
  );

  let [tlf, trf, tlb, trb, _tm] = body_ik(body_orientation, body_position, robot_configuration);

  let cp: Vec<Vector4<f32>> = vec![tlf, trf, tlb, trb].iter().map(|x| x * fp).collect();

  // lf
  let q = tlf.try_inverse().unwrap() * targets[0];
  let ik = leg_ik(q, robot_configuration);
  let ik_real = vec![
    std::f32::consts::FRAC_PI_2 - ik[0],
    std::f32::consts::FRAC_PI_3 - ik[1],
    std::f32::consts::PI - ik[2],
  ];
  println!(
    "leg_ik lf: {}, {}, {} -> {}, {}, {}",
    ik[0].to_degrees(),
    ik[1].to_degrees(),
    ik[2].to_degrees(),
    ik_real[0].to_degrees(),
    ik_real[1].to_degrees(),
    ik_real[2].to_degrees()
  );
  let p_lf: Vec<Vector4<f32>> = leg_fk(ik, &robot_configuration)
    .iter()
    .map(|x| tlf * x)
    .collect();

  // rf
  let q = ix * trf.try_inverse().unwrap() * targets[1];
  let ik = leg_ik(q, &robot_configuration);
  let ik_real = vec![
    std::f32::consts::FRAC_PI_2 + ik[0],
    2. * std::f32::consts::FRAC_PI_3 + ik[1],
    ik[2],
  ];
  println!(
    "leg_ik rf: {}, {}, {} -> {}, {}, {}",
    ik[0].to_degrees(),
    ik[1].to_degrees(),
    ik[2].to_degrees(),
    ik_real[0].to_degrees(),
    ik_real[1].to_degrees(),
    ik_real[2].to_degrees()
  );
  let p_rf: Vec<Vector4<f32>> = leg_fk(ik, &robot_configuration)
    .iter()
    .map(|x| trf * ix * x)
    .collect();

  // lb
  let q = tlb.try_inverse().unwrap() * targets[2];
  let ik = leg_ik(q, &robot_configuration);
  let ik_real = vec![
    std::f32::consts::FRAC_PI_2 + ik[0],
    std::f32::consts::FRAC_PI_3 - ik[1],
    std::f32::consts::PI - ik[2],
  ];
  println!(
    "leg_ik lb: {}, {}, {} -> {}, {}, {}",
    ik[0].to_degrees(),
    ik[1].to_degrees(),
    ik[2].to_degrees(),
    ik_real[0].to_degrees(),
    ik_real[1].to_degrees(),
    ik_real[2].to_degrees()
  );
  let p_lb: Vec<Vector4<f32>> = leg_fk(ik, &robot_configuration)
    .iter()
    .map(|x| tlb * x)
    .collect();

  // rb
  let q = ix * trb.try_inverse().unwrap() * targets[3];
  let ik = leg_ik(q, &robot_configuration);
  let ik_real = vec![
    std::f32::consts::FRAC_PI_2 - ik[0],
    2. * std::f32::consts::FRAC_PI_3 + ik[1],
    ik[2],
  ];
  println!(
    "leg_ik rb: {}, {}, {} -> {}, {}, {}",
    ik[0].to_degrees(),
    ik[1].to_degrees(),
    ik[2].to_degrees(),
    ik_real[0].to_degrees(),
    ik_real[1].to_degrees(),
    ik_real[2].to_degrees()
  );
  let p_rb: Vec<Vector4<f32>> = leg_fk(ik, &robot_configuration)
    .iter()
    .map(|x| trb * ix * x)
    .collect();

  [cp, p_lf, p_rf, p_lb, p_rb]
}

fn main() {
  let mut window = Window::new("Simulation");
  window.set_light(Light::StickToCamera);
  window.set_point_size(3.0);

  let robot_configuration = RobotConfiguration::new(207.5, 78.0, 60.5, 10., 107., 118.5);

  let body_orientation = [std::f32::consts::PI / 16., std::f32::consts::PI / 8., 0.];
  let body_position = [0., 0., 0.];

  let red = Point3::new(1.0, 0.0, 0.0);
  let green = Point3::new(0.0, 1.0, 0.0);
  let blue = Point3::new(0.0, 0.0, 1.0);

  let frequency = 60.; // in Hz
  let duration = 1.25; // in seconds

  let segments = generate_bezier_segments();

  let samples_lf: Vec<Vector4<f32>> = sample_bezier((frequency * 10. * duration) as u16, &segments)
    .iter()
    .map(|x| *x + Vector4::new(robot_configuration.l / 2., 0., 100., 0.))
    .collect();
  let samples_rf: Vec<Vector4<f32>> = sample_bezier((frequency * 10. * duration) as u16, &segments)
    .iter()
    .map(|x| *x + Vector4::new(robot_configuration.l / 2., 0., -100., 0.))
    .collect();
  let samples_lb: Vec<Vector4<f32>> = sample_bezier((frequency * 10. * duration) as u16, &segments)
    .iter()
    .map(|x| *x + Vector4::new(-robot_configuration.l / 2., 0., 100., 0.))
    .collect();
  let samples_rb: Vec<Vector4<f32>> = sample_bezier((frequency * 10. * duration) as u16, &segments)
    .iter()
    .map(|x| *x + Vector4::new(-robot_configuration.l / 2., 0., -100., 0.))
    .collect();

  let mut j = 0;

  while window.render() {
    let targets = [samples_lf[j], samples_rf[j], samples_lb[j], samples_rb[j]];

    // draw target points

    let [cp, p_lf, p_rf, p_lb, p_rb] = step(
      body_orientation,
      body_position,
      targets,
      &robot_configuration,
    );

    for i in 0..targets.len() {
      let x = targets[i];
      window.draw_point(&Point3::new(x[0], x[2], x[1]), &red);
    }

    // draw frame

    for i in 0..cp.len() {
      let x = cp[i];
      window.draw_point(&Point3::new(x[0], x[2], x[1]), &blue);
    }

    window.draw_line(
      &Point3::new(cp[0][0], cp[0][2], cp[0][1]),
      &Point3::new(cp[1][0], cp[1][2], cp[1][1]),
      &blue,
    );

    window.draw_line(
      &Point3::new(cp[1][0], cp[1][2], cp[1][1]),
      &Point3::new(cp[3][0], cp[3][2], cp[3][1]),
      &blue,
    );

    window.draw_line(
      &Point3::new(cp[3][0], cp[3][2], cp[3][1]),
      &Point3::new(cp[2][0], cp[2][2], cp[2][1]),
      &blue,
    );

    window.draw_line(
      &Point3::new(cp[2][0], cp[2][2], cp[2][1]),
      &Point3::new(cp[0][0], cp[0][2], cp[0][1]),
      &blue,
    );

    // draw lf

    for i in 0..p_lf.len() - 1 {
      let p1 = Point3::new(p_lf[i][0], p_lf[i][2], p_lf[i][1]);
      let p2 = Point3::new(p_lf[i + 1][0], p_lf[i + 1][2], p_lf[i + 1][1]);
      window.draw_point(&p1, &green);
      window.draw_line(&p1, &p2, &green);
    }

    // draw rf

    for i in 0..p_rf.len() - 1 {
      let p1 = Point3::new(p_rf[i][0], p_rf[i][2], p_rf[i][1]);
      let p2 = Point3::new(p_rf[i + 1][0], p_rf[i + 1][2], p_rf[i + 1][1]);
      window.draw_point(&p1, &green);
      window.draw_line(&p1, &p2, &green);
    }

    // draw lb

    for i in 0..p_lb.len() - 1 {
      let p1 = Point3::new(p_lb[i][0], p_lb[i][2], p_lb[i][1]);
      let p2 = Point3::new(p_lb[i + 1][0], p_lb[i + 1][2], p_lb[i + 1][1]);
      window.draw_point(&p1, &green);
      window.draw_line(&p1, &p2, &green);
    }

    // draw rb

    for i in 0..p_rb.len() - 1 {
      let p1 = Point3::new(p_rb[i][0], p_rb[i][2], p_rb[i][1]);
      let p2 = Point3::new(p_rb[i + 1][0], p_rb[i + 1][2], p_rb[i + 1][1]);
      window.draw_point(&p1, &green);
      window.draw_line(&p1, &p2, &green);
    }

    // draw trajectory lf

    for i in 0..samples_lf.len() - 1 {
      let p1 = Point3::new(samples_lf[i][0], samples_lf[i][2], samples_lf[i][1]);
      let p2 = Point3::new(
        samples_lf[i + 1][0],
        samples_lf[i + 1][2],
        samples_lf[i + 1][1],
      );
      window.draw_point(&p1, &red);
      window.draw_line(&p1, &p2, &red);
    }

    // draw trajectory rf

    for i in 0..samples_rf.len() - 1 {
      let p1 = Point3::new(samples_rf[i][0], samples_rf[i][2], samples_rf[i][1]);
      let p2 = Point3::new(
        samples_rf[i + 1][0],
        samples_rf[i + 1][2],
        samples_rf[i + 1][1],
      );
      window.draw_point(&p1, &red);
      window.draw_line(&p1, &p2, &red);
    }

    // draw trajectory lb

    for i in 0..samples_lb.len() - 1 {
      let p1 = Point3::new(samples_lb[i][0], samples_lb[i][2], samples_lb[i][1]);
      let p2 = Point3::new(
        samples_lb[i + 1][0],
        samples_lb[i + 1][2],
        samples_lb[i + 1][1],
      );
      window.draw_point(&p1, &red);
      window.draw_line(&p1, &p2, &red);
    }

    // draw trajectory rb

    for i in 0..samples_rb.len() - 1 {
      let p1 = Point3::new(samples_rb[i][0], samples_rb[i][2], samples_rb[i][1]);
      let p2 = Point3::new(
        samples_rb[i + 1][0],
        samples_rb[i + 1][2],
        samples_rb[i + 1][1],
      );
      window.draw_point(&p1, &red);
      window.draw_line(&p1, &p2, &red);
    }

    j = (j + 1) % (samples_lf.len() - 1);
  }
}
