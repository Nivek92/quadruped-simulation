use kiss3d::light::Light;
use kiss3d::window::Window;

use nalgebra::geometry::Point3;

use lib::Quadruped;

fn main() {
  let mut window = Window::new("Simulation");
  window.set_framerate_limit(Some(60));
  window.set_light(Light::StickToCamera);
  window.set_point_size(3.0);

  let mut quadruped = Quadruped::new(207.5, 78.0, 60.5, 10., 107., 118.5);
  quadruped.walk();

  let red = Point3::new(1., 0., 0.);
  let green = Point3::new(0., 1., 0.);
  let blue = Point3::new(0., 0., 1.);

  while window.render() {
    let [body, leg_lf, leg_rf, leg_lb, leg_rb] = quadruped.get_coordinates();

    // draw frame

    for i in 0..body.len() {
      let x = body[i];
      window.draw_point(&Point3::new(x[0], x[1], x[2]), &red);
    }

    window.draw_line(
      &Point3::new(body[0][0], body[0][1], body[0][2]),
      &Point3::new(body[1][0], body[1][1], body[1][2]),
      &blue,
    );

    window.draw_line(
      &Point3::new(body[1][0], body[1][1], body[1][2]),
      &Point3::new(body[3][0], body[3][1], body[3][2]),
      &blue,
    );

    window.draw_line(
      &Point3::new(body[3][0], body[3][1], body[3][2]),
      &Point3::new(body[2][0], body[2][1], body[2][2]),
      &blue,
    );

    window.draw_line(
      &Point3::new(body[2][0], body[2][1], body[2][2]),
      &Point3::new(body[0][0], body[0][1], body[0][2]),
      &blue,
    );

    // draw lf

    for i in 0..leg_lf.len() - 1 {
      let p1 = Point3::new(leg_lf[i][0], leg_lf[i][1], leg_lf[i][2]);
      let p2 = Point3::new(leg_lf[i + 1][0], leg_lf[i + 1][1], leg_lf[i + 1][2]);
      window.draw_point(&p1, &green);
      window.draw_line(&p1, &p2, &green);
    }

    // draw rf

    for i in 0..leg_rf.len() - 1 {
      let p1 = Point3::new(leg_rf[i][0], leg_rf[i][1], leg_rf[i][2]);
      let p2 = Point3::new(leg_rf[i + 1][0], leg_rf[i + 1][1], leg_rf[i + 1][2]);
      window.draw_point(&p1, &green);
      window.draw_line(&p1, &p2, &green);
    }

    // draw lb

    for i in 0..leg_lb.len() - 1 {
      let p1 = Point3::new(leg_lb[i][0], leg_lb[i][1], leg_lb[i][2]);
      let p2 = Point3::new(leg_lb[i + 1][0], leg_lb[i + 1][1], leg_lb[i + 1][2]);
      window.draw_point(&p1, &green);
      window.draw_line(&p1, &p2, &green);
    }

    // draw rb

    for i in 0..leg_rb.len() - 1 {
      let p1 = Point3::new(leg_rb[i][0], leg_rb[i][1], leg_rb[i][2]);
      let p2 = Point3::new(leg_rb[i + 1][0], leg_rb[i + 1][1], leg_rb[i + 1][2]);
      window.draw_point(&p1, &green);
      window.draw_line(&p1, &p2, &green);
    }
  }
}
