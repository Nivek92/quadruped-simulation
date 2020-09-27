#![feature(clamp)]

use kiss3d;
use kiss3d::light::Light;
use kiss3d::window::Window;

use nalgebra::base::Vector3;
use nalgebra::geometry::Point3;

use bezier::{generate_bezier_segments, sample_bezier};

fn main() {
    let mut window = Window::new("Simulation");
    window.set_light(Light::StickToCamera);
    window.set_point_size(10.0);

    let segments = generate_bezier_segments();

    let samples: Vec<Vector3<f32>> = segments
        .into_iter()
        .map(|segment| sample_bezier(60 * 4, &segment))
        .flatten()
        .collect();

    let points: Vec<Point3<f32>> = (&samples)
        .into_iter()
        .map(|p| Point3::new(p[0], p[1], p[2]))
        .collect();

    while window.render() {
        for i in 0..points.len() - 1 {
            window.draw_line(&points[i], &points[i + 1], &Point3::new(1.0, 0.0, 0.0));
        }
    }
}
