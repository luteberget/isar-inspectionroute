use eframe::egui::{Visuals, plot::LinkedAxisGroup};

mod data;
mod app;
mod backend;
mod gui;
mod planner;
mod dvrp;

fn main() {
    eframe::run_native("isar_inspectionroute interactive", eframe::NativeOptions::default(), Box::new(|ctx| {
        ctx.egui_ctx.set_visuals(Visuals::light());
        Box::new(app::WaypointsApp {
            pois: Default::default(),
            planner: planner::Planner::new(),
            parse_poi_window: None,
            connect_to_backend_window: None,
            backend: None,
            dock: None,
            axis_link: (0,LinkedAxisGroup::x()),
        })
    }));
}
