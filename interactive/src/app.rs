use eframe::egui::plot::LinkedAxisGroup;

use crate::{
    backend::{isar::IsarConnectionBuilder, Backend},
    data::Poi,
};

pub struct ParsePoiWindow {
    pub input: String,
    pub parse_result: Result<Vec<crate::data::Poi>, serde_json::Error>,
}

impl ParsePoiWindow {
    pub fn from_string(input: String) -> Self {
        // Try parsing a list
        let parse_result = serde_json::from_str::<Vec<crate::data::Poi>>(&input)
            .or_else(|_err| serde_json::from_str::<crate::data::Poi>(&input).map(|x| vec![x]));

        ParsePoiWindow {
            input,
            parse_result,
        }
    }
}

pub struct PendingPoi {
    pub poi: Poi,
    pub active: bool,
}

pub struct FinishedPoi {
    pub poi: Poi,
    pub success: bool,
}

pub struct WaypointsApp {
    pub backend: Option<Box<dyn Backend>>,
    pub pending_pois: Vec<PendingPoi>,
    pub finished_pois: Vec<FinishedPoi>,
    pub dock: Option<Poi>,
    pub planner: crate::planner::Planner,
    pub parse_poi_window: Option<ParsePoiWindow>,
    pub connect_to_backend_window: Option<IsarConnectionBuilder>,
    pub axis_link: (usize, LinkedAxisGroup),
}

impl eframe::App for crate::app::WaypointsApp {
    fn update(&mut self, ctx: &eframe::egui::Context, _frame: &mut eframe::Frame) {
        if let Some(state) = self.connect_to_backend_window.as_mut() {
            state.process_messages();
        }

        self.planner.process_messages();

        if let Some(backend) = self.backend.as_mut() {
            // Send events to planner.
            while let Some((_t, ev)) = backend.try_recv_event() {
                match &ev {
                    crate::backend::Event::TaskEnd(poi_name, success) => {
                        while let Some(idx) = self
                            .pending_pois
                            .iter()
                            .position(|p| &p.poi.name == poi_name)
                        {
                            let PendingPoi { poi, .. } = self.pending_pois.remove(idx);
                            self.finished_pois.push(FinishedPoi {
                                poi,
                                success: *success,
                            });
                        }
                    }
                    crate::backend::Event::TaskStart(poi_name) => {
                        for pending in self.pending_pois.iter_mut() {
                            if &pending.poi.name == poi_name {
                                pending.active = true;
                            }
                        }
                    }
                }

                self.planner.update_event(&ev);
            }
            // Send state to planner.
            self.planner.update_state(
                &self.pending_pois,
                &self.dock,
                backend.current_robot_state(),
            );

            // Send planned sequence to backend.
            if let (counter, Some(seq)) = self.planner.get_plan_sequence() {
                backend.set_plan(counter, seq, &self.dock);
            }
        }

        crate::gui::draw_gui(self, ctx);
    }
}
