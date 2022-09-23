use crate::planner::{PlanJob, PoiSequence};

/// Heuristically solve a distance-constrained
/// vehicle routing problem with specified start
/// state that is not the same as the depot state.
pub fn solve(planjob: PlanJob) -> PoiSequence {
    println!("DVRP solving {} pois", planjob.pois.len());
    // Temporarily, planner just does everything in one sequence.
    // integrate_plan(vec![planjob.pois], planjob.robot_state, planjob.dock)
    vec![planjob.pois]
}
