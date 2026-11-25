"""Offline stub standing in for the real prepare_mission module.

The real one analyses the flyover video; this test never wants that to run, and
the gate under test does not depend on it. Only the attributes proposed_solution
touches are provided.
"""
import os

SIM_LOGS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_no_sim_logs")
CSV_OUT = os.path.join(SIM_LOGS, "victim_location_estimates.csv")
NAV_GRID_OUT = os.path.join(SIM_LOGS, "nav_grid.npy")
MAP_OUT = os.path.join(SIM_LOGS, "map_estimate.png")


def find_recording(world_path=None):
    return None, None


def outputs_are_fresh(mp4):
    return True
