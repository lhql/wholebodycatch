"""Automatic takeoff, independent body flight and seven-joint arm commands."""
from astra_wall_control.launching import scenario


def generate_launch_description():
    return scenario('manual')
