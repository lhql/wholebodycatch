"""RViz 3D end-effector goal tracked jointly by the vehicle and manipulator."""
from astra_wall_control.launching import scenario


def generate_launch_description():
    return scenario('wholebody')
