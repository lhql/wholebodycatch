"""Static safe-corridor navigation with the complete aerial manipulator envelope."""
from astra_wall_control.launching import scenario


def generate_launch_description():
    return scenario('corridor')
