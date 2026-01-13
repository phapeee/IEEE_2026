from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="gpio_button_event",
                executable="gpio_button_event_node",
                name="gpio_button_event",
                output="screen",
                parameters=[
                    {
                        "backend": "auto",
                        "gpio_chip": "gpiochip0",
                        "gpio_line": 4,
                        "active_low": False,
                        "use_internal_pullup": False,
                        "polling_frequency_hz": 50.0,
                        "debounce_duration_ms": 30.0,
                        "smacc_event_topic": "smacc2/button_event",
                        "button_state_topic": "smacc2/button_state",
                        "event_object_tag": "GPIO4Button",
                        "event_source": "gpio_button_event.launch",
                        "pressed_event_type": "BUTTON_PRESSED",
                        "released_event_type": "BUTTON_RELEASED",
                        "switches": [],
                    }
                ],
            )
        ]
    )
