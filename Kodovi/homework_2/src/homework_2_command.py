#!/usr/bin/env python3

import select
import sys
import termios
import tty
from typing import Optional

import rclpy
from rclpy.node import Node

from homework_2.srv import RobotCommand


MODE_MANUAL = 1
MODE_AUTO = 2
MODE_STOP = 3

MANUAL_FORWARD = 1
MANUAL_BACKWARD = 2
MANUAL_LEFT = 3
MANUAL_RIGHT = 4
MANUAL_STOP = 5

CONTROLLER_BASIC = 1
CONTROLLER_REVERSE = 2
CONTROLLER_CONSTANT_SPEED = 3


class Homework2CommandClient(Node):
    def __init__(self) -> None:
        super().__init__("homework_2_command")
        self.client = self.create_client(RobotCommand, "/robot_command_service")

    def wait_for_controller(self) -> bool:
        print("Waiting for /robot_command_service ...")
        while rclpy.ok() and not self.client.wait_for_service(timeout_sec=1.0):
            print("  service is not available yet")
        return rclpy.ok()

    def send_command(
        self,
        mode: int,
        manual_command: int = MANUAL_STOP,
        controller_type: int = CONTROLLER_BASIC,
        goal_x: float = 0.0,
        goal_y: float = 0.0,
        linear_speed: float = 0.0,
        angular_speed: float = 0.0,
    ) -> Optional[RobotCommand.Response]:
        request = RobotCommand.Request()
        request.mode = mode
        request.manual_command = manual_command
        request.controller_type = controller_type
        request.goal_x = goal_x
        request.goal_y = goal_y
        request.linear_speed = linear_speed
        request.angular_speed = angular_speed

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def read_key(timeout: float = 0.1) -> str:
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if readable:
        return sys.stdin.read(1)
    return ""


def print_main_menu() -> None:
    print()
    print("Homework 2 command menu")
    print("1 - manual mode")
    print("2 - automatic goal mode")
    print("3 - stop robot")
    print("q - quit")


def print_manual_help() -> None:
    print()
    print("Manual mode")
    print("w - forward")
    print("s - backward")
    print("a - rotate left")
    print("d - rotate right")
    print("space - stop")
    print("q - return to menu")


def manual_mode(client: Homework2CommandClient) -> None:
    key_to_command = {
        "w": MANUAL_FORWARD,
        "s": MANUAL_BACKWARD,
        "a": MANUAL_LEFT,
        "d": MANUAL_RIGHT,
        " ": MANUAL_STOP,
    }

    print_manual_help()
    terminal_settings = termios.tcgetattr(sys.stdin)
    current_command = MANUAL_STOP
    last_reported_command = None

    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key()
            if key == "\x03":
                raise KeyboardInterrupt
            if key == "q":
                break
            if key in key_to_command:
                current_command = key_to_command[key]

            response = client.send_command(
                mode=MODE_MANUAL,
                manual_command=current_command,
            )

            if response is None:
                print("\rNo response from controller.", end="", flush=True)
            elif not response.success:
                print(f"\r{response.message}", end="", flush=True)
            elif current_command != last_reported_command:
                print(f"\r{response.message:<40}", end="", flush=True)
                last_reported_command = current_command
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        client.send_command(mode=MODE_STOP)
        print()


def read_float(prompt: str) -> float:
    while True:
        raw_value = input(prompt).strip()
        try:
            return float(raw_value)
        except ValueError:
            print("Please enter a valid number.")


def choose_controller() -> int:
    print()
    print("Controller type")
    print("1 - basic closed-loop controller")
    print("2 - reverse-capable controller")
    print("3 - constant-speed reverse-capable controller")

    while True:
        choice = input("Choose controller [1/2/3]: ").strip()
        if choice == "1":
            return CONTROLLER_BASIC
        if choice == "2":
            return CONTROLLER_REVERSE
        if choice == "3":
            return CONTROLLER_CONSTANT_SPEED
        print("Please choose 1, 2, or 3.")


def automatic_mode(client: Homework2CommandClient) -> None:
    goal_x = read_float("Goal x [m]: ")
    goal_y = read_float("Goal y [m]: ")
    controller_type = choose_controller()

    response = client.send_command(
        mode=MODE_AUTO,
        controller_type=controller_type,
        goal_x=goal_x,
        goal_y=goal_y,
    )

    if response is None:
        print("No response from controller.")
    else:
        print(response.message)


def main(args=None) -> None:
    rclpy.init(args=args)
    client = Homework2CommandClient()

    try:
        if not client.wait_for_controller():
            return

        while rclpy.ok():
            print_main_menu()
            choice = input("Choose option: ").strip().lower()

            if choice == "1":
                manual_mode(client)
            elif choice == "2":
                automatic_mode(client)
            elif choice == "3":
                response = client.send_command(mode=MODE_STOP)
                if response is not None:
                    print(response.message)
            elif choice == "q":
                client.send_command(mode=MODE_STOP)
                break
            else:
                print("Unknown option.")
    except KeyboardInterrupt:
        client.send_command(mode=MODE_STOP)
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
