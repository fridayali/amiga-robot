#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

from farm_ng.canbus.tool_control_pb2 import (
    ActuatorCommands,
    HBridgeCommand,
    HBridgeCommandType,
)
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file


class HBridgeControlNode(Node):
    def __init__(self):
        super().__init__("hbridge_control_node")

        self.current_command = 0

        self.create_subscription(
            Int8,
            "/hbridge",
            self.hbridge_callback,
            10
        )

        self.get_logger().info("Listening /hbridge")

    def hbridge_callback(self, msg):
        self.current_command = int(msg.data)
        self.get_logger().info(
            f"Received command: {self.current_command}"
        )


async def control_hbridge(ros_node, config_path: str):

    config: EventServiceConfig = proto_from_json_file(
        Path(config_path),
        EventServiceConfig()
    )

    client = EventClient(config)

    while True:

        commands = ActuatorCommands()

        if ros_node.current_command == 1:

            commands.hbridges.append(
                HBridgeCommand(
                    id=0,
                    command=HBridgeCommandType.HBRIDGE_FORWARD
                )
            )

            print("FORWARD")

        elif ros_node.current_command == -1:

            commands.hbridges.append(
                HBridgeCommand(
                    id=0,
                    command=HBridgeCommandType.HBRIDGE_REVERSE
                )
            )

            print("REVERSE")

        else:

            commands.hbridges.append(
                HBridgeCommand(
                    id=0,
                    command=HBridgeCommandType.HBRIDGE_STOPPED
                )
            )

            print("STOP")

        try:
            await asyncio.wait_for(
                client.request_reply("/control_tools", commands, decode=True),
                timeout=0.08,
            )
        except (asyncio.TimeoutError, Exception):
            pass

        await asyncio.sleep(0.1)


def ros_spin(node):
    rclpy.spin(node)


async def main_async():

    rclpy.init()

    node = HBridgeControlNode()

    spin_thread = threading.Thread(
        target=ros_spin,
        args=(node,),
        daemon=True
    )

    spin_thread.start()

    import sys
    config_path = sys.argv[1] if len(sys.argv) > 1 else \
        "/ros2_amiga_ws/src/ros2_bridge/config/tool_control.json"
    await control_hbridge(node, config_path)


if __name__ == "__main__":

    try:
        asyncio.run(main_async())

    except KeyboardInterrupt:
        pass