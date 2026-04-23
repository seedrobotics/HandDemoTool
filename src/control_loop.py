#!/usr/bin/env python3
import platform
import argparse
import time
from typing import Dict, List, Tuple
import numpy as np
import yaml
from dynamixel_sdk import (
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)

TORQUE_ON_ADDRESS = 24
GOAL_POSITION_ADDRESS = 30
ADDR_PRESENT_POSITION = 36
LEN_PRESENT_POSITION = 2

ADDR_PALM_IR_SENSOR = 94
LEN_PALM_IR_SENSOR = 2

ADDR_CURRENT = 68
LEN_CURRENT = 2
ADDR_TEMPERATURE = 43
LEN_TEMPERATURE = 1

PROTOCOL_VERSION = 2.0


def _load_config(path: str, is_left) -> Tuple[List[int], str, int, int]:
    with open(path, "r") as f:
        config = yaml.safe_load(f)

    system = platform.system()
    if system == "Windows":
        port = config["control_port_windows"]
    elif system == "Linux":
        port = config["control_port_linux"]

    params = config
    if isinstance(config, dict) and "hand_handle_node" in config:
        params = config.get("hand_handle_node", {}).get("ros__parameters", {})

    if is_left:
        joint_mapping = params.get("l_joint_mapping", {})
    else:
        joint_mapping = params.get("r_joint_mapping", {})
    if not joint_mapping:
        raise ValueError("joint_mapping not found in config")


    joint_ids = list(joint_mapping.values())

    if not port:
        raise ValueError("control_port/port not found in config")

    baudrate = int(params.get("baudrate", 1000000))
    if is_left:
        main_board = (params.get("l_mainboard"))
    else:
        main_board = (params.get("r_mainboard"))
    return joint_ids, port, baudrate, int(main_board)


def _int_to_2bytes(value: int) -> List[int]:
    return [
        DXL_LOBYTE(value),
        DXL_HIBYTE(value),
    ]


def _default_positions(joint_count: int) -> List[List[int]]:
    mid = 2048
    return [
        [mid for _ in range(joint_count)],
        [1024 for _ in range(joint_count)],
        [3072 for _ in range(joint_count)],
        [mid for _ in range(joint_count)],
    ]


def _normalize_cycle_times(times: List[float], count: int) -> List[float]:
    if not times:
        return [0.5 for _ in range(count)]
    if len(times) >= count:
        return times[:count]
    return times + [times[-1] for _ in range(count - len(times))]


class HandController:
    def __init__(self, config_path: str, is_left) -> None:
        joint_ids, port_name, baudrate, main_board_id = _load_config(config_path, is_left)
        self.joint_ids = joint_ids
        self.main_board_id = main_board_id

        self.port = PortHandler(port_name)
        self.packet = PacketHandler(PROTOCOL_VERSION)

        if not self.port.openPort():
            raise SystemExit(f"Failed to open port {port_name}")
        if not self.port.setBaudRate(baudrate):
            self.port.closePort()
            raise SystemExit(f"Failed to set baudrate {baudrate}")

        data = []
        for x in range(20):
            st= time.time()
            val = self.packet.read1ByteTxRx(self.port,self.main_board_id,2)
            fn=time.time()
            data.append(fn-st)
        median_value=np.median(data)*1000
        print(f"Median USB Latency is :{median_value}ms")

        if median_value > 3:
            print("Latency test of USB port shows high Latency. Please check Readme or run set_usb_latency.sh as root")
        else:
            print("USB latency is OK") 

        self.group_write = GroupSyncWrite(self.port, self.packet, GOAL_POSITION_ADDRESS, 2)
        self.group_pos_read = GroupSyncRead(
            self.port, self.packet, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
        )
        self.group_current_read = GroupSyncRead(
            self.port, self.packet, ADDR_CURRENT, LEN_CURRENT
        )
        self.group_temp_read = GroupSyncRead(
            self.port, self.packet, ADDR_TEMPERATURE, LEN_TEMPERATURE
        )

        for joint_id in self.joint_ids:
            if not self.group_pos_read.addParam(joint_id):
                raise SystemExit(f"Failed to add ID {joint_id} to position sync read")
            if not self.group_current_read.addParam(joint_id):
                raise SystemExit(f"Failed to add ID {joint_id} to current sync read")
            if not self.group_temp_read.addParam(joint_id):
                raise SystemExit(f"Failed to add ID {joint_id} to temp sync read")

    def enable_torque(self) -> None:
        for joint_id in self.joint_ids:
            result, error = self.packet.write1ByteTxRx(
                self.port, joint_id, TORQUE_ON_ADDRESS, 1
            )
            if result != COMM_SUCCESS:
                print(
                    f"Torque enable error (ID {joint_id}): "
                    f"{self.packet.getTxRxResult(result)}"
                )
            elif error != 0:
                print(
                    f"Torque enable error (ID {joint_id}): "
                    f"{self.packet.getRxPacketError(error)}"
                )

    def disable_torque(self) -> None:
        for joint_id in self.joint_ids:
            self.packet.write1ByteTxRx(self.port, joint_id, TORQUE_ON_ADDRESS, 0)

    def close(self) -> None:
        self.port.closePort()

    def write_and_read_cycle(self, targets: List[int]) -> Dict[str, List[int] | int]:
        self.group_write.clearParam()
        for joint_id, target in zip(self.joint_ids, targets):
            data = _int_to_2bytes(int(target))
            if not self.group_write.addParam(joint_id, data):
                print(f"Failed to add ID {joint_id} to sync write")
        result = self.group_write.txPacket()
        if result != COMM_SUCCESS:
            print(f"Sync write error: {self.packet.getTxRxResult(result)}")

        result = self.group_pos_read.txRxPacket()
        if result != COMM_SUCCESS:
            print(f"Sync read position error: {self.packet.getTxRxResult(result)}")

        result = self.group_current_read.txRxPacket()
        if result != COMM_SUCCESS:
            print(f"Sync read current error: {self.packet.getTxRxResult(result)}")

        result = self.group_temp_read.txRxPacket()
        if result != COMM_SUCCESS:
            print(f"Sync read temp error: {self.packet.getTxRxResult(result)}")

        ir_value, ir_result, ir_error = self.packet.read2ByteTxRx(
            self.port, self.main_board_id, ADDR_PALM_IR_SENSOR
        )
        if ir_result != COMM_SUCCESS:
            print(f"IR read error: {self.packet.getTxRxResult(ir_result)}")
        elif ir_error != 0:
            print(f"IR packet error: {self.packet.getRxPacketError(ir_error)}")

        present_positions = [
            self.group_pos_read.getData(
                joint_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
            for joint_id in self.joint_ids
        ]
        currents = [
            self.group_current_read.getData(joint_id, ADDR_CURRENT, LEN_CURRENT)
            for joint_id in self.joint_ids
        ]
        temperatures = [
            self.group_temp_read.getData(joint_id, ADDR_TEMPERATURE, LEN_TEMPERATURE)
            for joint_id in self.joint_ids
        ]

        return {
            "present_positions": present_positions,
            "currents": currents,
            "temperatures": temperatures,
            "ir_value": ir_value,
        }


def main() -> None:
    parser = argparse.ArgumentParser(description="Control loop test for RH8D hand")
    parser.add_argument("--config", default="RH8D_L.yaml", help="YAML config path")
    args = parser.parse_args()

    controller = HandController(args.config)

    positions = _default_positions(len(controller.joint_ids))
    cycle_times = _normalize_cycle_times([0.5, 0.5, 0.5, 0.5], len(positions))


    try:
        controller.enable_torque()
        for idx, (targets, dwell) in enumerate(zip(positions, cycle_times), start=1):
            cycle_start = time.monotonic()

            data = controller.write_and_read_cycle(targets)

            elapsed = time.monotonic() - cycle_start
            print(f"Cycle {idx}/{len(positions)} time: {elapsed:.4f}s | IR: {data['ir_value']}")
            print(f"  present_positions: {data['present_positions']}")
            print(f"  currents: {data['currents']}")
            print(f"  temperatures: {data['temperatures']}")

            if dwell > 0 and elapsed < dwell:
                time.sleep(dwell - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        controller.disable_torque()
        controller.close()


if __name__ == "__main__":
    main()
