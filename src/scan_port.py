#!/usr/bin/env python3
"""Scan a serial port for a Dynamixel with a specific ID."""

from __future__ import annotations

import argparse
import sys

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


def scan_id(port: str, baudrate: int, dxl_id: int, protocol_version: float) -> bool:
	"""Return True if a Dynamixel with the given ID responds on the port."""
	port_handler = PortHandler(port)
	packet_handler = PacketHandler(protocol_version)

	if not port_handler.openPort():
		print(f"Failed to open port: {port}")
		return False

	if not port_handler.setBaudRate(baudrate):
		print(f"Failed to set baudrate: {baudrate}")
		port_handler.closePort()
		return False

	model_number, comm_result, dxl_error = packet_handler.ping(port_handler, dxl_id)
	port_handler.closePort()

	if comm_result == COMM_SUCCESS and dxl_error == 0:
		print(f"Found Dynamixel ID {dxl_id} (model {model_number}) on {port} @ {baudrate}")
		return True

	print(f"No response from ID {dxl_id} on {port} @ {baudrate}")
	return False


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Scan a serial port for a Dynamixel ID.")
	parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyUSB0)")
	parser.add_argument("--baudrate", type=int, default=1000000, help="Baudrate to use")
	parser.add_argument("--id", dest="dxl_id", type=int, default=30, help="Dynamixel ID")
	parser.add_argument(
		"--protocol",
		type=float,
		default=2.0,
		help="Dynamixel protocol version (1.0 or 2.0)",
	)
	return parser.parse_args()


def main() -> int:
	args = parse_args()
	found = scan_id(args.port, args.baudrate, args.dxl_id, args.protocol)
	return 0 if found else 1


if __name__ == "__main__":
	raise SystemExit(main())
