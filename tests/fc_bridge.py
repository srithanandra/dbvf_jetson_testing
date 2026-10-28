import argparse
import logging
import os
import time
from pymavlink import mavutil
import serial

def setup_logger(log_path: str) -> logging.Logger:
    """Configure file and console logging for the MAVLink bridge."""
    logger: logging.Logger = logging.getLogger("FlightControllerBridge")
    logger.setLevel(logging.INFO)
    formatter: logging.Formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")

    try:
        log_directory: str = os.path.dirname(log_path)
        if log_directory:
            os.makedirs(log_directory, exist_ok=True)
        file_handler: logging.FileHandler = logging.FileHandler(log_path)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    except OSError as error:
        print(f"File logging error: {error}")

    console_handler: logging.StreamHandler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    return logger

def detect_flight_controller_port(default_port: str) -> str:
    """Select the active serial port, checking explicit device nodes first."""
    try:
        if os.path.exists(default_port):
            return default_port
        if os.path.exists("/dev/ttyACM1"):
            return "/dev/ttyACM1"
        if os.path.exists("/dev/fc_cube"):
            return "/dev/fc_cube"
        return default_port
    except OSError:
        return default_port

def connect_flight_controller(port: str, baud: int, logger: logging.Logger) -> mavutil.mavfile | None:
    """Establish a MAVLink serial connection and wait for the flight controller heartbeat."""
    connection: mavutil.mavfile | None = None
    try:
        connection = mavutil.mavlink_connection(port, baud=baud, source_system=1, source_component=191)
        logger.info(f"Waiting for heartbeat on {port} at {baud} baud...")
        heartbeat = connection.wait_heartbeat(timeout=10)
        if heartbeat is None:
            logger.warning(f"Timed out waiting for heartbeat on {port}.")
            connection.close()
            return None
        logger.info(f"Heartbeat received from system {connection.target_system}, component {connection.target_component}")
        return connection
    except Exception as error:
        logger.error(f"Failed to connect to flight controller on {port}: {error}")
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass
        return None

def request_telemetry_streams(connection: mavutil.mavfile, logger: logging.Logger) -> bool:
    """Request standard telemetry message intervals from the flight controller."""
    try:
        messages: list[tuple[int, int]] = [
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 200000),
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100000),
        ]
        for message_id, interval_microseconds in messages:
            connection.mav.command_long_send(
                connection.target_system,
                connection.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(message_id),
                float(interval_microseconds),
                0.0, 0.0, 0.0, 0.0, 0.0
            )
        logger.info("Requested telemetry streams from flight controller.")
        return True
    except Exception as error:
        logger.error(f"Error requesting streams: {error}")
        return False

def send_approved_test_command(connection: mavutil.mavfile, logger: logging.Logger) -> bool:
    """Send a safe, non-actuating command and verify the acknowledgment."""
    try:
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            float(mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION),
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        )
        acknowledgement = connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=3.0)
        if acknowledgement and acknowledgement.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
            logger.info(f"Approved command verified. Acknowledgement result code: {acknowledgement.result}")
            return True
        logger.warning("No COMMAND_ACK received within timeout.")
        return False
    except Exception as error:
        logger.error(f"Command exchange failed: {error}")
        return False

def run_bridge(default_port: str = "/dev/ttyACM1", baud: int = 115200) -> None:
    """Main loop maintaining companion heartbeat, telemetry ingestion, and auto-reconnection."""
    logger: logging.Logger = setup_logger("/var/log/dbvf/comms.log")
    connection: mavutil.mavfile | None = None
    last_heartbeat_sent: float = 0.0

    while True:
        try:
            if connection is None:
                active_port: str = detect_flight_controller_port(default_port)
                connection = connect_flight_controller(active_port, baud, logger)
                if connection is None:
                    time.sleep(2.0)
                    continue
                request_telemetry_streams(connection, logger)
                send_approved_test_command(connection, logger)

            current_time: float = time.time()
            if current_time - last_heartbeat_sent >= 1.0:
                connection.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                last_heartbeat_sent = current_time

            message = connection.recv_match(blocking=True, timeout=1.0)
            if message is None:
                continue

            message_type: str = message.get_type()
            if message_type == "BAD_DATA":
                continue
            elif message_type == "HEARTBEAT":
                armed: bool = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                logger.info(f"Flight Controller Status -> Armed: {armed} | Custom Mode: {message.custom_mode}")
            elif message_type == "GLOBAL_POSITION_INT":
                relative_altitude_feet: float = (message.relative_alt / 1000.0) * 3.28084
                logger.info(f"Position -> Lat: {message.lat / 1e7:.6f}, Lon: {message.lon / 1e7:.6f}, Alt AGL: {relative_altitude_feet:.2f} ft")
            elif message_type == "SYS_STATUS":
                voltage_battery: float = message.voltage_battery / 1000.0
                logger.info(f"Battery -> Voltage: {voltage_battery:.2f} V, Remaining: {message.battery_remaining}%")

        except (OSError, serial.SerialException) as error:
            logger.error(f"Serial link lost ({error}). Attempting recovery...")
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass
            connection = None
            time.sleep(2.0)
        except KeyboardInterrupt:
            logger.info("Shutting down flight controller bridge.")
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass
            break
        except Exception as error:
            logger.error(f"Unexpected loop error: {error}")
            time.sleep(1.0)

def main() -> None:
    """Parse runtime arguments and invoke the MAVLink communication bridge."""
    parser: argparse.ArgumentParser = argparse.ArgumentParser(description="CubeOrange MAVLink Telemetry Bridge")
    parser.add_argument("--port", type=str, default="/dev/ttyACM1")
    parser.add_argument("--baud", type=int, default=115200)

    try:
        arguments: argparse.Namespace = parser.parse_args()
        run_bridge(default_port=arguments.port, baud=arguments.baud)
    except Exception as error:
        print(f"Bridge execution failed: {error}")

if __name__ == "__main__":
    main()
