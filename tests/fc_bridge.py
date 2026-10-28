import argparse
import logging
import os
import time
from typing import Optional
from pymavlink import mavutil
import serial

def setup_logger(log_path: str) -> logging.Logger:
    """Configure file and console logging for the MAVLink bridge."""
    logger: logging.Logger = logging.getLogger("FCBridge")
    logger.setLevel(logging.INFO)
    formatter: logging.Formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")

    try:
        log_dir: str = os.path.dirname(log_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        fh: logging.FileHandler = logging.FileHandler(log_path)
        fh.setFormatter(formatter)
        logger.addHandler(fh)
    except OSError as err:
        print(f"File logging error: {err}")

    ch: logging.StreamHandler = logging.StreamHandler()
    ch.setFormatter(formatter)
    logger.addHandler(ch)
    return logger

def detect_fc_port(default_port: str) -> str:
    """Select the active serial port, checking explicit device nodes first."""
    try:
        if os.path.exists(default_port):
            return default_port
        if os.path.exists("/dev/ttyACM0"):
            return "/dev/ttyACM0"
        if os.path.exists("/dev/fc_cube"):
            return "/dev/fc_cube"
        return default_port
    except OSError:
        return default_port

def connect_fc(port: str, baud: int, logger: logging.Logger) -> Optional[mavutil.mavfile]:
    """Establish a MAVLink serial connection and wait for the flight controller heartbeat."""
    conn: Optional[mavutil.mavfile] = None
    try:
        conn = mavutil.mavlink_connection(port, baud=baud, source_system=1, source_component=191)
        logger.info(f"Waiting for heartbeat on {port} at {baud} baud...")
        hb = conn.wait_heartbeat(timeout=10)
        if hb is None:
            logger.warning(f"Timed out waiting for heartbeat on {port}.")
            conn.close()
            return None
        logger.info(f"Heartbeat received from system {conn.target_system}, component {conn.target_component}")
        return conn
    except Exception as err:
        logger.error(f"Failed to connect to FC on {port}: {err}")
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass
        return None

def request_telemetry_streams(conn: mavutil.mavfile, logger: logging.Logger) -> bool:
    """Request standard telemetry message intervals from the flight controller."""
    try:
        messages: list[tuple[int, int]] = [
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 200000),
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100000),
        ]
        for msg_id, interval_us in messages:
            conn.mav.command_long_send(
                conn.target_system,
                conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(msg_id),
                float(interval_us),
                0.0, 0.0, 0.0, 0.0, 0.0
            )
        logger.info("Requested telemetry streams from FC.")
        return True
    except Exception as err:
        logger.error(f"Error requesting streams: {err}")
        return False

def send_approved_test_command(conn: mavutil.mavfile, logger: logging.Logger) -> bool:
    """Send a safe, non-actuating command and verify the acknowledgment."""
    try:
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            float(mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION),
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        )
        ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=3.0)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
            logger.info(f"Approved command verified. ACK result code: {ack.result}")
            return True
        logger.warning("No COMMAND_ACK received within timeout.")
        return False
    except Exception as err:
        logger.error(f"Command exchange failed: {err}")
        return False

def run_bridge(default_port: str = "/dev/ttyACM0", baud: int = 115200) -> None:
    """Main loop maintaining companion heartbeat, telemetry ingestion, and auto-reconnection."""
    logger: logging.Logger = setup_logger("/var/log/dbvf/comms.log")
    conn: Optional[mavutil.mavfile] = None
    last_hb_sent: float = 0.0

    while True:
        try:
            if conn is None:
                active_port: str = detect_fc_port(default_port)
                conn = connect_fc(active_port, baud, logger)
                if conn is None:
                    time.sleep(2.0)
                    continue
                request_telemetry_streams(conn, logger)
                send_approved_test_command(conn, logger)

            now: float = time.time()
            if now - last_hb_sent >= 1.0:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                last_hb_sent = now

            msg = conn.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue

            msg_type: str = msg.get_type()
            if msg_type == "BAD_DATA":
                continue
            elif msg_type == "HEARTBEAT":
                armed: bool = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                logger.info(f"FC Status -> Armed: {armed} | Custom Mode: {msg.custom_mode}")
            elif msg_type == "GLOBAL_POSITION_INT":
                rel_alt_ft: float = (msg.relative_alt / 1000.0) * 3.28084
                logger.info(f"Position -> Lat: {msg.lat / 1e7:.6f}, Lon: {msg.lon / 1e7:.6f}, Alt AGL: {rel_alt_ft:.2f} ft")
            elif msg_type == "SYS_STATUS":
                vbat: float = msg.voltage_battery / 1000.0
                logger.info(f"Battery -> Voltage: {vbat:.2f} V, Remaining: {msg.battery_remaining}%")

        except (OSError, serial.SerialException) as err:
            logger.error(f"Serial link lost ({err}). Attempting recovery...")
            if conn is not None:
                try:
                    conn.close()
                except Exception:
                    pass
            conn = None
            time.sleep(2.0)
        except KeyboardInterrupt:
            logger.info("Shutting down FC bridge.")
            if conn is not None:
                try:
                    conn.close()
                except Exception:
                    pass
            break
        except Exception as err:
            logger.error(f"Unexpected loop error: {err}")
            time.sleep(1.0)

def main() -> None:
    """Parse runtime arguments and invoke the MAVLink communication bridge."""
    parser = argparse.ArgumentParser(description="CubeOrange MAVLink Telemetry Bridge")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    try:
        run_bridge(default_port=args.port, baud=args.baud)
    except Exception as err:
        print(f"Bridge execution failed: {err}")

if __name__ == "__main__":
    main()