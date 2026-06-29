from pymavlink import mavutil

DEFAULT_CONNECTION_STRING = "udpin:127.0.0.1:14550"
DEFAULT_BAUD = 115200
DEFAULT_HEARTBEAT_TIMEOUT = 15


def connect_mavlink(
        connection_string=DEFAULT_CONNECTION_STRING,
        baud=DEFAULT_BAUD,
        heartbeat_timeout=DEFAULT_HEARTBEAT_TIMEOUT,
        logger=None,
):
    """MAVLink baglantisi kurar ve heartbeat bekledikten sonra master nesnesini dondurur.

    Ornek connection_string degerleri:
        - SITL: "udpin:127.0.0.1:14550"
        - Orange Cube USB: "/dev/ttyACM0"
        - TELEM / USB-TTL: "/dev/ttyUSB0"
    """
    if logger is not None:
        logger.info(f"MAVLink baglaniyor: {connection_string}, baud={baud}")

    master = mavutil.mavlink_connection(connection_string, baud=baud)

    if logger is not None:
        logger.info("Heartbeat bekleniyor...")

    master.wait_heartbeat(timeout=heartbeat_timeout)

    if logger is not None:
        logger.info(
            f"MAVLink baglandi. system={master.target_system}, "
            f"component={master.target_component}"
        )

    return master


__all__ = [
    "DEFAULT_CONNECTION_STRING",
    "DEFAULT_BAUD",
    "DEFAULT_HEARTBEAT_TIMEOUT",
    "connect_mavlink",
]
