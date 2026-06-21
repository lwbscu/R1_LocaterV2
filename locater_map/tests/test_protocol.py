from locater_map.data_model import RobotFrame
from locater_map.protocol import make_r1_csv_v2_line, make_r1_csv_v3_line, make_r1m_line, parse_line


def test_r1m_valid_frame_parse():
    line = make_r1m_line(
        RobotFrame(
            source_time_ms=123456,
            seq=8021,
            pos_x_cm=152.34,
            pos_y_cm=86.20,
            pos_yaw_deg=91.50,
            calib_x_cm=150.80,
            calib_y_cm=85.90,
            calib_yaw_deg=91.20,
            h30_x_cm=12.40,
            h30_y_cm=2.10,
            h30_yaw_deg=90.80,
            lidar_x_cm=151.90,
            lidar_y_cm=86.40,
            lidar_yaw_deg=91.20,
            dt35_1_mm=345,
            dt35_2_mm=1280,
            status=0x003F,
        )
    )
    result = parse_line(line)
    assert result.ok
    assert result.frame is not None
    assert result.frame.seq == 8021
    assert result.frame.status == 0x003F
    assert result.frame.crc_ok


def test_crc_error_frame_is_rejected():
    good = make_r1m_line(RobotFrame(seq=1, status=1))
    bad = good[:-6] + "0000\r\n"
    result = parse_line(bad)
    assert result.frame is None
    assert result.crc_error


def test_no_crc_debug_frame():
    line = "$R1M,1,10,2,1,2,3,4,5,6,7,8,9,10,11,12,13,14,0x3F,\r\n"
    result = parse_line(line, allow_no_crc=True)
    assert result.ok
    assert result.no_crc
    assert result.frame is not None
    assert result.frame.crc_state == "no_crc"


def test_legacy_csv_parse():
    result = parse_line("90.0,1.0,2.0,3.0,4.0", mode="legacy_csv")
    assert result.ok
    assert result.frame is not None
    assert result.frame.h30_yaw_deg == 90.0
    assert result.frame.calib_x_cm == 3.0


def test_legacy_csv_nine_value_current_firmware_layout():
    result = parse_line("1.0,2.0,3.0,10.0,20.0,30.0,100.0,200.0,300.0", mode="legacy_csv")
    assert result.ok
    assert result.frame is not None
    assert result.frame.pos_x_cm == 1.0
    assert result.frame.pos_y_cm == 2.0
    assert result.frame.pos_yaw_deg == 3.0
    assert result.frame.lidar_x_cm == 10.0
    assert result.frame.lidar_y_cm == 20.0
    assert result.frame.lidar_yaw_deg == 30.0
    assert result.frame.calib_x_cm == 100.0
    assert result.frame.calib_y_cm == 200.0
    assert result.frame.calib_yaw_deg == 300.0
    assert result.frame.h30_yaw_deg == 300.0


def test_r1_csv_v2_current_firmware_layout():
    line = make_r1_csv_v2_line(
        RobotFrame(
            pos_x_cm=1.0,
            pos_y_cm=2.0,
            pos_yaw_deg=3.0,
            lidar_x_cm=10.0,
            lidar_y_cm=20.0,
            lidar_yaw_deg=30.0,
            calib_x_cm=100.0,
            calib_y_cm=200.0,
            calib_yaw_deg=300.0,
            h30_yaw_deg=301.0,
            h30_x_cm=4.0,
            h30_y_cm=5.0,
            encoder_x_cm=100.0,
            encoder_y_cm=200.0,
            h30_valid=True,
            h30_has_attitude=True,
            h30_has_accel=True,
            lidar_valid=True,
            lidar_online=False,
            h30_packet_count=12,
            h30_rx_byte_count=1200,
            lidar_packet_count=34,
            lidar_rx_byte_count=560,
            h30_crc_error_count=1,
            h30_frame_error_count=2,
            lidar_checksum_error_count=3,
            lidar_frame_error_count=4,
            source_time_ms=9988,
            h30_last_update_ms=9970,
            lidar_last_update_ms=9900,
            x_raw_count=100,
            y_raw_count=200,
            x_delta_count=-3,
            y_delta_count=4,
            x_total_count=-30,
            y_total_count=40,
            x_index_seen=True,
            y_index_seen=False,
            encoder_dis_p_mm=0.125,
            encoder_dis_q_mm=-0.250,
            status=0x00AF,
        )
    )
    result = parse_line(line, mode="r1_csv_v2")
    assert result.ok
    assert result.frame is not None
    assert result.frame.protocol == "r1_csv_v2"
    assert result.frame.h30_yaw_deg == 301.0
    assert result.frame.h30_packet_count == 12
    assert result.frame.h30_rx_byte_count == 1200
    assert result.frame.h30_has_accel
    assert result.frame.lidar_packet_count == 34
    assert result.frame.lidar_rx_byte_count == 560
    assert result.frame.source_time_ms == 9988
    assert result.frame.x_raw_count == 100
    assert result.frame.y_raw_count == 200
    assert result.frame.x_delta_count == -3
    assert result.frame.y_delta_count == 4
    assert result.frame.x_total_count == -30
    assert result.frame.y_total_count == 40
    assert result.frame.x_pulse_seen
    assert result.frame.y_pulse_seen
    assert result.frame.x_index_seen
    assert not result.frame.y_index_seen
    assert result.frame.encoder_dis_p_mm == 0.125
    assert result.frame.encoder_dis_q_mm == -0.250
    assert result.frame.status == 0x00AF


def test_r1_csv_v3_compact_firmware_layout():
    frame = RobotFrame(
        pos_x_cm=1.0,
        pos_y_cm=2.0,
        pos_yaw_deg=3.0,
        lidar_x_cm=10.0,
        lidar_y_cm=20.0,
        lidar_yaw_deg=30.0,
        encoder_x_cm=100.0,
        encoder_y_cm=200.0,
        h30_yaw_deg=300.0,
        dt35_1_mm=0.0,
        dt35_2_mm=0.0,
        status=0x0C4F,
    )
    result = parse_line(make_r1_csv_v3_line(frame), mode="r1_csv_v3")
    assert result.ok
    assert result.frame is not None
    assert result.frame.protocol == "r1_csv_v3"
    assert result.frame.pos_x_cm == 1.0
    assert result.frame.lidar_yaw_deg == 30.0
    assert result.frame.encoder_x_cm == 100.0
    assert result.frame.calib_x_cm == 100.0
    assert result.frame.calib_yaw_deg == 300.0
    assert result.frame.h30_yaw_deg == 300.0
    assert result.frame.x_pulse_seen
    assert result.frame.y_pulse_seen
    assert result.frame.h30_valid
    assert result.frame.lidar_valid
    assert result.frame.lidar_online
    assert not result.frame.dt35_1_valid
    assert not result.frame.dt35_2_valid
    assert result.frame.status == 0x0C4F


def test_r1_csv_v2_old_25_field_layout_still_parses():
    line = "1,2,3,4,5,6,7,8,9,10,11,12,13,14,1,1,0,0,21,0,2,3,0,0,7"
    result = parse_line(line, mode="r1_csv_v2")
    assert result.ok
    assert result.frame is not None
    assert result.frame.pos_x_cm == 1
    assert result.frame.h30_packet_count == 21
    assert result.frame.h30_rx_byte_count == 0
    assert result.frame.x_raw_count == 0
    assert result.frame.status == 7
