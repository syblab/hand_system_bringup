import glob
import os
import re
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


FLIR_USB_VENDOR_ID = "09cb"
FLIR_USB_PRODUCT_ID = "1007"
ORBBEC_SERIAL_NUMBER = "AY0F7010108"


def _run_command(command):
    """Run a short device-query command without stopping the launch."""
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    return result.stdout if result.returncode == 0 else ""


def _video_device_index(device_path):
    match = re.fullmatch(r"/dev/video(\d+)", device_path)
    return int(match.group(1)) if match else None


def _detect_flir_camera():
    """Find the FLIR Ex-Series UVC capture node by USB ID and format."""
    candidates = []
    for device_path in glob.glob("/dev/video*"):
        camera_index = _video_device_index(device_path)
        if camera_index is None:
            continue

        properties = _run_command([
            "udevadm",
            "info",
            "--query=property",
            f"--name={device_path}",
        ])
        is_flir = (
            f"ID_VENDOR_ID={FLIR_USB_VENDOR_ID}" in properties
            and f"ID_MODEL_ID={FLIR_USB_PRODUCT_ID}" in properties
        ) or "ID_MODEL=FLIR_Ex-Series" in properties
        if not is_flir:
            continue

        formats = _run_command([
            "v4l2-ctl",
            "--list-formats-ext",
            "-d",
            device_path,
        ])
        score = 0
        if ":capture:" in properties:
            score += 1
        if "YUYV" in formats:
            score += 4
        if "320x240" in formats:
            score += 2
        candidates.append((score, camera_index, device_path))

    if not candidates:
        return None, None

    # Prefer a 320x240 YUYV capture node. Use the lower index for ties.
    candidates.sort(key=lambda item: (-item[0], item[1]))
    _, camera_index, device_path = candidates[0]
    return camera_index, device_path


def _launch_flir_node(context, *args, **kwargs):
    del args, kwargs
    requested = LaunchConfiguration("flir_camera_index").perform(
        context
    ).strip()

    if requested.lower() in ("", "auto"):
        camera_index, device_path = _detect_flir_camera()
        if camera_index is None:
            return [LogInfo(msg=(
                "[ERROR] "
                "FLIR Ex-Series (USB 09cb:1007) was not found. "
                "Check USB/UVC mode and run v4l2-ctl --list-devices."
            ))]
        selection_message = (
            f"Auto-detected FLIR Ex-Series: {device_path} "
            f"(camera_index={camera_index})"
        )
    else:
        try:
            camera_index = int(requested)
        except ValueError:
            return [LogInfo(msg=(
                "[ERROR] "
                "flir_camera_index must be 'auto' or an integer, "
                f"but received: {requested}"
            ))]
        device_path = f"/dev/video{camera_index}"
        if not os.path.exists(device_path):
            return [LogInfo(msg=(
                f"[ERROR] Requested FLIR device does not exist: {device_path}"
            ))]
        selection_message = (
            f"Using manually selected FLIR device: {device_path}"
        )

    flir_node = Node(
        package="flir_temperature_click",
        executable="flir_temperature_click_node",
        name="flir_temperature_click",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_ros_image": False,
            "camera_index": camera_index,
            "camera_width": 320,
            "camera_height": 240,
            "camera_fps": 15.0,
            "display_scale": 2.0,
            "show_window": False,
            "enable_temperature_ocr": True,
            "publish_image_topic": "/flir/image_raw",
        }],
    )
    return [LogInfo(msg=selection_message), flir_node]


def generate_launch_description():
    """RGB-D、FLIR、手角度、スライダー、安全ゲート、EPOSを起動する。

    EPOSは既定で有効。モータ用3D角度とvalidフラグを直接購読し、
    EPOSノード自身で180度拒否・入力タイムアウト・system_readyを判定する。
    スライダー値は/slider_rawと/slider_smoothへ配信する。
    """

    arguments = [
        DeclareLaunchArgument("enable_epos", default_value="true"),
        DeclareLaunchArgument("enable_slider", default_value="true"),
        DeclareLaunchArgument(
            "slider_port", default_value="/dev/ttyACM0"
        ),
        DeclareLaunchArgument(
            "slider_use_binary", default_value="false"
        ),
        DeclareLaunchArgument(
            "slider_publish_rate_hz", default_value="100.0"
        ),
        DeclareLaunchArgument(
            "slider_smoothing_alpha", default_value="0.10"
        ),
        DeclareLaunchArgument(
            "slider_max_value", default_value="1023.0"
        ),
        DeclareLaunchArgument(
            "slider_ascii_max_lines_per_tick", default_value="20"
        ),
        DeclareLaunchArgument(
            "epos_angle_topic",
            default_value="/hand_index_angle_control",
        ),
        DeclareLaunchArgument(
            "epos_angle_valid_topic",
            default_value="/hand_index_angle_control_valid",
        ),
        DeclareLaunchArgument(
            "require_angle_valid", default_value="true"
        ),
        DeclareLaunchArgument(
            "angle_valid_timeout_sec", default_value="0.35"
        ),
        DeclareLaunchArgument(
            "safe_stop_angle_deg", default_value="180.0"
        ),
        DeclareLaunchArgument(
            "invalid_rearm_sec", default_value="1.0"
        ),
        DeclareLaunchArgument(
            "continue_close_on_angle_loss", default_value="true"
        ),
        DeclareLaunchArgument(
            "close_latch_angle_deg", default_value="140.0"
        ),
        DeclareLaunchArgument(
            "close_latch_release_angle_deg", default_value="110.0"
        ),
        DeclareLaunchArgument(
            "close_latch_confirm_count", default_value="4"
        ),
        DeclareLaunchArgument(
            "close_latch_release_confirm_count", default_value="8"
        ),
        DeclareLaunchArgument(
            "close_on_angle_loss_max_sec", default_value="10.0"
        ),
        DeclareLaunchArgument(
            "hold_last_target_on_angle_loss", default_value="true"
        ),
        DeclareLaunchArgument(
            "angle_target_hold_max_sec", default_value="10.0"
        ),
        DeclareLaunchArgument(
            "angle_recovery_confirm_count", default_value="6"
        ),
        DeclareLaunchArgument(
            "angle_recovery_match_tolerance_deg", default_value="8.0"
        ),
        DeclareLaunchArgument(
            "epos_slider_topic", default_value="/slider_smooth"
        ),
        DeclareLaunchArgument(
            "epos_ready_topic", default_value="/flir_fusion/system_ready"
        ),
        DeclareLaunchArgument(
            "require_system_ready", default_value="true"
        ),
        DeclareLaunchArgument("max_rpm", default_value="10000.0"),
        DeclareLaunchArgument(
            "minimum_running_rpm", default_value="10000.0"
        ),
        DeclareLaunchArgument(
            "near_minimum_rpm", default_value="5000.0"
        ),
        DeclareLaunchArgument(
            "open_near_minimum_rpm", default_value="5000.0"
        ),
        DeclareLaunchArgument(
            "open_near_speed_limit_rpm", default_value="5000.0"
        ),
        DeclareLaunchArgument(
            "slow_zone_error", default_value="120.0"
        ),
        DeclareLaunchArgument(
            "open_slow_zone_error", default_value="120.0"
        ),
        DeclareLaunchArgument(
            "precision_zone_error", default_value="35.0"
        ),
        DeclareLaunchArgument(
            "precision_minimum_rpm", default_value="2000.0"
        ),
        DeclareLaunchArgument(
            "precision_speed_limit_rpm", default_value="2000.0"
        ),
        DeclareLaunchArgument(
            "rpm_filter_alpha", default_value="0.35"
        ),
        DeclareLaunchArgument(
            "rpm_slew_rate", default_value="40000.0"
        ),
        DeclareLaunchArgument(
            "rpm_command_epsilon", default_value="20.0"
        ),
        DeclareLaunchArgument(
            "motor_direction_sign", default_value="1.0"
        ),
        DeclareLaunchArgument(
            "angle_open_deg", default_value="5.0"
        ),
        DeclareLaunchArgument(
            "angle_close_deg", default_value="150.0"
        ),
        DeclareLaunchArgument(
            "angle_input_margin_deg", default_value="8.0"
        ),
        DeclareLaunchArgument(
            "slider_open_value", default_value="959.0"
        ),
        DeclareLaunchArgument(
            "slider_close_value", default_value="2.0"
        ),
        DeclareLaunchArgument(
            "calibration_angle_points_csv",
            default_value="5,30,60,90,120,150",
        ),
        DeclareLaunchArgument(
            "calibration_slider_points_csv",
            default_value="959,794,596,398,200,2",
        ),
        DeclareLaunchArgument(
            "slider_valid_min", default_value="0.0"
        ),
        DeclareLaunchArgument(
            "slider_valid_max", default_value="1023.0"
        ),
        DeclareLaunchArgument(
            "open_slider_stop_band", default_value="8.0"
        ),
        DeclareLaunchArgument(
            "close_slider_stop_band", default_value="5.0"
        ),
        DeclareLaunchArgument(
            "close_slider_end_margin", default_value="5.0"
        ),
        DeclareLaunchArgument(
            "slider_move_band", default_value="10.0"
        ),
        DeclareLaunchArgument(
            "slider_jump_reject", default_value="150.0"
        ),
        DeclareLaunchArgument(
            "input_timeout_sec", default_value="0.30"
        ),
        DeclareLaunchArgument(
            "system_ready_timeout_sec", default_value="0.5"
        ),
        DeclareLaunchArgument(
            "max_continuous_run_sec", default_value="10.0"
        ),
        DeclareLaunchArgument(
            "require_closed_before_start", default_value="true"
        ),
        DeclareLaunchArgument(
            "startup_closed_angle_margin_deg", default_value="12.0"
        ),
        DeclareLaunchArgument(
            "startup_confirm_count", default_value="3"
        ),
        DeclareLaunchArgument(
            "control_median_window", default_value="5"
        ),
        DeclareLaunchArgument(
            "control_ema_alpha", default_value="0.35"
        ),
        DeclareLaunchArgument(
            "control_max_step_deg", default_value="20.0"
        ),
        DeclareLaunchArgument(
            "control_jump_confirm_frames", default_value="3"
        ),
        DeclareLaunchArgument(
            "control_jump_match_tolerance_deg", default_value="15.0"
        ),
        DeclareLaunchArgument(
            "control_valid_confirm_frames", default_value="3"
        ),
        DeclareLaunchArgument(
            "control_hold_sec", default_value="0.12"
        ),
        DeclareLaunchArgument(
            "orbbec_serial_number",
            default_value=ORBBEC_SERIAL_NUMBER,
        ),
        DeclareLaunchArgument(
            "flir_camera_index", default_value="auto"
        ),
        DeclareLaunchArgument("fusion_alpha", default_value="0.48"),
        DeclareLaunchArgument(
            "fusion_overlay_enabled", default_value="true"
        ),
        DeclareLaunchArgument(
            "fusion_display_scale", default_value="1.0"
        ),
        DeclareLaunchArgument(
            "fusion_display_scale_min", default_value="0.5"
        ),
        DeclareLaunchArgument(
            "fusion_display_scale_max", default_value="3.0"
        ),
        DeclareLaunchArgument(
            "fusion_display_scale_step", default_value="0.25"
        ),
        DeclareLaunchArgument(
            "enable_finger_motion_map", default_value="true"
        ),
        DeclareLaunchArgument(
            "finger_map_relative_mode", default_value="true"
        ),
        DeclareLaunchArgument(
            "finger_map_coordinate_mode", default_value="wrist_fixed"
        ),
        DeclareLaunchArgument(
            "finger_map_history_sec", default_value="8.0"
        ),
        DeclareLaunchArgument(
            "finger_map_range_m", default_value="0.20"
        ),
        DeclareLaunchArgument(
            "finger_map_display_scale", default_value="1.0"
        ),
        DeclareLaunchArgument(
            "finger_map_save_csv", default_value="true"
        ),
        DeclareLaunchArgument(
            "finger_map_path_deadband_m", default_value="0.003"
        ),
        DeclareLaunchArgument(
            "finger_map_max_segment_m", default_value="0.060"
        ),
        DeclareLaunchArgument(
            "calibration_point_count", default_value="5"
        ),
        DeclareLaunchArgument(
            "maximum_calibration_rmse_px", default_value="5.0"
        ),
        DeclareLaunchArgument(
            "homography_file",
            default_value="~/.ros/flir_depth_homography.yaml",
        ),
    ]

    astra_rgbd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("orbbec_camera"),
                "launch",
                "astra_stereo_u3.launch.py",
            ])
        ),
        launch_arguments={
            "serial_number": LaunchConfiguration(
                "orbbec_serial_number"
            ),
            "enable_color": "true",
            "enable_depth": "true",
            "enable_ir": "false",
            "color_width": "640",
            "color_height": "480",
            "color_fps": "30",
            "depth_width": "640",
            "depth_height": "400",
            "depth_fps": "30",
            "depth_format": "Y11",
            "depth_registration": "true",
            "enable_frame_sync": "false",
        }.items(),
    )

    hand_node = Node(
        package="hand_angle_node",
        executable="hand_angle_node",
        name="hand_angle_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_ros_color": True},
            {"color_topic": "/camera/color/image_raw"},
            {"color_info_topic": "/camera/color/camera_info"},
            {"depth_topic": "/camera/depth/image_raw"},
            {"display_scale": 1.0},
            {"display_scale_min": 0.5},
            {"display_scale_max": 3.0},
            {"display_scale_step": 0.25},
            {"depth_is_registered": True},
            {"control_angle_topic": "/hand_index_angle_control"},
            {
                "control_valid_topic":
                "/hand_index_angle_control_valid"
            },
            {
                "control_median_window": ParameterValue(
                    LaunchConfiguration("control_median_window"),
                    value_type=int,
                )
            },
            {
                "control_ema_alpha": ParameterValue(
                    LaunchConfiguration("control_ema_alpha"),
                    value_type=float,
                )
            },
            {
                "control_max_step_deg": ParameterValue(
                    LaunchConfiguration("control_max_step_deg"),
                    value_type=float,
                )
            },
            {
                "control_jump_confirm_frames": ParameterValue(
                    LaunchConfiguration("control_jump_confirm_frames"),
                    value_type=int,
                )
            },
            {
                "control_jump_match_tolerance_deg": ParameterValue(
                    LaunchConfiguration(
                        "control_jump_match_tolerance_deg"
                    ),
                    value_type=float,
                )
            },
            {
                "control_valid_confirm_frames": ParameterValue(
                    LaunchConfiguration("control_valid_confirm_frames"),
                    value_type=int,
                )
            },
            {
                "control_hold_sec": ParameterValue(
                    LaunchConfiguration("control_hold_sec"),
                    value_type=float,
                )
            },
        ],
    )

    fusion_node = Node(
        package="flir_depth_fusion",
        executable="flir_depth_fusion_node",
        name="flir_depth_fusion_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "rgb_topic": "/camera/color/image_raw",
            "depth_topic": "/camera/depth/image_raw",
            "camera_info_topic": "/camera/color/camera_info",
            "flir_topic": "/flir/image_raw",
            "max_temperature_topic": "/flir/max_temperature",
            "min_temperature_topic": "/flir/min_temperature",
            "homography_file": LaunchConfiguration("homography_file"),
            "overlay_alpha": ParameterValue(
                LaunchConfiguration("fusion_alpha"),
                value_type=float,
            ),
            "overlay_enabled": ParameterValue(
                LaunchConfiguration("fusion_overlay_enabled"),
                value_type=bool,
            ),
            "display_scale": ParameterValue(
                LaunchConfiguration("fusion_display_scale"),
                value_type=float,
            ),
            "display_scale_min": ParameterValue(
                LaunchConfiguration("fusion_display_scale_min"),
                value_type=float,
            ),
            "display_scale_max": ParameterValue(
                LaunchConfiguration("fusion_display_scale_max"),
                value_type=float,
            ),
            "display_scale_step": ParameterValue(
                LaunchConfiguration("fusion_display_scale_step"),
                value_type=float,
            ),
            "calibration_point_count": ParameterValue(
                LaunchConfiguration("calibration_point_count"),
                value_type=int,
            ),
            # FLIRの共通画角がRGB-D画像内で狭いため、
            # 対応点の凸包面積条件を標準値より緩和する。
            "minimum_point_area_ratio": 0.02,
            "maximum_calibration_rmse_px": ParameterValue(
                LaunchConfiguration(
                    "maximum_calibration_rmse_px"
                ),
                value_type=float,
            ),
            "require_ocr_for_ready": True,
            "ocr_timeout_sec": 3.0,
        }],
    )

    finger_motion_map_node = Node(
        package="finger_motion_map",
        executable="finger_motion_map_node",
        name="finger_motion_map_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(
            LaunchConfiguration("enable_finger_motion_map")
        ),
        parameters=[{
            "landmarks_topic": "/hand_landmarks_3d",
            "quality_topic": "/hand_landmarks_3d_quality",
            "output_image_topic": "/finger_motion_map/image",
            "coordinate_mode": LaunchConfiguration(
                "finger_map_coordinate_mode"
            ),
            "relative_mode": ParameterValue(
                LaunchConfiguration("finger_map_relative_mode"),
                value_type=bool,
            ),
            "history_seconds": ParameterValue(
                LaunchConfiguration("finger_map_history_sec"),
                value_type=float,
            ),
            "map_range_m": ParameterValue(
                LaunchConfiguration("finger_map_range_m"),
                value_type=float,
            ),
            "display_scale": ParameterValue(
                LaunchConfiguration("finger_map_display_scale"),
                value_type=float,
            ),
            "save_csv": ParameterValue(
                LaunchConfiguration("finger_map_save_csv"),
                value_type=bool,
            ),
            "path_deadband_m": ParameterValue(
                LaunchConfiguration("finger_map_path_deadband_m"),
                value_type=float,
            ),
            "maximum_trajectory_segment_m": ParameterValue(
                LaunchConfiguration("finger_map_max_segment_m"),
                value_type=float,
            ),
        }],
    )

    angle_gate_node = Node(
        package="flir_depth_fusion",
        executable="hand_angle_gate_node",
        name="hand_angle_gate_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "input_topic": "/hand_index_angle_control",
            "output_topic": "/hand_index_angle_control_gated",
            "ready_topic": "/flir_fusion/system_ready",
            "safe_stop_angle_deg": 180.0,
        }],
    )

    # Arduinoに接続したスライドボリュームをEPOSより先に起動する。
    # このノードは /slider_raw と /slider_smooth を配信する。
    # Arduino側のアナログ入力ピン（A0など）はArduinoスケッチで指定する。
    slider_node = Node(
        package="arduino_slider",
        executable="slider_node",
        name="slider_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_slider")),
        parameters=[{
            "port": LaunchConfiguration("slider_port"),
            "use_binary": ParameterValue(
                LaunchConfiguration("slider_use_binary"),
                value_type=bool,
            ),
            "publish_rate_hz": ParameterValue(
                LaunchConfiguration("slider_publish_rate_hz"),
                value_type=float,
            ),
            "smoothing_alpha": ParameterValue(
                LaunchConfiguration("slider_smoothing_alpha"),
                value_type=float,
            ),
            "max_value": ParameterValue(
                LaunchConfiguration("slider_max_value"),
                value_type=float,
            ),
            "topic_raw": "/slider_raw",
            "topic_smooth": "/slider_smooth",
            "ascii_max_lines_per_tick": ParameterValue(
                LaunchConfiguration(
                    "slider_ascii_max_lines_per_tick"
                ),
                value_type=int,
            ),
        }],
    )

    epos_node = Node(
        package="epos_control",
        executable="epos_control_node",
        name="epos_control_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_epos")),
        parameters=[{
            "index_angle_topic": LaunchConfiguration("epos_angle_topic"),
            "angle_valid_topic": LaunchConfiguration(
                "epos_angle_valid_topic"
            ),
            "require_angle_valid": ParameterValue(
                LaunchConfiguration("require_angle_valid"), value_type=bool
            ),
            "angle_valid_timeout_sec": ParameterValue(
                LaunchConfiguration("angle_valid_timeout_sec"),
                value_type=float,
            ),
            "safe_stop_angle_deg": ParameterValue(
                LaunchConfiguration("safe_stop_angle_deg"), value_type=float
            ),
            "invalid_rearm_sec": ParameterValue(
                LaunchConfiguration("invalid_rearm_sec"), value_type=float
            ),
            "continue_close_on_angle_loss": ParameterValue(
                LaunchConfiguration("continue_close_on_angle_loss"),
                value_type=bool,
            ),
            "close_latch_angle_deg": ParameterValue(
                LaunchConfiguration("close_latch_angle_deg"),
                value_type=float,
            ),
            "close_latch_release_angle_deg": ParameterValue(
                LaunchConfiguration("close_latch_release_angle_deg"),
                value_type=float,
            ),
            "close_latch_confirm_count": ParameterValue(
                LaunchConfiguration("close_latch_confirm_count"),
                value_type=int,
            ),
            "close_latch_release_confirm_count": ParameterValue(
                LaunchConfiguration(
                    "close_latch_release_confirm_count"
                ),
                value_type=int,
            ),
            "close_on_angle_loss_max_sec": ParameterValue(
                LaunchConfiguration("close_on_angle_loss_max_sec"),
                value_type=float,
            ),
            "hold_last_target_on_angle_loss": ParameterValue(
                LaunchConfiguration("hold_last_target_on_angle_loss"),
                value_type=bool,
            ),
            "angle_target_hold_max_sec": ParameterValue(
                LaunchConfiguration("angle_target_hold_max_sec"),
                value_type=float,
            ),
            "angle_recovery_confirm_count": ParameterValue(
                LaunchConfiguration("angle_recovery_confirm_count"),
                value_type=int,
            ),
            "angle_recovery_match_tolerance_deg": ParameterValue(
                LaunchConfiguration(
                    "angle_recovery_match_tolerance_deg"
                ),
                value_type=float,
            ),
            "slider_topic": LaunchConfiguration("epos_slider_topic"),
            "ready_topic": LaunchConfiguration("epos_ready_topic"),
            "require_system_ready": ParameterValue(
                LaunchConfiguration("require_system_ready"), value_type=bool
            ),
            "max_rpm": ParameterValue(
                LaunchConfiguration("max_rpm"), value_type=float
            ),
            "minimum_running_rpm": ParameterValue(
                LaunchConfiguration("minimum_running_rpm"), value_type=float
            ),
            "near_minimum_rpm": ParameterValue(
                LaunchConfiguration("near_minimum_rpm"), value_type=float
            ),
            "open_near_minimum_rpm": ParameterValue(
                LaunchConfiguration("open_near_minimum_rpm"), value_type=float
            ),
            "open_near_speed_limit_rpm": ParameterValue(
                LaunchConfiguration("open_near_speed_limit_rpm"),
                value_type=float,
            ),
            "slow_zone_error": ParameterValue(
                LaunchConfiguration("slow_zone_error"), value_type=float
            ),
            "open_slow_zone_error": ParameterValue(
                LaunchConfiguration("open_slow_zone_error"),
                value_type=float,
            ),
            "precision_zone_error": ParameterValue(
                LaunchConfiguration("precision_zone_error"),
                value_type=float,
            ),
            "precision_minimum_rpm": ParameterValue(
                LaunchConfiguration("precision_minimum_rpm"),
                value_type=float,
            ),
            "precision_speed_limit_rpm": ParameterValue(
                LaunchConfiguration("precision_speed_limit_rpm"),
                value_type=float,
            ),
            "rpm_filter_alpha": ParameterValue(
                LaunchConfiguration("rpm_filter_alpha"), value_type=float
            ),
            "rpm_slew_rate": ParameterValue(
                LaunchConfiguration("rpm_slew_rate"), value_type=float
            ),
            "rpm_command_epsilon": ParameterValue(
                LaunchConfiguration("rpm_command_epsilon"), value_type=float
            ),
            "motor_direction_sign": ParameterValue(
                LaunchConfiguration("motor_direction_sign"), value_type=float
            ),
            "angle_open_deg": ParameterValue(
                LaunchConfiguration("angle_open_deg"), value_type=float
            ),
            "angle_close_deg": ParameterValue(
                LaunchConfiguration("angle_close_deg"), value_type=float
            ),
            "angle_input_margin_deg": ParameterValue(
                LaunchConfiguration("angle_input_margin_deg"),
                value_type=float,
            ),
            "slider_open_value": ParameterValue(
                LaunchConfiguration("slider_open_value"), value_type=float
            ),
            "slider_close_value": ParameterValue(
                LaunchConfiguration("slider_close_value"), value_type=float
            ),
            "calibration_angle_points_csv": LaunchConfiguration(
                "calibration_angle_points_csv"
            ),
            "calibration_slider_points_csv": LaunchConfiguration(
                "calibration_slider_points_csv"
            ),
            "slider_valid_min": ParameterValue(
                LaunchConfiguration("slider_valid_min"), value_type=float
            ),
            "slider_valid_max": ParameterValue(
                LaunchConfiguration("slider_valid_max"), value_type=float
            ),
            "open_slider_stop_band": ParameterValue(
                LaunchConfiguration("open_slider_stop_band"), value_type=float
            ),
            "close_slider_stop_band": ParameterValue(
                LaunchConfiguration("close_slider_stop_band"), value_type=float
            ),
            "close_slider_end_margin": ParameterValue(
                LaunchConfiguration("close_slider_end_margin"),
                value_type=float,
            ),
            "slider_move_band": ParameterValue(
                LaunchConfiguration("slider_move_band"), value_type=float
            ),
            "slider_jump_reject": ParameterValue(
                LaunchConfiguration("slider_jump_reject"), value_type=float
            ),
            "input_timeout_sec": ParameterValue(
                LaunchConfiguration("input_timeout_sec"), value_type=float
            ),
            "system_ready_timeout_sec": ParameterValue(
                LaunchConfiguration("system_ready_timeout_sec"),
                value_type=float,
            ),
            "max_continuous_run_sec": ParameterValue(
                LaunchConfiguration("max_continuous_run_sec"), value_type=float
            ),
            "require_closed_before_start": ParameterValue(
                LaunchConfiguration("require_closed_before_start"),
                value_type=bool,
            ),
            "startup_closed_angle_margin_deg": ParameterValue(
                LaunchConfiguration("startup_closed_angle_margin_deg"),
                value_type=float,
            ),
            "startup_confirm_count": ParameterValue(
                LaunchConfiguration("startup_confirm_count"), value_type=int
            ),
        }],
    )

    return LaunchDescription(
        arguments
        + [
            astra_rgbd,
            # Arduinoの再起動待ちを含め、EPOSより十分早く開始する。
            TimerAction(period=2.0, actions=[slider_node]),
            TimerAction(
                period=1.0,
                actions=[OpaqueFunction(function=_launch_flir_node)],
            ),
            TimerAction(period=5.0, actions=[hand_node]),
            TimerAction(period=6.0, actions=[finger_motion_map_node]),
            TimerAction(
                period=5.0,
                actions=[fusion_node, angle_gate_node],
            ),
            TimerAction(period=8.0, actions=[epos_node]),
        ]
    )
