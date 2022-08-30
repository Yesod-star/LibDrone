using System;
using System.Resources;
using System.Runtime.InteropServices;
//using LibDroneX;
using System.Threading;
using System.Text;
using System.Net.Sockets;
using System.Net;
using System.IO;
using System.ComponentModel;

public partial class MAVLink
{


    /// extensions_start 0
    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 144)]
    ///<summary> Information about a low level gimbal. This message should be requested by the gimbal manager or a ground station using MAV_CMD_REQUEST_MESSAGE. The maximum angles and rates are the limits by hardware. However, the limits by software used are likely different/smaller and dependent on mode/settings/etc.. </summary>
    public struct mavlink_gimbal_device_information_t
    {
        public mavlink_gimbal_device_information_t(ulong uid, uint time_boot_ms, uint firmware_version, uint hardware_version, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max,/*GIMBAL_DEVICE_CAP_FLAGS*/ushort cap_flags, ushort custom_cap_flags, byte[] vendor_name, byte[] model_name, byte[] custom_name)
        {
            this.uid = uid;
            this.time_boot_ms = time_boot_ms;
            this.firmware_version = firmware_version;
            this.hardware_version = hardware_version;
            this.roll_min = roll_min;
            this.roll_max = roll_max;
            this.pitch_min = pitch_min;
            this.pitch_max = pitch_max;
            this.yaw_min = yaw_min;
            this.yaw_max = yaw_max;
            this.cap_flags = cap_flags;
            this.custom_cap_flags = custom_cap_flags;
            this.vendor_name = vendor_name;
            this.model_name = model_name;
            this.custom_name = custom_name;

        }

        /// <summary>UID of gimbal hardware (0 if unknown).   </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("UID of gimbal hardware (0 if unknown).")]
        //[FieldOffset(0)]
        public ulong uid;

        /// <summary>Timestamp (time since system boot).  [ms] </summary>
        [LibDrone.Frames.Text.Units("[ms]")]
        [Description("Timestamp (time since system boot).")]
        //[FieldOffset(8)]
        public uint time_boot_ms;

        /// <summary>Version of the gimbal firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).   </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Version of the gimbal firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).")]
        //[FieldOffset(12)]
        public uint firmware_version;

        /// <summary>Version of the gimbal hardware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).   </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Version of the gimbal hardware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).")]
        //[FieldOffset(16)]
        public uint hardware_version;

        /// <summary>Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)  [rad] </summary>
        [LibDrone.Frames.Text.Units("[rad]")]
        [Description("Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)")]
        //[FieldOffset(20)]
        public float roll_min;

        /// <summary>Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)  [rad] </summary>
        [LibDrone.Frames.Text.Units("[rad]")]
        [Description("Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)")]
        //[FieldOffset(24)]
        public float roll_max;

        /// <summary>Minimum hardware pitch angle (positive: up, negative: down)  [rad] </summary>
        [LibDrone.Frames.Text.Units("[rad]")]
        [Description("Minimum hardware pitch angle (positive: up, negative: down)")]
        //[FieldOffset(28)]
        public float pitch_min;

        /// <summary>Maximum hardware pitch angle (positive: up, negative: down)  [rad] </summary>
        [LibDrone.Frames.Text.Units("[rad]")]
        [Description("Maximum hardware pitch angle (positive: up, negative: down)")]
        //[FieldOffset(32)]
        public float pitch_max;

        /// <summary>Minimum hardware yaw angle (positive: to the right, negative: to the left)  [rad] </summary>
        [LibDrone.Frames.Text.Units("[rad]")]
        [Description("Minimum hardware yaw angle (positive: to the right, negative: to the left)")]
        //[FieldOffset(36)]
        public float yaw_min;

        /// <summary>Maximum hardware yaw angle (positive: to the right, negative: to the left)  [rad] </summary>
        [LibDrone.Frames.Text.Units("[rad]")]
        [Description("Maximum hardware yaw angle (positive: to the right, negative: to the left)")]
        //[FieldOffset(40)]
        public float yaw_max;

        /// <summary>Bitmap of gimbal capability flags. GIMBAL_DEVICE_CAP_FLAGS  bitmask</summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Bitmap of gimbal capability flags.")]
        //[FieldOffset(44)]
        public  /*GIMBAL_DEVICE_CAP_FLAGS*/ushort cap_flags;

        /// <summary>Bitmap for use for gimbal-specific capability flags.   bitmask</summary>
        [LibDrone.Frames.Text.Units("")]

        [Description("Bitmap for use for gimbal-specific capability flags.")]
        //[FieldOffset(46)]
        public ushort custom_cap_flags;

        /// <summary>Name of the gimbal vendor.   </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Name of the gimbal vendor.")]
        //[FieldOffset(48)]
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
        public byte[] vendor_name;

        /// <summary>Name of the gimbal model.   </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Name of the gimbal model.")]
        //[FieldOffset(80)]
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
        public byte[] model_name;

        /// <summary>Custom name of the gimbal given to it by the user.   </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Custom name of the gimbal given to it by the user.")]
        //[FieldOffset(112)]
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
        public byte[] custom_name;
    };



    ///<summary> Co-ordinate frames used by MAVLink. Not all frames are supported by all commands, messages, or vehicles.              Global frames use the following naming conventions:       - 'GLOBAL': Global co-ordinate frame with WGS84 latitude/longitude and altitude positive over mean sea level (MSL) by default.          The following modifiers may be used with 'GLOBAL':         - 'RELATIVE_ALT': Altitude is relative to the vehicle home position rather than MSL.         - 'TERRAIN_ALT': Altitude is relative to ground level rather than MSL.         - 'INT': Latitude/longitude (in degrees) are scaled by multiplying by 1E7.        Local frames use the following naming conventions:       - 'LOCAL': Origin of local frame is fixed relative to earth. Unless otherwise specified this origin is the origin of the vehicle position-estimator ('EKF').       - 'BODY': Origin of local frame travels with the vehicle. NOTE, 'BODY' does NOT indicate alignment of frame axis with vehicle attitude.       - 'OFFSET': Deprecated synonym for 'BODY' (origin travels with the vehicle). Not to be used for new frames.        Some deprecated frames do not follow these conventions (e.g. MAV_FRAME_BODY_NED and MAV_FRAME_BODY_OFFSET_NED).   </summary>
    public enum MAV_FRAME : byte
    {
        ///<summary> Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL). | </summary>
        [Description("Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL).")]
        GLOBAL = 0,
        ///<summary> NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth. | </summary>
        [Description("NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.")]
        LOCAL_NED = 1,
        ///<summary> NOT a coordinate frame, indicates a mission command. | </summary>
        [Description("NOT a coordinate frame, indicates a mission command.")]
        MISSION = 2,
        ///<summary> Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. | </summary>
        [Description("Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.")]
        GLOBAL_RELATIVE_ALT = 3,
        ///<summary> ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth. | </summary>
        [Description("ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.")]
        LOCAL_ENU = 4,
        ///<summary> Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude over mean sea level (MSL). | </summary>
        [Description("Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude over mean sea level (MSL).")]
        GLOBAL_INT = 5,
        ///<summary> Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude with 0 being at the altitude of the home location. | </summary>
        [Description("Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude with 0 being at the altitude of the home location.")]
        GLOBAL_RELATIVE_ALT_INT = 6,
        ///<summary> NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle. | </summary>
        [Description("NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle.")]
        LOCAL_OFFSET_NED = 7,
        ///<summary> Same as MAV_FRAME_LOCAL_NED when used to represent position values. Same as MAV_FRAME_BODY_FRD when used with velocity/accelaration values. | </summary>
        [Description("Same as MAV_FRAME_LOCAL_NED when used to represent position values. Same as MAV_FRAME_BODY_FRD when used with velocity/accelaration values.")]
        [Obsolete]
        BODY_NED = 8,
        ///<summary> This is the same as MAV_FRAME_BODY_FRD. | </summary>
        [Description("This is the same as MAV_FRAME_BODY_FRD.")]
        [Obsolete]
        BODY_OFFSET_NED = 9,
        ///<summary> Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | </summary>
        [Description("Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.")]
        GLOBAL_TERRAIN_ALT = 10,
        ///<summary> Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | </summary>
        [Description("Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.")]
        GLOBAL_TERRAIN_ALT_INT = 11,
        ///<summary> FRD local tangent frame (x: Forward, y: Right, z: Down) with origin that travels with vehicle. The forward axis is aligned to the front of the vehicle in the horizontal plane. | </summary>
        [Description("FRD local tangent frame (x: Forward, y: Right, z: Down) with origin that travels with vehicle. The forward axis is aligned to the front of the vehicle in the horizontal plane.")]
        BODY_FRD = 12,
        ///<summary> MAV_FRAME_BODY_FLU - Body fixed frame of reference, Z-up (x: Forward, y: Left, z: Up). | </summary>
        [Description("MAV_FRAME_BODY_FLU - Body fixed frame of reference, Z-up (x: Forward, y: Left, z: Up).")]
        [Obsolete]
        RESERVED_13 = 13,
        ///<summary> MAV_FRAME_MOCAP_NED - Odometry local coordinate frame of data given by a motion capture system, Z-down (x: North, y: East, z: Down). | </summary>
        [Description("MAV_FRAME_MOCAP_NED - Odometry local coordinate frame of data given by a motion capture system, Z-down (x: North, y: East, z: Down).")]
        [Obsolete]
        RESERVED_14 = 14,
        ///<summary> MAV_FRAME_MOCAP_ENU - Odometry local coordinate frame of data given by a motion capture system, Z-up (x: East, y: North, z: Up). | </summary>
        [Description("MAV_FRAME_MOCAP_ENU - Odometry local coordinate frame of data given by a motion capture system, Z-up (x: East, y: North, z: Up).")]
        [Obsolete]
        RESERVED_15 = 15,
        ///<summary> MAV_FRAME_VISION_NED - Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: North, y: East, z: Down). | </summary>
        [Description("MAV_FRAME_VISION_NED - Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: North, y: East, z: Down).")]
        [Obsolete]
        RESERVED_16 = 16,
        ///<summary> MAV_FRAME_VISION_ENU - Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: East, y: North, z: Up). | </summary>
        [Description("MAV_FRAME_VISION_ENU - Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: East, y: North, z: Up).")]
        [Obsolete]
        RESERVED_17 = 17,
        ///<summary> MAV_FRAME_ESTIM_NED - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: North, y: East, z: Down). | </summary>
        [Description("MAV_FRAME_ESTIM_NED - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: North, y: East, z: Down).")]
        [Obsolete]
        RESERVED_18 = 18,
        ///<summary> MAV_FRAME_ESTIM_ENU - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: East, y: North, z: Up). | </summary>
        [Description("MAV_FRAME_ESTIM_ENU - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: East, y: North, z: Up).")]
        [Obsolete]
        RESERVED_19 = 19,
        ///<summary> FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane. | </summary>
        [Description("FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane.")]
        LOCAL_FRD = 20,
        ///<summary> FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane. | </summary>
        [Description("FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane.")]
        LOCAL_FLU = 21

    };

    /// extensions_start 6
    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 167)]
    ///<summary> Obstacle distances in front of the sensor, starting from the left in increment degrees to the right </summary>
    public struct mavlink_obstacle_distance_t
    {
        public mavlink_obstacle_distance_t(ulong time_usec, ushort[] distances, ushort min_distance, ushort max_distance,/*MAV_DISTANCE_SENSOR*/byte sensor_type, byte increment, float increment_f, float angle_offset,/*MAV_FRAME*/byte frame)
        {
            this.time_usec = time_usec;
            this.distances = distances;
            this.min_distance = min_distance;
            this.max_distance = max_distance;
            this.sensor_type = sensor_type;
            this.increment = increment;
            this.increment_f = increment_f;
            this.angle_offset = angle_offset;
            this.frame = frame;

        }

        /// <summary>Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.  [us] </summary>
        [LibDrone.Frames.Text.Units("[us]")]
        [Description("Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.")]
        //[FieldOffset(0)]
        public ulong time_usec;

        /// <summary>Distance of obstacles around the vehicle with index 0 corresponding to north + angle_offset, unless otherwise specified in the frame. A value of 0 is valid and means that the obstacle is practically touching the sensor. A value of max_distance +1 means no obstacle is present. A value of UINT16_MAX for unknown/not used. In a array element, one unit corresponds to 1cm.  [cm] </summary>
        [LibDrone.Frames.Text.Units("[cm]")]
        [Description("Distance of obstacles around the vehicle with index 0 corresponding to north + angle_offset, unless otherwise specified in the frame. A value of 0 is valid and means that the obstacle is practically touching the sensor. A value of max_distance +1 means no obstacle is present. A value of UINT16_MAX for unknown/not used. In a array element, one unit corresponds to 1cm.")]
        //[FieldOffset(8)]
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 72)]
        public ushort[] distances;

        /// <summary>Minimum distance the sensor can measure.  [cm] </summary>
        [LibDrone.Frames.Text.Units("[cm]")]
        [Description("Minimum distance the sensor can measure.")]
        //[FieldOffset(152)]
        public ushort min_distance;

        /// <summary>Maximum distance the sensor can measure.  [cm] </summary>
        [LibDrone.Frames.Text.Units("[cm]")]
        [Description("Maximum distance the sensor can measure.")]
        //[FieldOffset(154)]
        public ushort max_distance;

        /// <summary>Class id of the distance sensor type. MAV_DISTANCE_SENSOR  </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Class id of the distance sensor type.")]
        //[FieldOffset(156)]
        public  /*MAV_DISTANCE_SENSOR*/byte sensor_type;

        /// <summary>Angular width in degrees of each array element. Increment direction is clockwise. This field is ignored if increment_f is non-zero.  [deg] </summary>
        [LibDrone.Frames.Text.Units("[deg]")]
        [Description("Angular width in degrees of each array element. Increment direction is clockwise. This field is ignored if increment_f is non-zero.")]
        //[FieldOffset(157)]
        public byte increment;

        /// <summary>Angular width in degrees of each array element as a float. If non-zero then this value is used instead of the uint8_t increment field. Positive is clockwise direction, negative is counter-clockwise.  [deg] </summary>
        [LibDrone.Frames.Text.Units("[deg]")]
        [Description("Angular width in degrees of each array element as a float. If non-zero then this value is used instead of the uint8_t increment field. Positive is clockwise direction, negative is counter-clockwise.")]
        //[FieldOffset(158)]
        public float increment_f;

        /// <summary>Relative angle offset of the 0-index element in the distances array. Value of 0 corresponds to forward. Positive is clockwise direction, negative is counter-clockwise.  [deg] </summary>
        [LibDrone.Frames.Text.Units("[deg]")]
        [Description("Relative angle offset of the 0-index element in the distances array. Value of 0 corresponds to forward. Positive is clockwise direction, negative is counter-clockwise.")]
        //[FieldOffset(162)]
        public float angle_offset;

        /// <summary>Coordinate frame of reference for the yaw rotation and offset of the sensor data. Defaults to MAV_FRAME_GLOBAL, which is north aligned. For body-mounted sensors use MAV_FRAME_BODY_FRD, which is vehicle front aligned. MAV_FRAME  </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Coordinate frame of reference for the yaw rotation and offset of the sensor data. Defaults to MAV_FRAME_GLOBAL, which is north aligned. For body-mounted sensors use MAV_FRAME_BODY_FRD, which is vehicle front aligned.")]
        //[FieldOffset(166)]
        public  /*MAV_FRAME*/byte frame;
    };

    /// extensions_start 8
    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 39)]
    ///<summary> Distance sensor information for an onboard rangefinder. </summary>
    public struct mavlink_distance_sensor_t
    {
        public mavlink_distance_sensor_t(uint time_boot_ms, ushort min_distance, ushort max_distance, ushort current_distance,/*MAV_DISTANCE_SENSOR*/byte type, byte id,/*MAV_SENSOR_ORIENTATION*/byte orientation, byte covariance, float horizontal_fov, float vertical_fov, float[] quaternion, byte signal_quality)
        {
            this.time_boot_ms = time_boot_ms;
            this.min_distance = min_distance;
            this.max_distance = max_distance;
            this.current_distance = current_distance;
            this.type = type;
            this.id = id;
            this.orientation = orientation;
            this.covariance = covariance;
            this.horizontal_fov = horizontal_fov;
            this.vertical_fov = vertical_fov;
            this.quaternion = quaternion;
            this.signal_quality = signal_quality;

        }

        /// <summary>Timestamp (time since system boot).  [ms] </summary>
        [LibDrone.Frames.Text.Units("[ms]")]
        [Description("Timestamp (time since system boot).")]
        //[FieldOffset(0)]
        public uint time_boot_ms;

        /// <summary>Minimum distance the sensor can measure  [cm] </summary>
        [LibDrone.Frames.Text.Units("[cm]")]
        [Description("Minimum distance the sensor can measure")]
        //[FieldOffset(4)]
        public ushort min_distance;

        /// <summary>Maximum distance the sensor can measure  [cm] </summary>
        [LibDrone.Frames.Text.Units("[cm]")]
        [Description("Maximum distance the sensor can measure")]
        //[FieldOffset(6)]
        public ushort max_distance;

        /// <summary>Current distance reading  [cm] </summary>
        [LibDrone.Frames.Text.Units("[cm]")]
        [Description("Current distance reading")]
        //[FieldOffset(8)]
        public ushort current_distance;

        /// <summary>Type of distance sensor. MAV_DISTANCE_SENSOR  </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Type of distance sensor.")]
        //[FieldOffset(10)]
        public  /*MAV_DISTANCE_SENSOR*/byte type;

        /// <summary>Onboard ID of the sensor   </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Onboard ID of the sensor")]
        //[FieldOffset(11)]
        public byte id;

        /// <summary>Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270 MAV_SENSOR_ORIENTATION  </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270")]
        //[FieldOffset(12)]
        public  /*MAV_SENSOR_ORIENTATION*/byte orientation;

        /// <summary>Measurement variance. Max standard deviation is 6cm. 255 if unknown.  [cm^2] </summary>
        [LibDrone.Frames.Text.Units("[cm^2]")]
        [Description("Measurement variance. Max standard deviation is 6cm. 255 if unknown.")]
        //[FieldOffset(13)]
        public byte covariance;

        /// <summary>Horizontal Field of View (angle) where the distance measurement is valid and the field of view is known. Otherwise this is set to 0.  [rad] </summary>
        [LibDrone.Frames.Text.Units("[rad]")]
        [Description("Horizontal Field of View (angle) where the distance measurement is valid and the field of view is known. Otherwise this is set to 0.")]
        //[FieldOffset(14)]
        public float horizontal_fov;

        /// <summary>Vertical Field of View (angle) where the distance measurement is valid and the field of view is known. Otherwise this is set to 0.  [rad] </summary>
        [LibDrone.Frames.Text.Units("[rad]")]
        [Description("Vertical Field of View (angle) where the distance measurement is valid and the field of view is known. Otherwise this is set to 0.")]
        //[FieldOffset(18)]
        public float vertical_fov;

        /// <summary>Quaternion of the sensor orientation in vehicle body frame (w, x, y, z order, zero-rotation is 1, 0, 0, 0). Zero-rotation is along the vehicle body x-axis. This field is required if the orientation is set to MAV_SENSOR_ROTATION_CUSTOM. Set it to 0 if invalid.'   </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Quaternion of the sensor orientation in vehicle body frame (w, x, y, z order, zero-rotation is 1, 0, 0, 0). Zero-rotation is along the vehicle body x-axis. This field is required if the orientation is set to MAV_SENSOR_ROTATION_CUSTOM. Set it to 0 if invalid.'")]
        //[FieldOffset(22)]
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public float[] quaternion;

        /// <summary>Signal quality of the sensor. Specific to each sensor type, representing the relation of the signal strength with the target reflectivity, distance, size or aspect, but normalised as a percentage. 0 = unknown/unset signal quality, 1 = invalid signal, 100 = perfect signal.  [%] </summary>
        [LibDrone.Frames.Text.Units("[%]")]
        [Description("Signal quality of the sensor. Specific to each sensor type, representing the relation of the signal strength with the target reflectivity, distance, size or aspect, but normalised as a percentage. 0 = unknown/unset signal quality, 1 = invalid signal, 100 = perfect signal.")]
        //[FieldOffset(38)]
        public byte signal_quality;
    };

    public enum MAV_CMD : ushort
    {
        ///<summary> Navigate to waypoint. |Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)| Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)| 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude| Longitude| Altitude|  </summary>
        [Description("Navigate to waypoint.")]
        WAYPOINT = 16,
        ///<summary> Loiter around this waypoint an unlimited amount of time |Empty| Empty| Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise| Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude| Longitude| Altitude|  </summary>
        [Description("Loiter around this waypoint an unlimited amount of time")]
        LOITER_UNLIM = 17,
        ///<summary> Loiter around this waypoint for X turns |Number of turns.| Empty| Radius around waypoint. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude| Longitude| Altitude|  </summary>
        [Description("Loiter around this waypoint for X turns")]
        LOITER_TURNS = 18,
        ///<summary> Loiter around this waypoint for X seconds |Loiter time.| Empty| Radius around waypoint. If positive loiter clockwise, else counter-clockwise.| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle.  NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude| Longitude| Altitude|  </summary>
        [Description("Loiter around this waypoint for X seconds")]
        LOITER_TIME = 19,
        ///<summary> Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Return to launch location")]
        RETURN_TO_LAUNCH = 20,
        ///<summary> Land at location. |Minimum target altitude if landing is aborted (0 = undefined/use system default).| Precision land mode.| Empty.| Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude.| Longitude.| Landing altitude (ground level in current frame).|  </summary>
        [Description("Land at location.")]
        LAND = 21,
        ///<summary> Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode. |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude| Longitude| Altitude|  </summary>
        [Description("Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode.")]
        TAKEOFF = 22,
        ///<summary> Land at local position (local frame only) |Landing target number (if available)| Maximum accepted offset from desired landing position - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land| Landing descend rate| Desired yaw angle| Y-axis position| X-axis position| Z-axis / ground level position|  </summary>
        [Description("Land at local position (local frame only)")]
        LAND_LOCAL = 23,
        ///<summary> Takeoff from local position (local frame only) |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Takeoff ascend rate| Yaw angle (if magnetometer or another yaw estimation source present), ignored without one of these| Y-axis position| X-axis position| Z-axis position|  </summary>
        [Description("Takeoff from local position (local frame only)")]
        TAKEOFF_LOCAL = 24,
        ///<summary> Vehicle following, i.e. this waypoint represents the position of a moving vehicle |Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation| Ground speed of vehicle to be followed| Radius around waypoint. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  </summary>
        [Description("Vehicle following, i.e. this waypoint represents the position of a moving vehicle")]
        FOLLOW = 25,
        ///<summary> Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. |Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.| Empty| Empty| Empty| Empty| Empty| Desired altitude|  </summary>
        [Description("Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached.")]
        CONTINUE_AND_CHANGE_ALT = 30,
        ///<summary> Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached. Additionally, if the Heading Required parameter is non-zero the aircraft will not leave the loiter until heading toward the next waypoint. |ng Required (0 = False)| Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.| Empty| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location| Latitude| Longitude| Altitude|  </summary>
        [Description("Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached. Additionally, if the Heading Required parameter is non-zero the aircraft will not leave the loiter until heading toward the next waypoint.")]
        LOITER_TO_ALT = 31,
        ///<summary> Begin following a target |System ID (of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode.| Reserved| Reserved| Altitude mode: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home.| Altitude above home. (used if mode=2)| Reserved| Time to land in which the MAV should go to the default position hold mode after a message RX timeout.|  </summary>
        [Description("Begin following a target")]
        DO_FOLLOW = 32,
        ///<summary> Reposition the MAV after a follow target command has been sent |Camera q1 (where 0 is on the ray from the camera to the tracking device)| Camera q2| Camera q3| Camera q4| altitude offset from target| X offset from target| Y offset from target|  </summary>
        [Description("Reposition the MAV after a follow target command has been sent")]
        DO_FOLLOW_REPOSITION = 33,
        ///<summary> Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of interest mode.| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  </summary>
        [Description("Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras.")]
        [Obsolete]
        ROI = 80,
        ///<summary> Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| le and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  </summary>
        [Description("Control autonomous path planning on the MAV.")]
        PATHPLANNING = 81,
        ///<summary> Navigate to waypoint using a spline path. |Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)| Empty| Empty| Empty| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  </summary>
        [Description("Navigate to waypoint using a spline path.")]
        SPLINE_WAYPOINT = 82,
        ///<summary> Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up. |Altitude.| Descent speed.| How long to wiggle the control surfaces to prevent them seizing up.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up.")]
        ALTITUDE_WAIT = 83,
        ///<summary> Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The command should be ignored by vehicles that dont support both VTOL and fixed-wing flight (multicopters, boats,etc.). |Empty| Front transition heading.| Empty| Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude| Longitude| Altitude|  </summary>
        [Description("Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The command should be ignored by vehicles that dont support both VTOL and fixed-wing flight (multicopters, boats,etc.).")]
        VTOL_TAKEOFF = 84,
        ///<summary> Land using VTOL mode |Empty| Empty| Approach altitude (with the same reference as the Altitude field). NaN if unspecified.| Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude| Longitude| Altitude (ground level)|  </summary>
        [Description("Land using VTOL mode")]
        VTOL_LAND = 85,
        ///<summary> hand control over to an external controller |0.5f on)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("hand control over to an external controller")]
        GUIDED_ENABLE = 92,
        ///<summary> Delay the next navigation command a number of seconds or until a specified time |Delay (-1 to enable time-of-day fields)| hour (24h format, UTC, -1 to ignore)| minute (24h format, UTC, -1 to ignore)| second (24h format, UTC, -1 to ignore)| Empty| Empty| Empty|  </summary>
        [Description("Delay the next navigation command a number of seconds or until a specified time")]
        DELAY = 93,
        ///<summary> Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload. |Maximum distance to descend.| Empty| Empty| Empty| Latitude| Longitude| Altitude|  </summary>
        [Description("Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload.")]
        PAYLOAD_PLACE = 94,
        ///<summary> NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration")]
        LAST = 95,
        ///<summary> Delay mission state machine. |Delay| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Delay mission state machine.")]
        CONDITION_DELAY = 112,
        ///<summary> Ascend/descend to target altitude at specified rate. Delay mission state machine until desired altitude reached. |Descent / Ascend rate.| Empty| Empty| Empty| Empty| Empty| Target Altitude|  </summary>
        [Description("Ascend/descend to target altitude at specified rate. Delay mission state machine until desired altitude reached.")]
        CONDITION_CHANGE_ALT = 113,
        ///<summary> Delay mission state machine until within desired distance of next NAV point. |Distance.| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Delay mission state machine until within desired distance of next NAV point.")]
        CONDITION_DISTANCE = 114,
        ///<summary> Reach a certain target angle. |target angle, 0 is north| angular speed| direction: -1: counter clockwise, 1: clockwise| 0: absolute angle, 1: relative offset| Empty| Empty| Empty|  </summary>
        [Description("Reach a certain target angle.")]
        CONDITION_YAW = 115,
        ///<summary> NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration")]
        CONDITION_LAST = 159,
        ///<summary> Set system mode. |Mode| Custom mode - this is system specific, please refer to the individual autopilot specifications for details.| Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.| Empty| Empty| Empty| Empty|  </summary>
        [Description("Set system mode.")]
        DO_SET_MODE = 176,
        ///<summary> Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Jump to the desired command in the mission list.  Repeat this action only the specified number of times")]
        DO_JUMP = 177,
        ///<summary> Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)| Speed (-1 indicates no change)| Throttle (-1 indicates no change)| 0: absolute, 1: relative| Empty| Empty| Empty|  </summary>
        [Description("Change speed and/or throttle set points.")]
        DO_CHANGE_SPEED = 178,
        ///<summary> Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  </summary>
        [Description("Changes the home location either to the current location or a specified location.")]
        DO_SET_HOME = 179,
        ///<summary> Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.")]
        DO_SET_PARAMETER = 180,
        ///<summary> Set a relay to a condition. |Relay instance number.| Setting. (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Set a relay to a condition.")]
        DO_SET_RELAY = 181,
        ///<summary> Cycle a relay on and off for a desired number of cycles with a desired period. |Relay instance number.| Cycle count.| Cycle time.| Empty| Empty| Empty| Empty|  </summary>
        [Description("Cycle a relay on and off for a desired number of cycles with a desired period.")]
        DO_REPEAT_RELAY = 182,
        ///<summary> Set a servo to a desired PWM value. |Servo instance number.| Pulse Width Modulation.| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Set a servo to a desired PWM value.")]
        DO_SET_SERVO = 183,
        ///<summary> Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo instance number.| Pulse Width Modulation.| Cycle count.| Cycle time.| Empty| Empty| Empty|  </summary>
        [Description("Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.")]
        DO_REPEAT_SERVO = 184,
        ///<summary> Terminate flight immediately |0.5| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Terminate flight immediately")]
        DO_FLIGHTTERMINATION = 185,
        ///<summary> Change altitude set point. |Altitude.| Frame of new altitude.| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Change altitude set point.")]
        DO_CHANGE_ALTITUDE = 186,
        ///<summary> Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence. |Empty| Empty| Empty| Empty| Latitude| Longitude| Empty|  </summary>
        [Description("Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence.")]
        DO_LAND_START = 189,
        ///<summary> Mission command to perform a landing from a rally point. |Break altitude| Landing speed| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Mission command to perform a landing from a rally point.")]
        DO_RALLY_LAND = 190,
        ///<summary> Mission command to safely abort an autonomous landing. |Altitude| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Mission command to safely abort an autonomous landing.")]
        DO_GO_AROUND = 191,
        ///<summary> Reposition the vehicle to a specific WGS84 global position. |Ground speed, less than 0 (-1) for default| Bitmask of option flags.| Loiter radius for planes. Positive values only, direction is controlled by Yaw value. A value of zero or NaN is ignored.| Yaw heading. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise)| Latitude| Longitude| Altitude|  </summary>
        [Description("Reposition the vehicle to a specific WGS84 global position.")]
        DO_REPOSITION = 192,
        ///<summary> If in a GPS controlled position mode, hold the current position or continue. |0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  </summary>
        [Description("If in a GPS controlled position mode, hold the current position or continue.")]
        DO_PAUSE_CONTINUE = 193,
        ///<summary> Set moving direction to forward or reverse. |Direction (0=Forward, 1=Reverse)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Set moving direction to forward or reverse.")]
        DO_SET_REVERSE = 194,
        ///<summary> Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Empty| Empty| Empty| Empty| Latitude of ROI location| Longitude of ROI location| Altitude of ROI location|  </summary>
        [Description("Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras.")]
        DO_SET_ROI_LOCATION = 195,
        ///<summary> Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Empty| Empty| Empty| Empty| Pitch offset from next waypoint, positive pitching up| Roll offset from next waypoint, positive rolling to the right| Yaw offset from next waypoint, positive yawing to the right|  </summary>
        [Description("Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras.")]
        DO_SET_ROI_WPNEXT_OFFSET = 196,
        ///<summary> Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras.")]
        DO_SET_ROI_NONE = 197,
        ///<summary> Mount tracks system with specified system ID. Determination of target vehicle position may be done with GLOBAL_POSITION_INT or any other means. |System ID| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Mount tracks system with specified system ID. Determination of target vehicle position may be done with GLOBAL_POSITION_INT or any other means.")]
        DO_SET_ROI_SYSID = 198,
        ///<summary> Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| 0: single images every n seconds| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  </summary>
        [Description("Control onboard camera system.")]
        DO_CONTROL_VIDEO = 200,
        ///<summary> Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of interest mode.| Waypoint index/ target ID (depends on param 1).| Region of interest index. (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  </summary>
        [Description("Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras.")]
        [Obsolete]
        DO_SET_ROI = 201,
        ///<summary> Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ). |Modes: P, TV, AV, M, Etc.| Shutter speed: Divisor number for one second.| Aperture: F stop number.| ISO number e.g. 80, 100, 200, Etc.| Exposure type enumerator.| Command Identity.| Main engine cut-off time before camera trigger. (0 means no cut-off)|  </summary>
        [Description("Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).")]
        DO_DIGICAM_CONFIGURE = 202,
        ///<summary> Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ). |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.|  </summary>
        [Description("Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).")]
        DO_DIGICAM_CONTROL = 203,
        ///<summary> Mission command to configure a camera or antenna mount |Mount operation mode| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|  </summary>
        [Description("Mission command to configure a camera or antenna mount")]
        DO_MOUNT_CONFIGURE = 204,
        ///<summary> Mission command to control a camera or antenna mount |pitch (WIP: DEPRECATED: or lat in degrees) depending on mount mode.| roll (WIP: DEPRECATED: or lon in degrees) depending on mount mode.| yaw (WIP: DEPRECATED: or alt in meters) depending on mount mode.| WIP: alt in meters depending on mount mode.| WIP: latitude in degrees * 1E7, set if appropriate mount mode.| WIP: longitude in degrees * 1E7, set if appropriate mount mode.| Mount mode.|  </summary>
        [Description("Mission command to control a camera or antenna mount")]
        DO_MOUNT_CONTROL = 205,
        ///<summary> Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera. |Camera trigger distance. 0 to stop triggering.| Camera shutter integration time. -1 or 0 to ignore| Trigger camera once immediately. (0 = no trigger, 1 = trigger)| Empty| Empty| Empty| Empty|  </summary>
        [Description("Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera.")]
        DO_SET_CAM_TRIGG_DIST = 206,
        ///<summary> Mission command to enable the geofence |enable? (0=disable, 1=enable, 2=disable_floor_only)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Mission command to enable the geofence")]
        DO_FENCE_ENABLE = 207,
        ///<summary> Mission item/command to release a parachute or enable/disable auto release. |Action| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Mission item/command to release a parachute or enable/disable auto release.")]
        DO_PARACHUTE = 208,
        ///<summary> Mission command to perform motor test. |Motor instance number. (from 1 to max number of motors on the vehicle)| Throttle type.| Throttle.| Timeout.| Motor count. (number of motors to test to test in sequence, waiting for the timeout above between them; 0=1 motor, 1=1 motor, 2=2 motors...)| Motor test order.| Empty|  </summary>
        [Description("Mission command to perform motor test.")]
        DO_MOTOR_TEST = 209,
        ///<summary> Change to/from inverted flight. |Inverted flight. (0=normal, 1=inverted)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Change to/from inverted flight.")]
        DO_INVERTED_FLIGHT = 210,
        ///<summary> Mission command to operate a gripper. |Gripper instance number.| Gripper action to perform.| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Mission command to operate a gripper.")]
        DO_GRIPPER = 211,
        ///<summary> Enable/disable autotune. |Enable (1: enable, 0:disable).| Specify which axes are autotuned. 0 indicates autopilot default settings.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Enable/disable autotune.")]
        DO_AUTOTUNE_ENABLE = 212,
        ///<summary> Sets a desired vehicle turn angle and speed change. |Yaw angle to adjust steering by.| Speed.| Final angle. (0=absolute, 1=relative)| Empty| Empty| Empty| Empty|  </summary>
        [Description("Sets a desired vehicle turn angle and speed change.")]
        SET_YAW_SPEED = 213,
        ///<summary> Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera. |Camera trigger cycle time. -1 or 0 to ignore.| Camera shutter integration time. Should be less than trigger cycle time. -1 or 0 to ignore.| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera.")]
        DO_SET_CAM_TRIGG_INTERVAL = 214,
        ///<summary> Set the distance to be repeated on mission resume |Distance.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Set the distance to be repeated on mission resume")]
        DO_SET_RESUME_REPEAT_DIST = 215,
        ///<summary> Control attached liquid sprayer |0: disable sprayer. 1: enable sprayer.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Control attached liquid sprayer")]
        DO_SPRAYER = 216,
        ///<summary> Pass instructions onto scripting, a script should be checking for a new command |uint16 ID value to be passed to scripting| float value to be passed to scripting| float value to be passed to scripting| float value to be passed to scripting| Empty.| Empty.| Empty.|  </summary>
        [Description("Pass instructions onto scripting, a script should be checking for a new command")]
        DO_SEND_SCRIPT_MESSAGE = 217,
        ///<summary> Execute auxiliary function |Auxiliary Function.| Switch Level.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Execute auxiliary function")]
        DO_AUX_FUNCTION = 218,
        ///<summary> Mission command to control a camera or antenna mount, using a quaternion as reference. |quaternion param q1, w (1 in null-rotation)| quaternion param q2, x (0 in null-rotation)| quaternion param q3, y (0 in null-rotation)| quaternion param q4, z (0 in null-rotation)| Empty| Empty| Empty|  </summary>
        [Description("Mission command to control a camera or antenna mount, using a quaternion as reference.")]
        DO_MOUNT_CONTROL_QUAT = 220,
        ///<summary> set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("set id of master controller")]
        DO_GUIDED_MASTER = 221,
        ///<summary> Set limits for external control |Timeout - maximum time that external controller will be allowed to control vehicle. 0 means no timeout.| Altitude (MSL) min - if vehicle moves below this alt, the command will be aborted and the mission will continue. 0 means no lower altitude limit.| Altitude (MSL) max - if vehicle moves above this alt, the command will be aborted and the mission will continue. 0 means no upper altitude limit.| Horizontal move limit - if vehicle moves more than this distance from its location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal move limit.| Empty| Empty| Empty|  </summary>
        [Description("Set limits for external control")]
        DO_GUIDED_LIMITS = 222,
        ///<summary> Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines |0: Stop engine, 1:Start Engine| 0: Warm start, 1:Cold start. Controls use of choke where applicable| Height delay. This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.| Empty| Empty| Empty| Empty|  </summary>
        [Description("Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines")]
        DO_ENGINE_CONTROL = 223,
        ///<summary> Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between). |Mission sequence value to set| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).")]
        DO_SET_MISSION_CURRENT = 224,
        ///<summary> NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("NOP - This command is only used to mark the upper limit of the DO commands in the enumeration")]
        DO_LAST = 240,
        ///<summary> Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero. |1: gyro calibration, 3: gyro temperature calibration| 1: magnetometer calibration| 1: ground pressure calibration| 1: radio RC calibration, 2: RC trim calibration| 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration| 1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration| 1: ESC calibration, 3: barometer temperature calibration|  </summary>
        [Description("Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero.")]
        PREFLIGHT_CALIBRATION = 241,
        ///<summary> Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  </summary>
        [Description("Set sensor offsets. This command will be only accepted if in pre-flight mode.")]
        PREFLIGHT_SET_SENSOR_OFFSETS = 242,
        ///<summary> Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to the legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during initial vehicle configuration (it is not a normal pre-flight command and has been poorly named). |1: Trigger actuator ID assignment and direction mapping. 0: Cancel command.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  </summary>
        [Description("Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to the legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during initial vehicle configuration (it is not a normal pre-flight command and has been poorly named).")]
        PREFLIGHT_UAVCAN = 243,
        ///<summary> Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| 1: logging rate (e.g. set to 1000 for 1000 Hz logging)| Reserved| Empty| Empty| Empty|  </summary>
        [Description("Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.")]
        PREFLIGHT_STORAGE = 245,
        ///<summary> Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.| WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded| WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded| Reserved (set to 0)| Reserved (set to 0)| WIP: ID (e.g. camera ID -1 for all IDs)|  </summary>
        [Description("Request the reboot or shutdown of system components.")]
        PREFLIGHT_REBOOT_SHUTDOWN = 246,
        ///<summary> Override current mission with command to pause mission, pause mission and move to position, continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether it holds in place or moves to another position. |MAV_GOTO_DO_HOLD: pause mission and either hold or move to specified position (depending on param2), MAV_GOTO_DO_CONTINUE: resume mission.| MAV_GOTO_HOLD_AT_CURRENT_POSITION: hold at current position, MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position.| Coordinate frame of hold point.| Desired yaw angle.| position.| Longitude/Y position.| Altitude/Z position.|  </summary>
        [Description("Override current mission with command to pause mission, pause mission and move to position, continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether it holds in place or moves to another position.")]
        OVERRIDE_GOTO = 252,
        ///<summary> Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this purpose). The camera is triggered each time this distance is exceeded, then the mount moves to the next position. Params 4~6 set-up the angle limits and number of positions for oblique survey, where mount-enabled vehicles automatically roll the camera between shots to emulate an oblique camera setup (providing an increased HFOV). This command can also be used to set the shutter integration time for the camera. |Camera trigger distance. 0 to stop triggering.| Camera shutter integration time. 0 to ignore| The minimum interval in which the camera is capable of taking subsequent pictures repeatedly. 0 to ignore.| Total number of roll positions at which the camera will capture photos (images captures spread evenly across the limits defined by param5).| Angle limits that the camera can be rolled to left and right of center.| Fixed pitch angle that the camera will hold in oblique mode if the mount is actuated in the pitch axis.| Empty|  </summary>
        [Description("Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this purpose). The camera is triggered each time this distance is exceeded, then the mount moves to the next position. Params 4~6 set-up the angle limits and number of positions for oblique survey, where mount-enabled vehicles automatically roll the camera between shots to emulate an oblique camera setup (providing an increased HFOV). This command can also be used to set the shutter integration time for the camera.")]
        OBLIQUE_SURVEY = 260,
        ///<summary> start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("start running a mission")]
        MISSION_START = 300,
        ///<summary> Arms / Disarms a component |0: disarm, 1: arm| 0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Arms / Disarms a component")]
        COMPONENT_ARM_DISARM = 400,
        ///<summary> Instructs system to run pre-arm checks.  This command should return MAV_RESULT_TEMPORARILY_REJECTED in the case the system is armed, otherwse MAV_RESULT_ACCEPTED.  Note that the return value from executing this command does not indicate whether the vehicle is armable or not, just whether the system has successfully run/is currently running the checks.  The result of the checks is reflected in the SYS_STATUS message. |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Instructs system to run pre-arm checks.  This command should return MAV_RESULT_TEMPORARILY_REJECTED in the case the system is armed, otherwse MAV_RESULT_ACCEPTED.  Note that the return value from executing this command does not indicate whether the vehicle is armable or not, just whether the system has successfully run/is currently running the checks.  The result of the checks is reflected in the SYS_STATUS message.")]
        RUN_PREARM_CHECKS = 401,
        ///<summary> Request the home position from the vehicle. |Reserved| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  </summary>
        [Description("Request the home position from the vehicle.")]
        GET_HOME_POSITION = 410,
        ///<summary> Starts receiver pairing. |0:Spektrum.| RC type.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Starts receiver pairing.")]
        START_RX_PAIR = 500,
        ///<summary> Request the interval between messages for a particular MAVLink message ID. The receiver should ACK the command and then emit its response in a MESSAGE_INTERVAL message. |The MAVLink message ID| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Request the interval between messages for a particular MAVLink message ID. The receiver should ACK the command and then emit its response in a MESSAGE_INTERVAL message.")]
        GET_MESSAGE_INTERVAL = 510,
        ///<summary> Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM. |The MAVLink message ID| The interval between two messages. Set to -1 to disable and 0 to request default rate.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.|  </summary>
        [Description("Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM.")]
        SET_MESSAGE_INTERVAL = 511,
        ///<summary> Request the target system(s) emit a single instance of a specified message (i.e. a 'one-shot' version of MAV_CMD_SET_MESSAGE_INTERVAL). |The MAVLink message ID of the requested message.| d. Otherwise, the use of this parameter (if any) must be defined in the requested message. By default assumed not used (0).| The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| Target address for requested message (if message has target address fields). 0: Flight-stack default, 1: address of requestor, 2: broadcast.|  </summary>
        [Description("Request the target system(s) emit a single instance of a specified message (i.e. a 'one-shot' version of MAV_CMD_SET_MESSAGE_INTERVAL).")]
        REQUEST_MESSAGE = 512,
        ///<summary> Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit their capabilities in an PROTOCOL_VERSION message |1: Request supported protocol versions by all nodes on the network| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit their capabilities in an PROTOCOL_VERSION message")]
        [Obsolete]
        REQUEST_PROTOCOL_VERSION = 519,
        ///<summary> Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in an AUTOPILOT_VERSION message |1: Request autopilot version| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in an AUTOPILOT_VERSION message")]
        [Obsolete]
        REQUEST_AUTOPILOT_CAPABILITIES = 520,
        ///<summary> Request camera information (CAMERA_INFORMATION). |0: No action 1: Request camera capabilities| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Request camera information (CAMERA_INFORMATION).")]
        [Obsolete]
        REQUEST_CAMERA_INFORMATION = 521,
        ///<summary> Request camera settings (CAMERA_SETTINGS). |0: No Action 1: Request camera settings| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Request camera settings (CAMERA_SETTINGS).")]
        [Obsolete]
        REQUEST_CAMERA_SETTINGS = 522,
        ///<summary> Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage. |Storage ID (0 for all, 1 for first, 2 for second, etc.)| 0: No Action 1: Request storage information| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage.")]
        [Obsolete]
        REQUEST_STORAGE_INFORMATION = 525,
        ///<summary> Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage. |Storage ID (1 for first, 2 for second, etc.)| 0: No action 1: Format storage| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage.")]
        STORAGE_FORMAT = 526,
        ///<summary> Request camera capture status (CAMERA_CAPTURE_STATUS) |0: No Action 1: Request camera capture status| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Request camera capture status (CAMERA_CAPTURE_STATUS)")]
        [Obsolete]
        REQUEST_CAMERA_CAPTURE_STATUS = 527,
        ///<summary> Request flight information (FLIGHT_INFORMATION) |1: Request flight information| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Request flight information (FLIGHT_INFORMATION)")]
        [Obsolete]
        REQUEST_FLIGHT_INFORMATION = 528,
        ///<summary> Reset all camera settings to Factory Default |0: No Action 1: Reset all settings| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Reset all camera settings to Factory Default")]
        RESET_CAMERA_SETTINGS = 529,
        ///<summary> Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming. |Reserved (Set to 0)| Camera mode| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:0)| Reserved (default:0)| Reserved (default:NaN)|  </summary>
        [Description("Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming.")]
        SET_CAMERA_MODE = 530,
        ///<summary> Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG. |Tag.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG.")]
        JUMP_TAG = 600,
        ///<summary> Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number. |Target tag to jump to.| Repeat count.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number.")]
        DO_JUMP_TAG = 601,
        ///<summary> High level setpoint to be sent to a gimbal manager to set a gimbal attitude. It is possible to set combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used to signal unset. Note: a gimbal is never to react to this command but only the gimbal manager. |Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for LOCK mode).| Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK mode).| Pitch rate (positive to pitch up).| Yaw rate (positive to yaw to the right).| Gimbal manager flags to use.| Reserved (default:0)| Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).|  </summary>
        [Description("High level setpoint to be sent to a gimbal manager to set a gimbal attitude. It is possible to set combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used to signal unset. Note: a gimbal is never to react to this command but only the gimbal manager.")]
        [Obsolete]
        DO_GIMBAL_MANAGER_PITCHYAW = 1000,
        ///<summary> Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values. |Reserved (Set to 0)| Desired elapsed time between two consecutive pictures (in seconds). Minimum values depend on hardware (typically greater than 2 seconds).| Total number of images to capture. 0 to capture forever/until MAV_CMD_IMAGE_STOP_CAPTURE.| ID for each capture command to prevent double captures when a command is re-transmitted.| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:NaN)|  </summary>
        [Description("Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values.")]
        IMAGE_START_CAPTURE = 2000,
        ///<summary> Stop image capture sequence Use NaN for reserved values. |Reserved (Set to 0)| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:0)| Reserved (default:0)| Reserved (default:NaN)|  </summary>
        [Description("Stop image capture sequence Use NaN for reserved values.")]
        IMAGE_STOP_CAPTURE = 2001,
        ///<summary> Enable or disable on-board camera triggering system. |Trigger enable/disable (0 for disable, 1 for start), -1 to ignore| 1 to reset the trigger sequence, -1 or 0 to ignore| 1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Enable or disable on-board camera triggering system.")]
        DO_TRIGGER_CONTROL = 2003,
        ///<summary> Starts video capture (recording). |Video Stream ID (0 for all streams)| Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency)| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:NaN)|  </summary>
        [Description("Starts video capture (recording).")]
        VIDEO_START_CAPTURE = 2500,
        ///<summary> Stop the current video capture (recording). |Video Stream ID (0 for all streams)| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:NaN)|  </summary>
        [Description("Stop the current video capture (recording).")]
        VIDEO_STOP_CAPTURE = 2501,
        ///<summary> Start video streaming |Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Start video streaming")]
        VIDEO_START_STREAMING = 2502,
        ///<summary> Stop the given video stream |Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Stop the given video stream")]
        VIDEO_STOP_STREAMING = 2503,
        ///<summary> Request video stream information (VIDEO_STREAM_INFORMATION) |Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Request video stream information (VIDEO_STREAM_INFORMATION)")]
        [Obsolete]
        REQUEST_VIDEO_STREAM_INFORMATION = 2504,
        ///<summary> Request video stream status (VIDEO_STREAM_STATUS) |Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Request video stream status (VIDEO_STREAM_STATUS)")]
        [Obsolete]
        REQUEST_VIDEO_STREAM_STATUS = 2505,
        ///<summary> Request to start streaming logging data over MAVLink (see also LOGGING_DATA message) |Format: 0: ULog| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  </summary>
        [Description("Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)")]
        LOGGING_START = 2510,
        ///<summary> Request to stop streaming log data over MAVLink |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  </summary>
        [Description("Request to stop streaming log data over MAVLink")]
        LOGGING_STOP = 2511,
        ///<summary>  |Landing gear ID (default: 0, -1 for all)| Landing gear position (Down: 0, Up: 1, NaN for no change)| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:NaN)|  </summary>
        [Description("")]
        AIRFRAME_CONFIGURATION = 2520,
        ///<summary> Request to start/stop transmitting over the high latency telemetry |Control transmission over high latency telemetry (0: stop, 1: start)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        [Description("Request to start/stop transmitting over the high latency telemetry")]
        CONTROL_HIGH_LATENCY = 2600,
        ///<summary> Create a panorama at the current position |Viewing angle horizontal of the panorama (+- 0.5 the total angle)| Viewing angle vertical of panorama.| Speed of the horizontal rotation.| Speed of the vertical rotation.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Create a panorama at the current position")]
        PANORAMA_CREATE = 2800,
        ///<summary> Request VTOL transition |The target VTOL state. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Request VTOL transition")]
        DO_VTOL_TRANSITION = 3000,
        ///<summary> Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.          |Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.         ")]
        ARM_AUTHORIZATION_REQUEST = 3001,
        ///<summary> This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes.                    |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes.                   ")]
        SET_GUIDED_SUBMODE_STANDARD = 4000,
        ///<summary> This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.                    |Radius of desired circle in CIRCLE_MODE| User defined| User defined| User defined| Target latitude of center of circle in CIRCLE_MODE| Target longitude of center of circle in CIRCLE_MODE| Reserved (default:0)|  </summary>
        [Description("This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.                   ")]
        SET_GUIDED_SUBMODE_CIRCLE = 4001,
        ///<summary> Fence return point (there can only be one such point in a geofence definition). If rally points are supported they should be used instead. |Reserved| Reserved| Reserved| Reserved| Latitude| Longitude| Altitude|  </summary>
        [Description("Fence return point (there can only be one such point in a geofence definition). If rally points are supported they should be used instead.")]
        FENCE_RETURN_POINT = 5000,
        ///<summary> Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.          |Polygon vertex count| Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group, must be the same for all points in each polygon| Reserved| Reserved| Latitude| Longitude| Reserved|  </summary>
        [Description("Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.         ")]
        FENCE_POLYGON_VERTEX_INCLUSION = 5001,
        ///<summary> Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.          |Polygon vertex count| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  </summary>
        [Description("Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.         ")]
        FENCE_POLYGON_VERTEX_EXCLUSION = 5002,
        ///<summary> Circular fence area. The vehicle must stay inside this area.          |Radius.| Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group| Reserved| Reserved| Latitude| Longitude| Reserved|  </summary>
        [Description("Circular fence area. The vehicle must stay inside this area.         ")]
        FENCE_CIRCLE_INCLUSION = 5003,
        ///<summary> Circular fence area. The vehicle must stay outside this area.          |Radius.| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  </summary>
        [Description("Circular fence area. The vehicle must stay outside this area.         ")]
        FENCE_CIRCLE_EXCLUSION = 5004,
        ///<summary> Rally point. You can have multiple rally points defined.          |Reserved| Reserved| Reserved| Reserved| Latitude| Longitude| Altitude|  </summary>
        [Description("Rally point. You can have multiple rally points defined.         ")]
        RALLY_POINT = 5100,
        ///<summary> Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages. |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  </summary>
        [Description("Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages.")]
        UAVCAN_GET_NODE_INFO = 5200,
        ///<summary> Trigger the start of an ADSB-out IDENT. This should only be used when requested to do so by an Air Traffic Controller in controlled airspace. This starts the IDENT which is then typically held for 18 seconds by the hardware per the Mode A, C, and S transponder spec. |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  </summary>
        [Description("Trigger the start of an ADSB-out IDENT. This should only be used when requested to do so by an Air Traffic Controller in controlled airspace. This starts the IDENT which is then typically held for 18 seconds by the hardware per the Mode A, C, and S transponder spec.")]
        DO_ADSB_OUT_IDENT = 10001,
        ///<summary> Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. |Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.| Desired approach vector in compass heading. A negative value indicates the system can define the approach vector at will.| Desired ground speed at release time. This can be overridden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.| Minimum altitude clearance to the release position. A negative value indicates the system can define the clearance at will.| Latitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled)| Longitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled)| Altitude (MSL)|  </summary>
        [Description("Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity.")]
        PAYLOAD_PREPARE_DEPLOY = 30001,
        ///<summary> Control the payload deployment. |ormal mission. 1: switch to payload deployment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  </summary>
        [Description("Control the payload deployment.")]
        PAYLOAD_CONTROL_DEPLOY = 30002,
        ///<summary> User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  </summary>
        [Description("User defined waypoint item. Ground Station will show the Vehicle as flying through this item.")]
        WAYPOINT_USER_1 = 31000,
        ///<summary> User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  </summary>
        [Description("User defined waypoint item. Ground Station will show the Vehicle as flying through this item.")]
        WAYPOINT_USER_2 = 31001,
        ///<summary> User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  </summary>
        [Description("User defined waypoint item. Ground Station will show the Vehicle as flying through this item.")]
        WAYPOINT_USER_3 = 31002,
        ///<summary> User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  </summary>
        [Description("User defined waypoint item. Ground Station will show the Vehicle as flying through this item.")]
        WAYPOINT_USER_4 = 31003,
        ///<summary> User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  </summary>
        [Description("User defined waypoint item. Ground Station will show the Vehicle as flying through this item.")]
        WAYPOINT_USER_5 = 31004,
        ///<summary> User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  </summary>
        [Description("User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.")]
        SPATIAL_USER_1 = 31005,
        ///<summary> User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  </summary>
        [Description("User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.")]
        SPATIAL_USER_2 = 31006,
        ///<summary> User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  </summary>
        [Description("User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.")]
        SPATIAL_USER_3 = 31007,
        ///<summary> User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  </summary>
        [Description("User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.")]
        SPATIAL_USER_4 = 31008,
        ///<summary> User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  </summary>
        [Description("User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.")]
        SPATIAL_USER_5 = 31009,
        ///<summary> User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  </summary>
        [Description("User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.")]
        USER_1 = 31010,
        ///<summary> User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  </summary>
        [Description("User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.")]
        USER_2 = 31011,
        ///<summary> User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |r defined| User defined| User defined| User defined| User defined| User defined| User defined|  </summary>
        [Description("User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.")]
        USER_3 = 31012,
        ///<summary> User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  </summary>
        [Description("User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.")]
        USER_4 = 31013,
        ///<summary> User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  </summary>
        [Description("User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.")]
        USER_5 = 31014,
        ///<summary> Request forwarding of CAN packets from the given CAN bus to this interface. CAN Frames are sent using CAN_FRAME and CANFD_FRAME messages |Bus number (0 to disable forwarding, 1 for first bus, 2 for 2nd bus, 3 for 3rd bus).| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Request forwarding of CAN packets from the given CAN bus to this interface. CAN Frames are sent using CAN_FRAME and CANFD_FRAME messages")]
        CAN_FORWARD = 32000,
        ///<summary> A system wide power-off event has been initiated. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("A system wide power-off event has been initiated.")]
        POWER_OFF_INITIATED = 42000,
        ///<summary> FLY button has been clicked. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("FLY button has been clicked.")]
        SOLO_BTN_FLY_CLICK = 42001,
        ///<summary> FLY button has been held for 1.5 seconds. |Takeoff altitude.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("FLY button has been held for 1.5 seconds.")]
        SOLO_BTN_FLY_HOLD = 42002,
        ///<summary> PAUSE button has been clicked. |1 if Solo is in a shot mode, 0 otherwise.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("PAUSE button has been clicked.")]
        SOLO_BTN_PAUSE_CLICK = 42003,
        ///<summary> Magnetometer calibration based on fixed position         in earth field given by inclination, declination and intensity. |Magnetic declination.| Magnetic inclination.| Magnetic intensity.| Yaw.| Empty.| Empty.| Empty.|  </summary>
        [Description("Magnetometer calibration based on fixed position         in earth field given by inclination, declination and intensity.")]
        FIXED_MAG_CAL = 42004,
        ///<summary> Magnetometer calibration based on fixed expected field values. |Field strength X.| Field strength Y.| th Z.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Magnetometer calibration based on fixed expected field values.")]
        FIXED_MAG_CAL_FIELD = 42005,
        ///<summary> Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero then use the current vehicle location. |Yaw of vehicle in earth frame.| CompassMask, 0 for all.| Latitude.| Longitude.| Empty.| Empty.| Empty.|  </summary>
        [Description("Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero then use the current vehicle location.")]
        FIXED_MAG_CAL_YAW = 42006,
        ///<summary> Set EKF sensor source set. |Source Set Id.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Set EKF sensor source set.")]
        SET_EKF_SOURCE_SET = 42007,
        ///<summary> Initiate a magnetometer calibration. |Bitmask of magnetometers to calibrate. Use 0 to calibrate all sensors that can be started (sensors may not start if disabled, unhealthy, etc.). The command will NACK if calibration does not start for a sensor explicitly specified by the bitmask.| Automatically retry on failure (0=no retry, 1=retry).| Save without user input (0=require input, 1=autosave).| Delay.| Autoreboot (0=user reboot, 1=autoreboot).| Empty.| Empty.|  </summary>
        [Description("Initiate a magnetometer calibration.")]
        DO_START_MAG_CAL = 42424,
        ///<summary> Accept a magnetometer calibration. |Bitmask of magnetometers that calibration is accepted (0 means all).| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Accept a magnetometer calibration.")]
        DO_ACCEPT_MAG_CAL = 42425,
        ///<summary> Cancel a running magnetometer calibration. |Bitmask of magnetometers to cancel a running calibration (0 means all).| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Cancel a running magnetometer calibration.")]
        DO_CANCEL_MAG_CAL = 42426,
        ///<summary> Command autopilot to get into factory test/diagnostic mode. |0: activate test mode, 1: exit test mode.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Command autopilot to get into factory test/diagnostic mode.")]
        SET_FACTORY_TEST_MODE = 42427,
        ///<summary> Reply with the version banner. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Reply with the version banner.")]
        DO_SEND_BANNER = 42428,
        ///<summary> Used when doing accelerometer calibration. When sent to the GCS tells it what position to put the vehicle in. When sent to the vehicle says what position the vehicle is in. |Position.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Used when doing accelerometer calibration. When sent to the GCS tells it what position to put the vehicle in. When sent to the vehicle says what position the vehicle is in.")]
        ACCELCAL_VEHICLE_POS = 42429,
        ///<summary> Causes the gimbal to reset and boot as if it was just powered on. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Causes the gimbal to reset and boot as if it was just powered on.")]
        GIMBAL_RESET = 42501,
        ///<summary> Reports progress and success or failure of gimbal axis calibration procedure. |Gimbal axis we're reporting calibration progress for.| Current calibration progress for this axis.| Status of the calibration.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Reports progress and success or failure of gimbal axis calibration procedure.")]
        GIMBAL_AXIS_CALIBRATION_STATUS = 42502,
        ///<summary> Starts commutation calibration on the gimbal. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Starts commutation calibration on the gimbal.")]
        GIMBAL_REQUEST_AXIS_CALIBRATION = 42503,
        ///<summary> Erases gimbal application and parameters. |Magic number.| Magic number.| Magic number.| Magic number.| Magic number.| Magic number.| Magic number.|  </summary>
        [Description("Erases gimbal application and parameters.")]
        GIMBAL_FULL_RESET = 42505,
        ///<summary> Command to operate winch. |Winch instance number.| Action to perform.| Length of cable to release (negative to wind).| Release rate (negative to wind).| Empty.| Empty.| Empty.|  </summary>
        [Description("Command to operate winch.")]
        DO_WINCH = 42600,
        ///<summary> Update the bootloader |Empty| Empty| Empty| Empty| Magic number - set to 290876 to actually flash| Empty| Empty|  </summary>
        [Description("Update the bootloader")]
        FLASH_BOOTLOADER = 42650,
        ///<summary> Reset battery capacity for batteries that accumulate consumed battery via integration. |Bitmask of batteries to reset. Least significant bit is for the first battery.| Battery percentage remaining to set.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Reset battery capacity for batteries that accumulate consumed battery via integration.")]
        BATTERY_RESET = 42651,
        ///<summary> Issue a trap signal to the autopilot process, presumably to enter the debugger. |Magic number - set to 32451 to actually trap.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  </summary>
        [Description("Issue a trap signal to the autopilot process, presumably to enter the debugger.")]
        DEBUG_TRAP = 42700,
        ///<summary> Control onboard scripting. |Scripting command to execute| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  </summary>
        [Description("Control onboard scripting.")]
        SCRIPTING = 42701,
        ///<summary> Scripting command as NAV command with wait for completion. |integer command number (0 to 255)| timeout for operation in seconds. Zero means no timeout (0 to 255)| argument1.| argument2.| Empty| Empty| Empty|  </summary>
        [Description("Scripting command as NAV command with wait for completion.")]
        SCRIPT_TIME = 42702,
        ///<summary> Maintain an attitude for a specified time. |Time to maintain specified attitude and climb rate| Roll angle in degrees (positive is lean right, negative is lean left)| Pitch angle in degrees (positive is lean back, negative is lean forward)| Yaw angle| Climb rate| Empty| Empty|  </summary>
        [Description("Maintain an attitude for a specified time.")]
        ATTITUDE_TIME = 42703,
        ///<summary> Change flight speed at a given rate. This slews the vehicle at a controllable rate between it's previous speed and the new one. (affects GUIDED only. Outside GUIDED, aircraft ignores these commands. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.) |Airspeed or groundspeed.| Target Speed| Acceleration rate, 0 to take effect instantly| Empty| Empty| Empty| Empty|  </summary>
        [Description("Change flight speed at a given rate. This slews the vehicle at a controllable rate between it's previous speed and the new one. (affects GUIDED only. Outside GUIDED, aircraft ignores these commands. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.)")]
        GUIDED_CHANGE_SPEED = 43000,
        ///<summary> Change target altitude at a given rate. This slews the vehicle at a controllable rate between it's previous altitude and the new one. (affects GUIDED only. Outside GUIDED, aircraft ignores these commands. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.) |Empty| Empty| Rate of change, toward new altitude. 0 for maximum rate change. Positive numbers only, as negative numbers will not converge on the new target alt.| Empty| Empty| Empty| Target Altitude|  </summary>
        [Description("Change target altitude at a given rate. This slews the vehicle at a controllable rate between it's previous altitude and the new one. (affects GUIDED only. Outside GUIDED, aircraft ignores these commands. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.)")]
        GUIDED_CHANGE_ALTITUDE = 43001,
        ///<summary> Change to target heading at a given rate, overriding previous heading/s. This slews the vehicle at a controllable rate between it's previous heading and the new one. (affects GUIDED only. Exiting GUIDED returns aircraft to normal behaviour defined elsewhere. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.) |course-over-ground or raw vehicle heading.| Target heading.| Maximum centripetal accelearation, ie rate of change,  toward new heading.| Empty| Empty| Empty| Empty|  </summary>
        [Description("Change to target heading at a given rate, overriding previous heading/s. This slews the vehicle at a controllable rate between it's previous heading and the new one. (affects GUIDED only. Exiting GUIDED returns aircraft to normal behaviour defined elsewhere. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.)")]
        GUIDED_CHANGE_HEADING = 43002,

    };

    public enum MAV_MOUNT_MODE : byte
    {
        ///<summary> Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization | </summary>
        [Description("Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization")]
        RETRACT = 0,
        ///<summary> Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory. | </summary>
        [Description("Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.")]
        NEUTRAL = 1,
        ///<summary> Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization | </summary>
        [Description("Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization")]
        MAVLINK_TARGETING = 2,
        ///<summary> Load neutral position and start RC Roll,Pitch,Yaw control with stabilization | </summary>
        [Description("Load neutral position and start RC Roll,Pitch,Yaw control with stabilization")]
        RC_TARGETING = 3,
        ///<summary> Load neutral position and start to point to Lat,Lon,Alt | </summary>
        [Description("Load neutral position and start to point to Lat,Lon,Alt")]
        GPS_POINT = 4,
        ///<summary> Gimbal tracks system with specified system ID | </summary>
        [Description("Gimbal tracks system with specified system ID")]
        SYSID_TARGET = 5,
        ///<summary> Gimbal tracks home location | </summary>
        [Description("Gimbal tracks home location")]
        HOME_LOCATION = 6,

    };

    ///<summary> Enumeration of sensor orientation, according to its rotations </summary>
    public enum MAV_SENSOR_ORIENTATION : byte
    {
        ///<summary> Roll: 0, Pitch: 0, Yaw: 0 | </summary>
        [Description("Roll: 0, Pitch: 0, Yaw: 0")]
        MAV_SENSOR_ROTATION_NONE = 0,
        ///<summary> Roll: 0, Pitch: 0, Yaw: 45 | </summary>
        [Description("Roll: 0, Pitch: 0, Yaw: 45")]
        MAV_SENSOR_ROTATION_YAW_45 = 1,
        ///<summary> Roll: 0, Pitch: 0, Yaw: 90 | </summary>
        [Description("Roll: 0, Pitch: 0, Yaw: 90")]
        MAV_SENSOR_ROTATION_YAW_90 = 2,
        ///<summary> Roll: 0, Pitch: 0, Yaw: 135 | </summary>
        [Description("Roll: 0, Pitch: 0, Yaw: 135")]
        MAV_SENSOR_ROTATION_YAW_135 = 3,
        ///<summary> Roll: 0, Pitch: 0, Yaw: 180 | </summary>
        [Description("Roll: 0, Pitch: 0, Yaw: 180")]
        MAV_SENSOR_ROTATION_YAW_180 = 4,
        ///<summary> Roll: 0, Pitch: 0, Yaw: 225 | </summary>
        [Description("Roll: 0, Pitch: 0, Yaw: 225")]
        MAV_SENSOR_ROTATION_YAW_225 = 5,
        ///<summary> Roll: 0, Pitch: 0, Yaw: 270 | </summary>
        [Description("Roll: 0, Pitch: 0, Yaw: 270")]
        MAV_SENSOR_ROTATION_YAW_270 = 6,
        ///<summary> Roll: 0, Pitch: 0, Yaw: 315 | </summary>
        [Description("Roll: 0, Pitch: 0, Yaw: 315")]
        MAV_SENSOR_ROTATION_YAW_315 = 7,
        ///<summary> Roll: 180, Pitch: 0, Yaw: 0 | </summary>
        [Description("Roll: 180, Pitch: 0, Yaw: 0")]
        MAV_SENSOR_ROTATION_ROLL_180 = 8,
        ///<summary> Roll: 180, Pitch: 0, Yaw: 45 | </summary>
        [Description("Roll: 180, Pitch: 0, Yaw: 45")]
        MAV_SENSOR_ROTATION_ROLL_180_YAW_45 = 9,
        ///<summary> Roll: 180, Pitch: 0, Yaw: 90 | </summary>
        [Description("Roll: 180, Pitch: 0, Yaw: 90")]
        MAV_SENSOR_ROTATION_ROLL_180_YAW_90 = 10,
        ///<summary> Roll: 180, Pitch: 0, Yaw: 135 | </summary>
        [Description("Roll: 180, Pitch: 0, Yaw: 135")]
        MAV_SENSOR_ROTATION_ROLL_180_YAW_135 = 11,
        ///<summary> Roll: 0, Pitch: 180, Yaw: 0 | </summary>
        [Description("Roll: 0, Pitch: 180, Yaw: 0")]
        MAV_SENSOR_ROTATION_PITCH_180 = 12,
        ///<summary> Roll: 180, Pitch: 0, Yaw: 225 | </summary>
        [Description("Roll: 180, Pitch: 0, Yaw: 225")]
        MAV_SENSOR_ROTATION_ROLL_180_YAW_225 = 13,
        ///<summary> Roll: 180, Pitch: 0, Yaw: 270 | </summary>
        [Description("Roll: 180, Pitch: 0, Yaw: 270")]
        MAV_SENSOR_ROTATION_ROLL_180_YAW_270 = 14,
        ///<summary> Roll: 180, Pitch: 0, Yaw: 315 | </summary>
        [Description("Roll: 180, Pitch: 0, Yaw: 315")]
        MAV_SENSOR_ROTATION_ROLL_180_YAW_315 = 15,
        ///<summary> Roll: 90, Pitch: 0, Yaw: 0 | </summary>
        [Description("Roll: 90, Pitch: 0, Yaw: 0")]
        MAV_SENSOR_ROTATION_ROLL_90 = 16,
        ///<summary> Roll: 90, Pitch: 0, Yaw: 45 | </summary>
        [Description("Roll: 90, Pitch: 0, Yaw: 45")]
        MAV_SENSOR_ROTATION_ROLL_90_YAW_45 = 17,
        ///<summary> Roll: 90, Pitch: 0, Yaw: 90 | </summary>
        [Description("Roll: 90, Pitch: 0, Yaw: 90")]
        MAV_SENSOR_ROTATION_ROLL_90_YAW_90 = 18,
        ///<summary> Roll: 90, Pitch: 0, Yaw: 135 | </summary>
        [Description("Roll: 90, Pitch: 0, Yaw: 135")]
        MAV_SENSOR_ROTATION_ROLL_90_YAW_135 = 19,
        ///<summary> Roll: 270, Pitch: 0, Yaw: 0 | </summary>
        [Description("Roll: 270, Pitch: 0, Yaw: 0")]
        MAV_SENSOR_ROTATION_ROLL_270 = 20,
        ///<summary> Roll: 270, Pitch: 0, Yaw: 45 | </summary>
        [Description("Roll: 270, Pitch: 0, Yaw: 45")]
        MAV_SENSOR_ROTATION_ROLL_270_YAW_45 = 21,
        ///<summary> Roll: 270, Pitch: 0, Yaw: 90 | </summary>
        [Description("Roll: 270, Pitch: 0, Yaw: 90")]
        MAV_SENSOR_ROTATION_ROLL_270_YAW_90 = 22,
        ///<summary> Roll: 270, Pitch: 0, Yaw: 135 | </summary>
        [Description("Roll: 270, Pitch: 0, Yaw: 135")]
        MAV_SENSOR_ROTATION_ROLL_270_YAW_135 = 23,
        ///<summary> Roll: 0, Pitch: 90, Yaw: 0 | </summary>
        [Description("Roll: 0, Pitch: 90, Yaw: 0")]
        MAV_SENSOR_ROTATION_PITCH_90 = 24,
        ///<summary> Roll: 0, Pitch: 270, Yaw: 0 | </summary>
        [Description("Roll: 0, Pitch: 270, Yaw: 0")]
        MAV_SENSOR_ROTATION_PITCH_270 = 25,
        ///<summary> Roll: 0, Pitch: 180, Yaw: 90 | </summary>
        [Description("Roll: 0, Pitch: 180, Yaw: 90")]
        MAV_SENSOR_ROTATION_PITCH_180_YAW_90 = 26,
        ///<summary> Roll: 0, Pitch: 180, Yaw: 270 | </summary>
        [Description("Roll: 0, Pitch: 180, Yaw: 270")]
        MAV_SENSOR_ROTATION_PITCH_180_YAW_270 = 27,
        ///<summary> Roll: 90, Pitch: 90, Yaw: 0 | </summary>
        [Description("Roll: 90, Pitch: 90, Yaw: 0")]
        MAV_SENSOR_ROTATION_ROLL_90_PITCH_90 = 28,
        ///<summary> Roll: 180, Pitch: 90, Yaw: 0 | </summary>
        [Description("Roll: 180, Pitch: 90, Yaw: 0")]
        MAV_SENSOR_ROTATION_ROLL_180_PITCH_90 = 29,
        ///<summary> Roll: 270, Pitch: 90, Yaw: 0 | </summary>
        [Description("Roll: 270, Pitch: 90, Yaw: 0")]
        MAV_SENSOR_ROTATION_ROLL_270_PITCH_90 = 30,
        ///<summary> Roll: 90, Pitch: 180, Yaw: 0 | </summary>
        [Description("Roll: 90, Pitch: 180, Yaw: 0")]
        MAV_SENSOR_ROTATION_ROLL_90_PITCH_180 = 31,
        ///<summary> Roll: 270, Pitch: 180, Yaw: 0 | </summary>
        [Description("Roll: 270, Pitch: 180, Yaw: 0")]
        MAV_SENSOR_ROTATION_ROLL_270_PITCH_180 = 32,
        ///<summary> Roll: 90, Pitch: 270, Yaw: 0 | </summary>
        [Description("Roll: 90, Pitch: 270, Yaw: 0")]
        MAV_SENSOR_ROTATION_ROLL_90_PITCH_270 = 33,
        ///<summary> Roll: 180, Pitch: 270, Yaw: 0 | </summary>
        [Description("Roll: 180, Pitch: 270, Yaw: 0")]
        MAV_SENSOR_ROTATION_ROLL_180_PITCH_270 = 34,
        ///<summary> Roll: 270, Pitch: 270, Yaw: 0 | </summary>
        [Description("Roll: 270, Pitch: 270, Yaw: 0")]
        MAV_SENSOR_ROTATION_ROLL_270_PITCH_270 = 35,
        ///<summary> Roll: 90, Pitch: 180, Yaw: 90 | </summary>
        [Description("Roll: 90, Pitch: 180, Yaw: 90")]
        MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 = 36,
        ///<summary> Roll: 90, Pitch: 0, Yaw: 270 | </summary>
        [Description("Roll: 90, Pitch: 0, Yaw: 270")]
        MAV_SENSOR_ROTATION_ROLL_90_YAW_270 = 37,
        ///<summary> Roll: 90, Pitch: 68, Yaw: 293 | </summary>
        [Description("Roll: 90, Pitch: 68, Yaw: 293")]
        MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293 = 38,
        ///<summary> Pitch: 315 | </summary>
        [Description("Pitch: 315")]
        MAV_SENSOR_ROTATION_PITCH_315 = 39,
        ///<summary> Roll: 90, Pitch: 315 | </summary>
        [Description("Roll: 90, Pitch: 315")]
        MAV_SENSOR_ROTATION_ROLL_90_PITCH_315 = 40,
        ///<summary> Custom orientation | </summary>
        [Description("Custom orientation")]
        MAV_SENSOR_ROTATION_CUSTOM = 100,

    };


    /// extensions_start 0
    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 31)]
    ///<summary> The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows whether the system is currently active or not and if an emergency occurred. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occurred it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout. </summary>
    public struct mavlink_sys_status_t
    {
        public mavlink_sys_status_t(/*MAV_SYS_STATUS_SENSOR*/uint onboard_control_sensors_present,/*MAV_SYS_STATUS_SENSOR*/uint onboard_control_sensors_enabled,/*MAV_SYS_STATUS_SENSOR*/uint onboard_control_sensors_health, ushort load, ushort voltage_battery, short current_battery, ushort drop_rate_comm, ushort errors_comm, ushort errors_count1, ushort errors_count2, ushort errors_count3, ushort errors_count4, sbyte battery_remaining)
        {
            this.onboard_control_sensors_present = onboard_control_sensors_present;
            this.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
            this.onboard_control_sensors_health = onboard_control_sensors_health;
            this.load = load;
            this.voltage_battery = voltage_battery;
            this.current_battery = current_battery;
            this.drop_rate_comm = drop_rate_comm;
            this.errors_comm = errors_comm;
            this.errors_count1 = errors_count1;
            this.errors_count2 = errors_count2;
            this.errors_count3 = errors_count3;
            this.errors_count4 = errors_count4;
            this.battery_remaining = battery_remaining;

        }

        /// <summary>Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. MAV_SYS_STATUS_SENSOR  bitmask</summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.")]
        //[FieldOffset(0)]
        public  /*MAV_SYS_STATUS_SENSOR*/uint onboard_control_sensors_present;

        /// <summary>Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. MAV_SYS_STATUS_SENSOR  bitmask</summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.")]
        //[FieldOffset(4)]
        public  /*MAV_SYS_STATUS_SENSOR*/uint onboard_control_sensors_enabled;

        /// <summary>Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy. MAV_SYS_STATUS_SENSOR  bitmask</summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy.")]
        //[FieldOffset(8)]
        public  /*MAV_SYS_STATUS_SENSOR*/uint onboard_control_sensors_health;

        /// <summary>Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000  [d%] </summary>
        [LibDrone.Frames.Text.Units("[d%]")]
        [Description("Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000")]
        //[FieldOffset(12)]
        public ushort load;

        /// <summary>Battery voltage, UINT16_MAX: Voltage not sent by autopilot  [mV] </summary>
        [LibDrone.Frames.Text.Units("[mV]")]
        [Description("Battery voltage, UINT16_MAX: Voltage not sent by autopilot")]
        //[FieldOffset(14)]
        public ushort voltage_battery;

        /// <summary>Battery current, -1: Current not sent by autopilot  [cA] </summary>
        [LibDrone.Frames.Text.Units("[cA]")]
        [Description("Battery current, -1: Current not sent by autopilot")]
        //[FieldOffset(16)]
        public short current_battery;

        /// <summary>Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)  [c%] </summary>
        [LibDrone.Frames.Text.Units("[c%]")]
        [Description("Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)")]
        //[FieldOffset(18)]
        public ushort drop_rate_comm;

        /// <summary>Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)   </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)")]
        //[FieldOffset(20)]
        public ushort errors_comm;

        /// <summary>Autopilot-specific errors   </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Autopilot-specific errors")]
        //[FieldOffset(22)]
        public ushort errors_count1;

        /// <summary>Autopilot-specific errors   </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Autopilot-specific errors")]
        //[FieldOffset(24)]
        public ushort errors_count2;

        /// <summary>Autopilot-specific errors   </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Autopilot-specific errors")]
        //[FieldOffset(26)]
        public ushort errors_count3;

        /// <summary>Autopilot-specific errors   </summary>
        [LibDrone.Frames.Text.Units("")]
        [Description("Autopilot-specific errors")]
        //[FieldOffset(28)]
        public ushort errors_count4;

        /// <summary>Battery energy remaining, -1: Battery remaining energy not sent by autopilot  [%] </summary>
        [LibDrone.Frames.Text.Units("[%]")]
        [Description("Battery energy remaining, -1: Battery remaining energy not sent by autopilot")]
        //[FieldOffset(30)]
        public sbyte battery_remaining;
    };

    public const string MAVLINK_BUILD_DATE = "Tue Jun 07 2022";
    public const string MAVLINK_WIRE_PROTOCOL_VERSION = "2.0";
    public const int MAVLINK_MAX_PAYLOAD_LEN = 255;

    public const byte MAVLINK_CORE_HEADER_LEN = 9;///< Length of core header (of the comm. layer)
    public const byte MAVLINK_CORE_HEADER_MAVLINK1_LEN = 5;///< Length of MAVLink1 core header (of the comm. layer)
    public const byte MAVLINK_NUM_HEADER_BYTES = (MAVLINK_CORE_HEADER_LEN + 1);///< Length of all header bytes, including core and stx
    public const byte MAVLINK_NUM_CHECKSUM_BYTES = 2;
    public const byte MAVLINK_NUM_NON_PAYLOAD_BYTES = (MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES);

    public const int MAVLINK_MAX_PACKET_LEN = (MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_SIGNATURE_BLOCK_LEN);///< Maximum packet length
    public const byte MAVLINK_SIGNATURE_BLOCK_LEN = 13;

    public const int MAVLINK_LITTLE_ENDIAN = 1;
    public const int MAVLINK_BIG_ENDIAN = 0;

    public const byte MAVLINK_STX = 253;

    public const byte MAVLINK_STX_MAVLINK1 = 0xFE;

    public const byte MAVLINK_ENDIAN = MAVLINK_LITTLE_ENDIAN;

    public const bool MAVLINK_ALIGNED_FIELDS = (1 == 1);

    public const byte MAVLINK_CRC_EXTRA = 1;

    public const byte MAVLINK_COMMAND_24BIT = 1;

    public const bool MAVLINK_NEED_BYTE_SWAP = (MAVLINK_ENDIAN == MAVLINK_LITTLE_ENDIAN);


    //Second Variable set

    public const byte MAVLINK_VERSION = 2;

    public const byte MAVLINK_IFLAG_SIGNED = 0x01;
    public const byte MAVLINK_IFLAG_MASK = 0x01;



    public static message_info[] MAVLINK_MESSAGE_INFOS = new message_info[] {
            //Connection 6
            new message_info(1, "SYS_STATUS", 124, 31, 31, typeof( mavlink_sys_status_t )),
            new message_info(2, "SYSTEM_TIME", 137, 12, 12, typeof( mavlink_system_time_t )),
            new message_info(4, "PING", 237, 14, 14, typeof( mavlink_ping_t )),
            new message_info(5, "CHANGE_OPERATOR_CONTROL", 217, 28, 28, typeof( mavlink_change_operator_control_t )),
            new message_info(6, "CHANGE_OPERATOR_CONTROL_ACK", 104, 3, 3, typeof( mavlink_change_operator_control_ack_t )),
            new message_info(7, "AUTH_KEY", 119, 32, 32, typeof( mavlink_auth_key_t )),

            //Trocar modo 5
            new message_info(11, "SET_MODE", 89, 6, 6, typeof( mavlink_set_mode_t )),
            new message_info(20, "PARAM_REQUEST_READ", 214, 20, 20, typeof( mavlink_param_request_read_t )),
            new message_info(21, "PARAM_REQUEST_LIST", 159, 2, 2, typeof( mavlink_param_request_list_t )),
            new message_info(22, "PARAM_VALUE", 220, 25, 25, typeof( mavlink_param_value_t )),
            new message_info(23, "PARAM_SET", 168, 23, 23, typeof( mavlink_param_set_t )),

            //Altitude 4
            new message_info(30, "ATTITUDE", 39, 28, 28, typeof( mavlink_attitude_t )),
            new message_info(31, "ATTITUDE_QUATERNION", 246, 32, 48, typeof( mavlink_attitude_quaternion_t )),
            new message_info(82, "SET_ATTITUDE_TARGET", 49, 39, 39, typeof( mavlink_set_attitude_target_t )),
            new message_info(83, "ATTITUDE_TARGET", 22, 37, 37, typeof(mavlink_attitude_target_t)),

            //Voo 4
            new message_info(84, "SET_POSITION_TARGET_LOCAL_NED", 143, 53, 53, typeof( mavlink_set_position_target_local_ned_t )),
            new message_info(85, "POSITION_TARGET_LOCAL_NED", 140, 51, 51, typeof( mavlink_position_target_local_ned_t )),
            new message_info(86, "SET_POSITION_TARGET_GLOBAL_INT", 5, 53, 53, typeof( mavlink_set_position_target_global_int_t )),
            new message_info(87, "POSITION_TARGET_GLOBAL_INT", 150, 51, 51, typeof( mavlink_position_target_global_int_t )),

            //Land 1
            new message_info(149, "LANDING_TARGET", 200, 30, 60, typeof( mavlink_landing_target_t )),

            //HeartBeat
            new message_info(0, "HEARTBEAT", 50, 9, 9, typeof(mavlink_heartbeat_t))
        };
    public struct message_info
    {
        public uint msgid { get; internal set; }
        public string name { get; internal set; }
        public byte crc { get; internal set; }
        public uint minlength { get; internal set; }
        public uint length { get; internal set; }
        public Type type { get; internal set; }

        public message_info(uint msgid, string name, byte crc, uint minlength, uint length, Type type)
        {
            this.msgid = msgid;
            this.name = name;
            this.crc = crc;
            this.minlength = minlength;
            this.length = length;
            this.type = type;
        }

        public override string ToString()
        {
            return string.Format("{0} - {1}", name, msgid);
        }


    }
}