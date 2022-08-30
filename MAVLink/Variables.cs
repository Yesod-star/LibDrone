//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using System.Threading.Tasks;
//using System.Runtime.InteropServices;
//using static LibDrone.Frames.Text;



///// extensions_start 13
//[StructLayout(LayoutKind.Sequential, Pack = 1, Size = 47)]
/////<summary> Camera Capture Feedback. </summary>
/////
//public struct mavlink_camera_feedback_t {
//    public mavlink_camera_feedback_t(ulong time_usec, int lat, int lng, float alt_msl, float alt_rel, float roll, float pitch, float yaw, float foc_len, ushort img_idx, byte target_system, byte cam_idx,/*CAMERA_FEEDBACK_FLAGS*/byte flags, ushort completed_captures) {
//        this.time_usec = time_usec;
//        this.lat = lat;
//        this.lng = lng;
//        this.alt_msl = alt_msl;
//        this.alt_rel = alt_rel;
//        this.roll = roll;
//        this.pitch = pitch;
//        this.yaw = yaw;
//        this.foc_len = foc_len;
//        this.img_idx = img_idx;
//        this.target_system = target_system;
//        this.cam_idx = cam_idx;
//        this.flags = flags;
//        this.completed_captures = completed_captures;

//    }

//    /// <summary>Image timestamp (since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB).  [us] </summary>
//    [Units("[us]")]
//    [Description("Image timestamp (since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB).")]
//    //[FieldOffset(0)]
//    public ulong time_usec;

//    /// <summary>Latitude.  [degE7] </summary>
//    [Units("[degE7]")]
//    [Description("Latitude.")]
//    //[FieldOffset(8)]
//    public int lat;

//    /// <summary>Longitude.  [degE7] </summary>
//    [Units("[degE7]")]
//    [Description("Longitude.")]
//    //[FieldOffset(12)]
//    public int lng;

//    /// <summary>Altitude (MSL).  [m] </summary>
//    [Units("[m]")]
//    [Description("Altitude (MSL).")]
//    //[FieldOffset(16)]
//    public float alt_msl;

//    /// <summary>Altitude (Relative to HOME location).  [m] </summary>
//    [Units("[m]")]
//    [Description("Altitude (Relative to HOME location).")]
//    //[FieldOffset(20)]
//    public float alt_rel;

//    /// <summary>Camera Roll angle (earth frame, +-180).  [deg] </summary>
//    [Units("[deg]")]
//    [Description("Camera Roll angle (earth frame, +-180).")]
//    //[FieldOffset(24)]
//    public float roll;

//    /// <summary>Camera Pitch angle (earth frame, +-180).  [deg] </summary>
//    [Units("[deg]")]
//    [Description("Camera Pitch angle (earth frame, +-180).")]
//    //[FieldOffset(28)]
//    public float pitch;

//    /// <summary>Camera Yaw (earth frame, 0-360, true).  [deg] </summary>
//    [Units("[deg]")]
//    [Description("Camera Yaw (earth frame, 0-360, true).")]
//    //[FieldOffset(32)]
//    public float yaw;

//    /// <summary>Focal Length.  [mm] </summary>
//    [Units("[mm]")]
//    [Description("Focal Length.")]
//    //[FieldOffset(36)]
//    public float foc_len;

//    /// <summary>Image index.   </summary>
//    [Units("")]
//    [Description("Image index.")]
//    //[FieldOffset(40)]
//    public ushort img_idx;

//    /// <summary>System ID.   </summary>
//    [Units("")]
//    [Description("System ID.")]
//    //[FieldOffset(42)]
//    public byte target_system;

//    /// <summary>Camera ID.   </summary>
//    [Units("")]
//    [Description("Camera ID.")]
//    //[FieldOffset(43)]
//    public byte cam_idx;

//    /// <summary>Feedback flags. CAMERA_FEEDBACK_FLAGS  </summary>
//    [Units("")]
//    [Description("Feedback flags.")]
//    //[FieldOffset(44)]
//    public  /*CAMERA_FEEDBACK_FLAGS*/byte flags;

//    /// <summary>Completed image captures.   </summary>
//    [Units("")]
//    [Description("Completed image captures.")]
//    //[FieldOffset(45)]
//    public ushort completed_captures;
//};
//public partial class MAVLink {

//    ///<summary> Result of mission operation (in a MISSION_ACK message). </summary>
//    public enum MAV_MISSION_RESULT : byte
//    {
//        ///<summary> mission accepted OK | </summary>
//        [Description("mission accepted OK")]
//        MAV_MISSION_ACCEPTED = 0,
//        ///<summary> Generic error / not accepting mission commands at all right now. | </summary>
//        [Description("Generic error / not accepting mission commands at all right now.")]
//        MAV_MISSION_ERROR = 1,
//        ///<summary> Coordinate frame is not supported. | </summary>
//        [Description("Coordinate frame is not supported.")]
//        MAV_MISSION_UNSUPPORTED_FRAME = 2,
//        ///<summary> Command is not supported. | </summary>
//        [Description("Command is not supported.")]
//        MAV_MISSION_UNSUPPORTED = 3,
//        ///<summary> Mission items exceed storage space. | </summary>
//        [Description("Mission items exceed storage space.")]
//        MAV_MISSION_NO_SPACE = 4,
//        ///<summary> One of the parameters has an invalid value. | </summary>
//        [Description("One of the parameters has an invalid value.")]
//        MAV_MISSION_INVALID = 5,
//        ///<summary> param1 has an invalid value. | </summary>
//        [Description("param1 has an invalid value.")]
//        MAV_MISSION_INVALID_PARAM1 = 6,
//        ///<summary> param2 has an invalid value. | </summary>
//        [Description("param2 has an invalid value.")]
//        MAV_MISSION_INVALID_PARAM2 = 7,
//        ///<summary> param3 has an invalid value. | </summary>
//        [Description("param3 has an invalid value.")]
//        MAV_MISSION_INVALID_PARAM3 = 8,
//        ///<summary> param4 has an invalid value. | </summary>
//        [Description("param4 has an invalid value.")]
//        MAV_MISSION_INVALID_PARAM4 = 9,
//        ///<summary> x / param5 has an invalid value. | </summary>
//        [Description("x / param5 has an invalid value.")]
//        MAV_MISSION_INVALID_PARAM5_X = 10,
//        ///<summary> y / param6 has an invalid value. | </summary>
//        [Description("y / param6 has an invalid value.")]
//        MAV_MISSION_INVALID_PARAM6_Y = 11,
//        ///<summary> z / param7 has an invalid value. | </summary>
//        [Description("z / param7 has an invalid value.")]
//        MAV_MISSION_INVALID_PARAM7 = 12,
//        ///<summary> Mission item received out of sequence | </summary>
//        [Description("Mission item received out of sequence")]
//        MAV_MISSION_INVALID_SEQUENCE = 13,
//        ///<summary> Not accepting any mission commands from this communication partner. | </summary>
//        [Description("Not accepting any mission commands from this communication partner.")]
//        MAV_MISSION_DENIED = 14,
//        ///<summary> Current mission operation cancelled (e.g. mission upload, mission download). | </summary>
//        [Description("Current mission operation cancelled (e.g. mission upload, mission download).")]
//        MAV_MISSION_OPERATION_CANCELLED = 15,

//    };

//    /// extensions_start 3
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 4)]
//    ///<summary> Acknowledgment message during waypoint handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero). </summary>
//    public struct mavlink_mission_ack_t
//    {
//        public mavlink_mission_ack_t(byte target_system, byte target_component,/*MAV_MISSION_RESULT*/byte type,/*MAV_MISSION_TYPE*/byte mission_type)
//        {
//            this.target_system = target_system;
//            this.target_component = target_component;
//            this.type = type;
//            this.mission_type = mission_type;

//        }

//        /// <summary>System ID   </summary>
//        [Units("")]
//        [Description("System ID")]
//        //[FieldOffset(0)]
//        public byte target_system;

//        /// <summary>Component ID   </summary>
//        [Units("")]
//        [Description("Component ID")]
//        //[FieldOffset(1)]
//        public byte target_component;

//        /// <summary>Mission result. MAV_MISSION_RESULT  </summary>
//        [Units("")]
//        [Description("Mission result.")]
//        //[FieldOffset(2)]
//        public  /*MAV_MISSION_RESULT*/byte type;

//        /// <summary>Mission type. MAV_MISSION_TYPE  </summary>
//        [Units("")]
//        [Description("Mission type.")]
//        //[FieldOffset(3)]
//        public  /*MAV_MISSION_TYPE*/byte mission_type;
//    };


//    [Obsolete]
//    /// extensions_start 3
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 5)]
//    ///<summary> Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. https://mavlink.io/en/services/mission.html </summary>
//    public struct mavlink_mission_request_t
//    {
//        public mavlink_mission_request_t(ushort seq, byte target_system, byte target_component,/*MAV_MISSION_TYPE*/byte mission_type)
//        {
//            this.seq = seq;
//            this.target_system = target_system;
//            this.target_component = target_component;
//            this.mission_type = mission_type;

//        }

//        /// <summary>Sequence   </summary>
//        [Units("")]
//        [Description("Sequence")]
//        //[FieldOffset(0)]
//        public ushort seq;

//        /// <summary>System ID   </summary>
//        [Units("")]
//        [Description("System ID")]
//        //[FieldOffset(2)]
//        public byte target_system;

//        /// <summary>Component ID   </summary>
//        [Units("")]
//        [Description("Component ID")]
//        //[FieldOffset(3)]
//        public byte target_component;

//        /// <summary>Mission type. MAV_MISSION_TYPE  </summary>
//        [Units("")]
//        [Description("Mission type.")]
//        //[FieldOffset(4)]
//        public  /*MAV_MISSION_TYPE*/byte mission_type;
//    };

//    /// extensions_start 0
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 28)]
//    ///<summary> The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right). </summary>
//    public struct mavlink_attitude_t
//    {
//        public mavlink_attitude_t(uint time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
//        {
//            this.time_boot_ms = time_boot_ms;
//            this.roll = roll;
//            this.pitch = pitch;
//            this.yaw = yaw;
//            this.rollspeed = rollspeed;
//            this.pitchspeed = pitchspeed;
//            this.yawspeed = yawspeed;

//        }

//        /// <summary>Timestamp (time since system boot).  [ms] </summary>
//        [Units("[ms]")]
//        [Description("Timestamp (time since system boot).")]
//        //[FieldOffset(0)]
//        public uint time_boot_ms;

//        /// <summary>Roll angle (-pi..+pi)  [rad] </summary>
//        [Units("[rad]")]
//        [Description("Roll angle (-pi..+pi)")]
//        //[FieldOffset(4)]
//        public float roll;

//        /// <summary>Pitch angle (-pi..+pi)  [rad] </summary>
//        [Units("[rad]")]
//        [Description("Pitch angle (-pi..+pi)")]
//        //[FieldOffset(8)]
//        public float pitch;

//        /// <summary>Yaw angle (-pi..+pi)  [rad] </summary>
//        [Units("[rad]")]
//        [Description("Yaw angle (-pi..+pi)")]
//        //[FieldOffset(12)]
//        public float yaw;

//        /// <summary>Roll angular speed  [rad/s] </summary>
//        [Units("[rad/s]")]
//        [Description("Roll angular speed")]
//        //[FieldOffset(16)]
//        public float rollspeed;

//        /// <summary>Pitch angular speed  [rad/s] </summary>
//        [Units("[rad/s]")]
//        [Description("Pitch angular speed")]
//        //[FieldOffset(20)]
//        public float pitchspeed;

//        /// <summary>Yaw angular speed  [rad/s] </summary>
//        [Units("[rad/s]")]
//        [Description("Yaw angular speed")]
//        //[FieldOffset(24)]
//        public float yawspeed;
//    };

//    ///<summary> A data stream is not a fixed set of messages, but rather a      recommendation to the autopilot software. Individual autopilots may or may not obey      the recommended messages. </summary>
//    public enum MAV_DATA_STREAM : int /*default*/
//    {
//        ///<summary> Enable all data streams | </summary>
//        [Description("Enable all data streams")]
//        ALL = 0,
//        ///<summary> Enable IMU_RAW, GPS_RAW, GPS_STATUS packets. | </summary>
//        [Description("Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.")]
//        RAW_SENSORS = 1,
//        ///<summary> Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS | </summary>
//        [Description("Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS")]
//        EXTENDED_STATUS = 2,
//        ///<summary> Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW | </summary>
//        [Description("Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW")]
//        RC_CHANNELS = 3,
//        ///<summary> Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT. | </summary>
//        [Description("Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.")]
//        RAW_CONTROLLER = 4,
//        ///<summary> Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages. | </summary>
//        [Description("Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.")]
//        POSITION = 6,
//        ///<summary> Dependent on the autopilot | </summary>
//        [Description("Dependent on the autopilot")]
//        EXTRA1 = 10,
//        ///<summary> Dependent on the autopilot | </summary>
//        [Description("Dependent on the autopilot")]
//        EXTRA2 = 11,
//        ///<summary> Dependent on the autopilot | </summary>
//        [Description("Dependent on the autopilot")]
//        EXTRA3 = 12,

//    };
//    public enum MAV_COMPONENT : int /*default*/
//{
//        ///<summary> Target id (target_component) used to broadcast messages to all components of the receiving system. Components should attempt to process messages with this component ID and forward to components on any other interfaces. Note: This is not a valid *source* component id for a message. | </summary>
//        [Description("Target id (target_component) used to broadcast messages to all components of the receiving system. Components should attempt to process messages with this component ID and forward to components on any other interfaces. Note: This is not a valid *source* component id for a message.")]
//        MAV_COMP_ID_ALL = 0,
//        ///<summary> System flight controller component ('autopilot'). Only one autopilot is expected in a particular system. | </summary>
//        [Description("System flight controller component ('autopilot'). Only one autopilot is expected in a particular system.")]
//        MAV_COMP_ID_AUTOPILOT1 = 1,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER1 = 25,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER2 = 26,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER3 = 27,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER4 = 28,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER5 = 29,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER6 = 30,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER7 = 31,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER8 = 32,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER9 = 33,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER10 = 34,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER11 = 35,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER12 = 36,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER13 = 37,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER14 = 38,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER15 = 39,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER16 = 40,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER17 = 41,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER18 = 42,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER19 = 43,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER20 = 44,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER21 = 45,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER22 = 46,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER23 = 47,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER24 = 48,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER25 = 49,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER26 = 50,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER27 = 51,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER28 = 52,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER29 = 53,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER30 = 54,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER31 = 55,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER32 = 56,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER33 = 57,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER34 = 58,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER35 = 59,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER36 = 60,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER37 = 61,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER38 = 62,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER39 = 63,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER40 = 64,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER41 = 65,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER42 = 66,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER43 = 67,
//        ///<summary> Telemetry radio (e.g. SiK radio, or other component that emits RADIO_STATUS messages). | </summary>
//        [Description("Telemetry radio (e.g. SiK radio, or other component that emits RADIO_STATUS messages).")]
//        MAV_COMP_ID_TELEMETRY_RADIO = 68,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER45 = 69,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER46 = 70,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER47 = 71,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER48 = 72,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER49 = 73,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER50 = 74,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER51 = 75,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER52 = 76,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER53 = 77,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER54 = 78,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER55 = 79,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER56 = 80,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER57 = 81,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER58 = 82,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER59 = 83,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER60 = 84,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER61 = 85,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER62 = 86,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER63 = 87,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER64 = 88,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER65 = 89,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER66 = 90,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER67 = 91,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER68 = 92,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER69 = 93,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER70 = 94,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER71 = 95,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER72 = 96,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER73 = 97,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER74 = 98,
//        ///<summary> Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. | </summary>
//        [Description("Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.")]
//        MAV_COMP_ID_USER75 = 99,
//        ///<summary> Camera #1. | </summary>
//        [Description("Camera #1.")]
//        MAV_COMP_ID_CAMERA = 100,
//        ///<summary> Camera #2. | </summary>
//        [Description("Camera #2.")]
//        MAV_COMP_ID_CAMERA2 = 101,
//        ///<summary> Camera #3. | </summary>
//        [Description("Camera #3.")]
//        MAV_COMP_ID_CAMERA3 = 102,
//        ///<summary> Camera #4. | </summary>
//        [Description("Camera #4.")]
//        MAV_COMP_ID_CAMERA4 = 103,
//        ///<summary> Camera #5. | </summary>
//        [Description("Camera #5.")]
//        MAV_COMP_ID_CAMERA5 = 104,
//        ///<summary> Camera #6. | </summary>
//        [Description("Camera #6.")]
//        MAV_COMP_ID_CAMERA6 = 105,
//        ///<summary> Servo #1. | </summary>
//        [Description("Servo #1.")]
//        MAV_COMP_ID_SERVO1 = 140,
//        ///<summary> Servo #2. | </summary>
//        [Description("Servo #2.")]
//        MAV_COMP_ID_SERVO2 = 141,
//        ///<summary> Servo #3. | </summary>
//        [Description("Servo #3.")]
//        MAV_COMP_ID_SERVO3 = 142,
//        ///<summary> Servo #4. | </summary>
//        [Description("Servo #4.")]
//        MAV_COMP_ID_SERVO4 = 143,
//        ///<summary> Servo #5. | </summary>
//        [Description("Servo #5.")]
//        MAV_COMP_ID_SERVO5 = 144,
//        ///<summary> Servo #6. | </summary>
//        [Description("Servo #6.")]
//        MAV_COMP_ID_SERVO6 = 145,
//        ///<summary> Servo #7. | </summary>
//        [Description("Servo #7.")]
//        MAV_COMP_ID_SERVO7 = 146,
//        ///<summary> Servo #8. | </summary>
//        [Description("Servo #8.")]
//        MAV_COMP_ID_SERVO8 = 147,
//        ///<summary> Servo #9. | </summary>
//        [Description("Servo #9.")]
//        MAV_COMP_ID_SERVO9 = 148,
//        ///<summary> Servo #10. | </summary>
//        [Description("Servo #10.")]
//        MAV_COMP_ID_SERVO10 = 149,
//        ///<summary> Servo #11. | </summary>
//        [Description("Servo #11.")]
//        MAV_COMP_ID_SERVO11 = 150,
//        ///<summary> Servo #12. | </summary>
//        [Description("Servo #12.")]
//        MAV_COMP_ID_SERVO12 = 151,
//        ///<summary> Servo #13. | </summary>
//        [Description("Servo #13.")]
//        MAV_COMP_ID_SERVO13 = 152,
//        ///<summary> Servo #14. | </summary>
//        [Description("Servo #14.")]
//        MAV_COMP_ID_SERVO14 = 153,
//        ///<summary> Gimbal #1. | </summary>
//        [Description("Gimbal #1.")]
//        MAV_COMP_ID_GIMBAL = 154,
//        ///<summary> Logging component. | </summary>
//        [Description("Logging component.")]
//        MAV_COMP_ID_LOG = 155,
//        ///<summary> Automatic Dependent Surveillance-Broadcast (ADS-B) component. | </summary>
//        [Description("Automatic Dependent Surveillance-Broadcast (ADS-B) component.")]
//        MAV_COMP_ID_ADSB = 156,
//        ///<summary> On Screen Display (OSD) devices for video links. | </summary>
//        [Description("On Screen Display (OSD) devices for video links.")]
//        MAV_COMP_ID_OSD = 157,
//        ///<summary> Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter microservice. | </summary>
//        [Description("Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter microservice.")]
//        MAV_COMP_ID_PERIPHERAL = 158,
//        ///<summary> Gimbal ID for QX1. | </summary>
//        [Description("Gimbal ID for QX1.")]
//        [Obsolete]
//        MAV_COMP_ID_QX1_GIMBAL = 159,
//        ///<summary> FLARM collision alert component. | </summary>
//        [Description("FLARM collision alert component.")]
//        MAV_COMP_ID_FLARM = 160,
//        ///<summary> Parachute component. | </summary>
//        [Description("Parachute component.")]
//        MAV_COMP_ID_PARACHUTE = 161,
//        ///<summary> Gimbal #2. | </summary>
//        [Description("Gimbal #2.")]
//        MAV_COMP_ID_GIMBAL2 = 171,
//        ///<summary> Gimbal #3. | </summary>
//        [Description("Gimbal #3.")]
//        MAV_COMP_ID_GIMBAL3 = 172,
//        ///<summary> Gimbal #4 | </summary>
//        [Description("Gimbal #4")]
//        MAV_COMP_ID_GIMBAL4 = 173,
//        ///<summary> Gimbal #5. | </summary>
//        [Description("Gimbal #5.")]
//        MAV_COMP_ID_GIMBAL5 = 174,
//        ///<summary> Gimbal #6. | </summary>
//        [Description("Gimbal #6.")]
//        MAV_COMP_ID_GIMBAL6 = 175,
//        ///<summary> Battery #1. | </summary>
//        [Description("Battery #1.")]
//        MAV_COMP_ID_BATTERY = 180,
//        ///<summary> Battery #2. | </summary>
//        [Description("Battery #2.")]
//        MAV_COMP_ID_BATTERY2 = 181,
//        ///<summary> CAN over MAVLink client. | </summary>
//        [Description("CAN over MAVLink client.")]
//        MAV_COMP_ID_MAVCAN = 189,
//        ///<summary> Component that can generate/supply a mission flight plan (e.g. GCS or developer API). | </summary>
//        [Description("Component that can generate/supply a mission flight plan (e.g. GCS or developer API).")]
//        MAV_COMP_ID_MISSIONPLANNER = 190,
//        ///<summary> Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on. | </summary>
//        [Description("Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on.")]
//        MAV_COMP_ID_ONBOARD_COMPUTER = 191,
//        ///<summary> Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on. | </summary>
//        [Description("Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on.")]
//        MAV_COMP_ID_ONBOARD_COMPUTER2 = 192,
//        ///<summary> Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on. | </summary>
//        [Description("Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on.")]
//        MAV_COMP_ID_ONBOARD_COMPUTER3 = 193,
//        ///<summary> Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on. | </summary>
//        [Description("Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on.")]
//        MAV_COMP_ID_ONBOARD_COMPUTER4 = 194,
//        ///<summary> Component that finds an optimal path between points based on a certain constraint (e.g. minimum snap, shortest path, cost, etc.). | </summary>
//        [Description("Component that finds an optimal path between points based on a certain constraint (e.g. minimum snap, shortest path, cost, etc.).")]
//        MAV_COMP_ID_PATHPLANNER = 195,
//        ///<summary> Component that plans a collision free path between two points. | </summary>
//        [Description("Component that plans a collision free path between two points.")]
//        MAV_COMP_ID_OBSTACLE_AVOIDANCE = 196,
//        ///<summary> Component that provides position estimates using VIO techniques. | </summary>
//        [Description("Component that provides position estimates using VIO techniques.")]
//        MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY = 197,
//        ///<summary> Component that manages pairing of vehicle and GCS. | </summary>
//        [Description("Component that manages pairing of vehicle and GCS.")]
//        MAV_COMP_ID_PAIRING_MANAGER = 198,
//        ///<summary> Inertial Measurement Unit (IMU) #1. | </summary>
//        [Description("Inertial Measurement Unit (IMU) #1.")]
//        MAV_COMP_ID_IMU = 200,
//        ///<summary> Inertial Measurement Unit (IMU) #2. | </summary>
//        [Description("Inertial Measurement Unit (IMU) #2.")]
//        MAV_COMP_ID_IMU_2 = 201,
//        ///<summary> Inertial Measurement Unit (IMU) #3. | </summary>
//        [Description("Inertial Measurement Unit (IMU) #3.")]
//        MAV_COMP_ID_IMU_3 = 202,
//        ///<summary> GPS #1. | </summary>
//        [Description("GPS #1.")]
//        MAV_COMP_ID_GPS = 220,
//        ///<summary> GPS #2. | </summary>
//        [Description("GPS #2.")]
//        MAV_COMP_ID_GPS2 = 221,
//        ///<summary> Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet). | </summary>
//        [Description("Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).")]
//        MAV_COMP_ID_ODID_TXRX_1 = 236,
//        ///<summary> Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet). | </summary>
//        [Description("Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).")]
//        MAV_COMP_ID_ODID_TXRX_2 = 237,
//        ///<summary> Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet). | </summary>
//        [Description("Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).")]
//        MAV_COMP_ID_ODID_TXRX_3 = 238,
//        ///<summary> Component to bridge MAVLink to UDP (i.e. from a UART). | </summary>
//        [Description("Component to bridge MAVLink to UDP (i.e. from a UART).")]
//        MAV_COMP_ID_UDP_BRIDGE = 240,
//        ///<summary> Component to bridge to UART (i.e. from UDP). | </summary>
//        [Description("Component to bridge to UART (i.e. from UDP).")]
//        MAV_COMP_ID_UART_BRIDGE = 241,
//        ///<summary> Component handling TUNNEL messages (e.g. vendor specific GUI of a component). | </summary>
//        [Description("Component handling TUNNEL messages (e.g. vendor specific GUI of a component).")]
//        MAV_COMP_ID_TUNNEL_NODE = 242,
//        ///<summary> Component for handling system messages (e.g. to ARM, takeoff, etc.). | </summary>
//        [Description("Component for handling system messages (e.g. to ARM, takeoff, etc.).")]
//        [Obsolete]
//        MAV_COMP_ID_SYSTEM_CONTROL = 250,

//    };

//    ///<summary> MAVLINK component type reported in HEARTBEAT message. Flight controllers must report the type of the vehicle on which they are mounted (e.g. MAV_TYPE_OCTOROTOR). All other components must report a value appropriate for their type (e.g. a camera must use MAV_TYPE_CAMERA). </summary>
//    public enum MAV_TYPE : byte {
//            ///<summary> Generic micro air vehicle | </summary>
//            [Description("Generic micro air vehicle")]
//            GENERIC = 0,
//            ///<summary> Fixed wing aircraft. | </summary>
//            [Description("Fixed wing aircraft.")]
//            FIXED_WING = 1,
//            ///<summary> Quadrotor | </summary>
//            [Description("Quadrotor")]
//            QUADROTOR = 2,
//            ///<summary> Coaxial helicopter | </summary>
//            [Description("Coaxial helicopter")]
//            COAXIAL = 3,
//            ///<summary> Normal helicopter with tail rotor. | </summary>
//            [Description("Normal helicopter with tail rotor.")]
//            HELICOPTER = 4,
//            ///<summary> Ground installation | </summary>
//            [Description("Ground installation")]
//            ANTENNA_TRACKER = 5,
//            ///<summary> Operator control unit / ground control station | </summary>
//            [Description("Operator control unit / ground control station")]
//            GCS = 6,
//            ///<summary> Airship, controlled | </summary>
//            [Description("Airship, controlled")]
//            AIRSHIP = 7,
//            ///<summary> Free balloon, uncontrolled | </summary>
//            [Description("Free balloon, uncontrolled")]
//            FREE_BALLOON = 8,
//            ///<summary> Rocket | </summary>
//            [Description("Rocket")]
//            ROCKET = 9,
//            ///<summary> Ground rover | </summary>
//            [Description("Ground rover")]
//            GROUND_ROVER = 10,
//            ///<summary> Surface vessel, boat, ship | </summary>
//            [Description("Surface vessel, boat, ship")]
//            SURFACE_BOAT = 11,
//            ///<summary> Submarine | </summary>
//            [Description("Submarine")]
//            SUBMARINE = 12,
//            ///<summary> Hexarotor | </summary>
//            [Description("Hexarotor")]
//            HEXAROTOR = 13,
//            ///<summary> Octorotor | </summary>
//            [Description("Octorotor")]
//            OCTOROTOR = 14,
//            ///<summary> Tricopter | </summary>
//            [Description("Tricopter")]
//            TRICOPTER = 15,
//            ///<summary> Flapping wing | </summary>
//            [Description("Flapping wing")]
//            FLAPPING_WING = 16,
//            ///<summary> Kite | </summary>
//            [Description("Kite")]
//            KITE = 17,
//            ///<summary> Onboard companion controller | </summary>
//            [Description("Onboard companion controller")]
//            ONBOARD_CONTROLLER = 18,
//            ///<summary> Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter. | </summary>
//            [Description("Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.")]
//            VTOL_DUOROTOR = 19,
//            ///<summary> Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter. | </summary>
//            [Description("Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.")]
//            VTOL_QUADROTOR = 20,
//            ///<summary> Tiltrotor VTOL | </summary>
//            [Description("Tiltrotor VTOL")]
//            VTOL_TILTROTOR = 21,
//            ///<summary> VTOL reserved 2 | </summary>
//            [Description("VTOL reserved 2")]
//            VTOL_RESERVED2 = 22,
//            ///<summary> VTOL reserved 3 | </summary>
//            [Description("VTOL reserved 3")]
//            VTOL_RESERVED3 = 23,
//            ///<summary> VTOL reserved 4 | </summary>
//            [Description("VTOL reserved 4")]
//            VTOL_RESERVED4 = 24,
//            ///<summary> VTOL reserved 5 | </summary>
//            [Description("VTOL reserved 5")]
//            VTOL_RESERVED5 = 25,
//            ///<summary> Gimbal | </summary>
//            [Description("Gimbal")]
//            GIMBAL = 26,
//            ///<summary> ADSB system | </summary>
//            [Description("ADSB system")]
//            ADSB = 27,
//            ///<summary> Steerable, nonrigid airfoil | </summary>
//            [Description("Steerable, nonrigid airfoil")]
//            PARAFOIL = 28,
//            ///<summary> Dodecarotor | </summary>
//            [Description("Dodecarotor")]
//            DODECAROTOR = 29,
//            ///<summary> Camera | </summary>
//            [Description("Camera")]
//            CAMERA = 30,
//            ///<summary> Charging station | </summary>
//            [Description("Charging station")]
//            CHARGING_STATION = 31,
//            ///<summary> FLARM collision avoidance system | </summary>
//            [Description("FLARM collision avoidance system")]
//            FLARM = 32,
//            ///<summary> Servo | </summary>
//            [Description("Servo")]
//            SERVO = 33,
//            ///<summary> Open Drone ID. See https://mavlink.io/en/services/opendroneid.html. | </summary>
//            [Description("Open Drone ID. See https://mavlink.io/en/services/opendroneid.html.")]
//            ODID = 34,
//            ///<summary> Decarotor | </summary>
//            [Description("Decarotor")]
//            DECAROTOR = 35,
//            ///<summary> Battery | </summary>
//            [Description("Battery")]
//            BATTERY = 36,
//            ///<summary> Parachute | </summary>
//            [Description("Parachute")]
//            PARACHUTE = 37,
//            ///<summary> Log | </summary>
//            [Description("Log")]
//            LOG = 38,
//            ///<summary> OSD | </summary>
//            [Description("OSD")]
//            OSD = 39,
//            ///<summary> IMU | </summary>
//            [Description("IMU")]
//            IMU = 40,
//            ///<summary> GPS | </summary>
//            [Description("GPS")]
//            GPS = 41,
//            ///<summary> Winch | </summary>
//            [Description("Winch")]
//            WINCH = 42,

//        };
//    ///<summary> Micro air vehicle / autopilot classes. This identifies the individual model. </summary>
//    public enum MAV_AUTOPILOT : byte {
//        ///<summary> Generic autopilot, full support for everything | </summary>
//        [Description("Generic autopilot, full support for everything")]
//        GENERIC = 0,
//        ///<summary> Reserved for future use. | </summary>
//        [Description("Reserved for future use.")]
//        RESERVED = 1,
//        ///<summary> SLUGS autopilot, http://slugsuav.soe.ucsc.edu | </summary>
//        [Description("SLUGS autopilot, http://slugsuav.soe.ucsc.edu")]
//        SLUGS = 2,
//        ///<summary> ArduPilot - Plane/Copter/Rover/Sub/Tracker, https://ardupilot.org | </summary>
//        [Description("ArduPilot - Plane/Copter/Rover/Sub/Tracker, https://ardupilot.org")]
//        ARDUPILOTMEGA = 3,
//        ///<summary> OpenPilot, http://openpilot.org | </summary>
//        [Description("OpenPilot, http://openpilot.org")]
//        OPENPILOT = 4,
//        ///<summary> Generic autopilot only supporting simple waypoints | </summary>
//        [Description("Generic autopilot only supporting simple waypoints")]
//        GENERIC_WAYPOINTS_ONLY = 5,
//        ///<summary> Generic autopilot supporting waypoints and other simple navigation commands | </summary>
//        [Description("Generic autopilot supporting waypoints and other simple navigation commands")]
//        GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6,
//        ///<summary> Generic autopilot supporting the full mission command set | </summary>
//        [Description("Generic autopilot supporting the full mission command set")]
//        GENERIC_MISSION_FULL = 7,
//        ///<summary> No valid autopilot, e.g. a GCS or other MAVLink component | </summary>
//        [Description("No valid autopilot, e.g. a GCS or other MAVLink component")]
//        INVALID = 8,
//        ///<summary> PPZ UAV - http://nongnu.org/paparazzi | </summary>
//        [Description("PPZ UAV - http://nongnu.org/paparazzi")]
//        PPZ = 9,
//        ///<summary> UAV Dev Board | </summary>
//        [Description("UAV Dev Board")]
//        UDB = 10,
//        ///<summary> FlexiPilot | </summary>
//        [Description("FlexiPilot")]
//        FP = 11,
//        ///<summary> PX4 Autopilot - http://px4.io/ | </summary>
//        [Description("PX4 Autopilot - http://px4.io/")]
//        PX4 = 12,
//        ///<summary> SMACCMPilot - http://smaccmpilot.org | </summary>
//        [Description("SMACCMPilot - http://smaccmpilot.org")]
//        SMACCMPILOT = 13,
//        ///<summary> AutoQuad -- http://autoquad.org | </summary>
//        [Description("AutoQuad -- http://autoquad.org")]
//        AUTOQUAD = 14,
//        ///<summary> Armazila -- http://armazila.com | </summary>
//        [Description("Armazila -- http://armazila.com")]
//        ARMAZILA = 15,
//        ///<summary> Aerob -- http://aerob.ru | </summary>
//        [Description("Aerob -- http://aerob.ru")]
//        AEROB = 16,
//        ///<summary> ASLUAV autopilot -- http://www.asl.ethz.ch | </summary>
//        [Description("ASLUAV autopilot -- http://www.asl.ethz.ch")]
//        ASLUAV = 17,
//        ///<summary> SmartAP Autopilot - http://sky-drones.com | </summary>
//        [Description("SmartAP Autopilot - http://sky-drones.com")]
//        SMARTAP = 18,
//        ///<summary> AirRails - http://uaventure.com | </summary>
//        [Description("AirRails - http://uaventure.com")]
//        AIRRAILS = 19,
//        ///<summary> Fusion Reflex - https://fusion.engineering | </summary>
//        [Description("Fusion Reflex - https://fusion.engineering")]
//        REFLEX = 20,

//    };

//    public enum MAVLINK_MSG_ID {

//        SYS_STATUS = 1,
//        SYSTEM_TIME = 2,
//        PING = 4,
//        CHANGE_OPERATOR_CONTROL = 5,
//        CHANGE_OPERATOR_CONTROL_ACK = 6,
//        AUTH_KEY = 7,
//        SET_MODE = 11,
//        PARAM_REQUEST_READ = 20,
//        PARAM_REQUEST_LIST = 21,
//        PARAM_VALUE = 22,
//        PARAM_SET = 23,
//        GPS_RAW_INT = 24,
//        GPS_STATUS = 25,
//        SCALED_IMU = 26,
//        RAW_IMU = 27,
//        RAW_PRESSURE = 28,
//        SCALED_PRESSURE = 29,
//        ATTITUDE = 30,
//        ATTITUDE_QUATERNION = 31,
//        LOCAL_POSITION_NED = 32,
//        GLOBAL_POSITION_INT = 33,
//        RC_CHANNELS_SCALED = 34,
//        RC_CHANNELS_RAW = 35,
//        SERVO_OUTPUT_RAW = 36,
//        MISSION_REQUEST_PARTIAL_LIST = 37,
//        MISSION_WRITE_PARTIAL_LIST = 38,
//        MISSION_ITEM = 39,
//        MISSION_REQUEST = 40,
//        MISSION_SET_CURRENT = 41,
//        MISSION_CURRENT = 42,
//        MISSION_REQUEST_LIST = 43,
//        MISSION_COUNT = 44,
//        MISSION_CLEAR_ALL = 45,
//        MISSION_ITEM_REACHED = 46,
//        MISSION_ACK = 47,
//        SET_GPS_GLOBAL_ORIGIN = 48,
//        GPS_GLOBAL_ORIGIN = 49,
//        PARAM_MAP_RC = 50,
//        MISSION_REQUEST_INT = 51,
//        SAFETY_SET_ALLOWED_AREA = 54,
//        SAFETY_ALLOWED_AREA = 55,
//        ATTITUDE_QUATERNION_COV = 61,
//        NAV_CONTROLLER_OUTPUT = 62,
//        GLOBAL_POSITION_INT_COV = 63,
//        LOCAL_POSITION_NED_COV = 64,
//        RC_CHANNELS = 65,
//        REQUEST_DATA_STREAM = 66,
//        DATA_STREAM = 67,
//        MANUAL_CONTROL = 69,
//        RC_CHANNELS_OVERRIDE = 70,
//        MISSION_ITEM_INT = 73,
//        VFR_HUD = 74,
//        COMMAND_INT = 75,
//        COMMAND_LONG = 76,
//        COMMAND_ACK = 77,
//        MANUAL_SETPOINT = 81,
//        SET_ATTITUDE_TARGET = 82,
//        ATTITUDE_TARGET = 83,
//        SET_POSITION_TARGET_LOCAL_NED = 84,
//        POSITION_TARGET_LOCAL_NED = 85,
//        SET_POSITION_TARGET_GLOBAL_INT = 86,
//        POSITION_TARGET_GLOBAL_INT = 87,
//        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET = 89,
//        HIL_STATE = 90,
//        HIL_CONTROLS = 91,
//        HIL_RC_INPUTS_RAW = 92,
//        HIL_ACTUATOR_CONTROLS = 93,
//        OPTICAL_FLOW = 100,
//        GLOBAL_VISION_POSITION_ESTIMATE = 101,
//        VISION_POSITION_ESTIMATE = 102,
//        VISION_SPEED_ESTIMATE = 103,
//        VICON_POSITION_ESTIMATE = 104,
//        HIGHRES_IMU = 105,
//        OPTICAL_FLOW_RAD = 106,
//        HIL_SENSOR = 107,
//        SIM_STATE = 108,
//        RADIO_STATUS = 109,
//        FILE_TRANSFER_PROTOCOL = 110,
//        TIMESYNC = 111,
//        CAMERA_TRIGGER = 112,
//        HIL_GPS = 113,
//        HIL_OPTICAL_FLOW = 114,
//        HIL_STATE_QUATERNION = 115,
//        SCALED_IMU2 = 116,
//        LOG_REQUEST_LIST = 117,
//        LOG_ENTRY = 118,
//        LOG_REQUEST_DATA = 119,
//        LOG_DATA = 120,
//        LOG_ERASE = 121,
//        LOG_REQUEST_END = 122,
//        GPS_INJECT_DATA = 123,
//        GPS2_RAW = 124,
//        POWER_STATUS = 125,
//        SERIAL_CONTROL = 126,
//        GPS_RTK = 127,
//        GPS2_RTK = 128,
//        SCALED_IMU3 = 129,
//        DATA_TRANSMISSION_HANDSHAKE = 130,
//        ENCAPSULATED_DATA = 131,
//        DISTANCE_SENSOR = 132,
//        TERRAIN_REQUEST = 133,
//        TERRAIN_DATA = 134,
//        TERRAIN_CHECK = 135,
//        TERRAIN_REPORT = 136,
//        SCALED_PRESSURE2 = 137,
//        ATT_POS_MOCAP = 138,
//        SET_ACTUATOR_CONTROL_TARGET = 139,
//        ACTUATOR_CONTROL_TARGET = 140,
//        ALTITUDE = 141,
//        RESOURCE_REQUEST = 142,
//        SCALED_PRESSURE3 = 143,
//        FOLLOW_TARGET = 144,
//        CONTROL_SYSTEM_STATE = 146,
//        BATTERY_STATUS = 147,
//        AUTOPILOT_VERSION = 148,
//        LANDING_TARGET = 149,
//        SENSOR_OFFSETS = 150,
//        SET_MAG_OFFSETS = 151,
//        MEMINFO = 152,
//        AP_ADC = 153,
//        DIGICAM_CONFIGURE = 154,
//        DIGICAM_CONTROL = 155,
//        MOUNT_CONFIGURE = 156,
//        MOUNT_CONTROL = 157,
//        MOUNT_STATUS = 158,
//        FENCE_POINT = 160,
//        FENCE_FETCH_POINT = 161,
//        FENCE_STATUS = 162,
//        AHRS = 163,
//        SIMSTATE = 164,
//        HWSTATUS = 165,
//        RADIO = 166,
//        LIMITS_STATUS = 167,
//        WIND = 168,
//        DATA16 = 169,
//        DATA32 = 170,
//        DATA64 = 171,
//        DATA96 = 172,
//        RANGEFINDER = 173,
//        AIRSPEED_AUTOCAL = 174,
//        RALLY_POINT = 175,
//        RALLY_FETCH_POINT = 176,
//        COMPASSMOT_STATUS = 177,
//        AHRS2 = 178,
//        CAMERA_STATUS = 179,
//        CAMERA_FEEDBACK = 180,
//        BATTERY2 = 181,
//        AHRS3 = 182,
//        AUTOPILOT_VERSION_REQUEST = 183,
//        REMOTE_LOG_DATA_BLOCK = 184,
//        REMOTE_LOG_BLOCK_STATUS = 185,
//        LED_CONTROL = 186,
//        MAG_CAL_PROGRESS = 191,
//        MAG_CAL_REPORT = 192,
//        EKF_STATUS_REPORT = 193,
//        PID_TUNING = 194,
//        DEEPSTALL = 195,
//        GIMBAL_REPORT = 200,
//        GIMBAL_CONTROL = 201,
//        GIMBAL_TORQUE_CMD_REPORT = 214,
//        GOPRO_HEARTBEAT = 215,
//        GOPRO_GET_REQUEST = 216,
//        GOPRO_GET_RESPONSE = 217,
//        GOPRO_SET_REQUEST = 218,
//        GOPRO_SET_RESPONSE = 219,
//        EFI_STATUS = 225,
//        RPM = 226,
//        ESTIMATOR_STATUS = 230,
//        WIND_COV = 231,
//        GPS_INPUT = 232,
//        GPS_RTCM_DATA = 233,
//        HIGH_LATENCY = 234,
//        HIGH_LATENCY2 = 235,
//        VIBRATION = 241,
//        HOME_POSITION = 242,
//        SET_HOME_POSITION = 243,
//        MESSAGE_INTERVAL = 244,
//        EXTENDED_SYS_STATE = 245,
//        ADSB_VEHICLE = 246,
//        COLLISION = 247,
//        V2_EXTENSION = 248,
//        MEMORY_VECT = 249,
//        DEBUG_VECT = 250,
//        NAMED_VALUE_FLOAT = 251,
//        NAMED_VALUE_INT = 252,
//        STATUSTEXT = 253,
//        DEBUG = 254,
//        SETUP_SIGNING = 256,
//        BUTTON_CHANGE = 257,
//        PLAY_TUNE = 258,
//        CAMERA_INFORMATION = 259,
//        CAMERA_SETTINGS = 260,
//        STORAGE_INFORMATION = 261,
//        CAMERA_CAPTURE_STATUS = 262,
//        CAMERA_IMAGE_CAPTURED = 263,
//        FLIGHT_INFORMATION = 264,
//        MOUNT_ORIENTATION = 265,
//        LOGGING_DATA = 266,
//        LOGGING_DATA_ACKED = 267,
//        LOGGING_ACK = 268,
//        VIDEO_STREAM_INFORMATION = 269,
//        VIDEO_STREAM_STATUS = 270,
//        GIMBAL_DEVICE_INFORMATION = 283,
//        GIMBAL_DEVICE_SET_ATTITUDE = 284,
//        GIMBAL_DEVICE_ATTITUDE_STATUS = 285,
//        AUTOPILOT_STATE_FOR_GIMBAL_DEVICE = 286,
//        WIFI_CONFIG_AP = 299,
//        AIS_VESSEL = 301,
//        UAVCAN_NODE_STATUS = 310,
//        UAVCAN_NODE_INFO = 311,
//        PARAM_EXT_REQUEST_READ = 320,
//        PARAM_EXT_REQUEST_LIST = 321,
//        PARAM_EXT_VALUE = 322,
//        PARAM_EXT_SET = 323,
//        PARAM_EXT_ACK = 324,
//        OBSTACLE_DISTANCE = 330,
//        ODOMETRY = 331,
//        ISBD_LINK_STATUS = 335,
//        RAW_RPM = 339,
//        UTM_GLOBAL_POSITION = 340,
//        DEBUG_FLOAT_ARRAY = 350,
//        SMART_BATTERY_INFO = 370,
//        GENERATOR_STATUS = 373,
//        ACTUATOR_OUTPUT_STATUS = 375,
//        TUNNEL = 385,
//        CAN_FRAME = 386,
//        CANFD_FRAME = 387,
//        CAN_FILTER_MODIFY = 388,
//        WHEEL_DISTANCE = 9000,
//        WINCH_STATUS = 9005,
//        UAVIONIX_ADSB_OUT_CFG = 10001,
//        UAVIONIX_ADSB_OUT_DYNAMIC = 10002,
//        UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT = 10003,
//        UAVIONIX_ADSB_OUT_CFG_REGISTRATION = 10004,
//        UAVIONIX_ADSB_OUT_CFG_FLIGHTID = 10005,
//        UAVIONIX_ADSB_GET = 10006,
//        UAVIONIX_ADSB_OUT_CONTROL = 10007,
//        UAVIONIX_ADSB_OUT_STATUS = 10008,
//        DEVICE_OP_READ = 11000,
//        DEVICE_OP_READ_REPLY = 11001,
//        DEVICE_OP_WRITE = 11002,
//        DEVICE_OP_WRITE_REPLY = 11003,
//        ADAP_TUNING = 11010,
//        VISION_POSITION_DELTA = 11011,
//        AOA_SSA = 11020,
//        ESC_TELEMETRY_1_TO_4 = 11030,
//        ESC_TELEMETRY_5_TO_8 = 11031,
//        ESC_TELEMETRY_9_TO_12 = 11032,
//        OSD_PARAM_CONFIG = 11033,
//        OSD_PARAM_CONFIG_REPLY = 11034,
//        OSD_PARAM_SHOW_CONFIG = 11035,
//        OSD_PARAM_SHOW_CONFIG_REPLY = 11036,
//        OBSTACLE_DISTANCE_3D = 11037,
//        WATER_DEPTH = 11038,
//        MCU_STATUS = 11039,
//        ESC_TELEMETRY_13_TO_16 = 11040,
//        ESC_TELEMETRY_17_TO_20 = 11041,
//        ESC_TELEMETRY_21_TO_24 = 11042,
//        ESC_TELEMETRY_25_TO_28 = 11043,
//        ESC_TELEMETRY_29_TO_32 = 11044,
//        HYGROMETER_SENSOR = 12920,
//        ICAROUS_HEARTBEAT = 42000,
//        ICAROUS_KINEMATIC_BANDS = 42001,
//        VIDEO_STREAM_INFORMATION99 = 26900,
//        HERELINK_TELEM = 50003,
//        HEARTBEAT = 0,
//    }

//    /// extensions_start 0
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 42)]
//    ///<summary> Message appropriate for high latency connections like Iridium (version 2) </summary>
//    public struct mavlink_high_latency2_t {
//        public mavlink_high_latency2_t(uint timestamp, int latitude, int longitude, ushort custom_mode, short altitude, short target_altitude, ushort target_distance, ushort wp_num,/*HL_FAILURE_FLAG*/ushort failure_flags,/*MAV_TYPE*/byte type,/*MAV_AUTOPILOT*/byte autopilot, byte heading, byte target_heading, byte throttle, byte airspeed, byte airspeed_sp, byte groundspeed, byte windspeed, byte wind_heading, byte eph, byte epv, sbyte temperature_air, sbyte climb_rate, sbyte battery, sbyte custom0, sbyte custom1, sbyte custom2) {
//            this.timestamp = timestamp;
//            this.latitude = latitude;
//            this.longitude = longitude;
//            this.custom_mode = custom_mode;
//            this.altitude = altitude;
//            this.target_altitude = target_altitude;
//            this.target_distance = target_distance;
//            this.wp_num = wp_num;
//            this.failure_flags = failure_flags;
//            this.type = type;
//            this.autopilot = autopilot;
//            this.heading = heading;
//            this.target_heading = target_heading;
//            this.throttle = throttle;
//            this.airspeed = airspeed;
//            this.airspeed_sp = airspeed_sp;
//            this.groundspeed = groundspeed;
//            this.windspeed = windspeed;
//            this.wind_heading = wind_heading;
//            this.eph = eph;
//            this.epv = epv;
//            this.temperature_air = temperature_air;
//            this.climb_rate = climb_rate;
//            this.battery = battery;
//            this.custom0 = custom0;
//            this.custom1 = custom1;
//            this.custom2 = custom2;

//        }

//        /// <summary>Timestamp (milliseconds since boot or Unix epoch)  [ms] </summary>
//        [Units("[ms]")]
//        [Description("Timestamp (milliseconds since boot or Unix epoch)")]
//        //[FieldOffset(0)]
//        public uint timestamp;

//        /// <summary>Latitude  [degE7] </summary>
//        [Units("[degE7]")]
//        [Description("Latitude")]
//        //[FieldOffset(4)]
//        public int latitude;

//        /// <summary>Longitude  [degE7] </summary>
//        [Units("[degE7]")]
//        [Description("Longitude")]
//        //[FieldOffset(8)]
//        public int longitude;

//        /// <summary>A bitfield for use for autopilot-specific flags (2 byte version).   bitmask</summary>
//        [Units("")]
//        [Description("A bitfield for use for autopilot-specific flags (2 byte version).")]
//        //[FieldOffset(12)]
//        public ushort custom_mode;

//        /// <summary>Altitude above mean sea level  [m] </summary>
//        [Units("[m]")]
//        [Description("Altitude above mean sea level")]
//        //[FieldOffset(14)]
//        public short altitude;

//        /// <summary>Altitude setpoint  [m] </summary>
//        [Units("[m]")]
//        [Description("Altitude setpoint")]
//        //[FieldOffset(16)]
//        public short target_altitude;

//        /// <summary>Distance to target waypoint or position  [dam] </summary>
//        [Units("[dam]")]
//        [Description("Distance to target waypoint or position")]
//        //[FieldOffset(18)]
//        public ushort target_distance;

//        /// <summary>Current waypoint number   </summary>
//        [Units("")]
//        [Description("Current waypoint number")]
//        //[FieldOffset(20)]
//        public ushort wp_num;

//        /// <summary>Bitmap of failure flags. HL_FAILURE_FLAG  bitmask</summary>
//        [Units("")]
//        [Description("Bitmap of failure flags.")]
//        //[FieldOffset(22)]
//        public  /*HL_FAILURE_FLAG*/ushort failure_flags;

//        /// <summary>Type of the MAV (quadrotor, helicopter, etc.) MAV_TYPE  </summary>
//        [Units("")]
//        [Description("Type of the MAV (quadrotor, helicopter, etc.)")]
//        //[FieldOffset(24)]
//        public  /*MAV_TYPE*/byte type;

//        /// <summary>Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers. MAV_AUTOPILOT  </summary>
//        [Units("")]
//        [Description("Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.")]
//        //[FieldOffset(25)]
//        public  /*MAV_AUTOPILOT*/byte autopilot;

//        /// <summary>Heading  [deg/2] </summary>
//        [Units("[deg/2]")]
//        [Description("Heading")]
//        //[FieldOffset(26)]
//        public byte heading;

//        /// <summary>Heading setpoint  [deg/2] </summary>
//        [Units("[deg/2]")]
//        [Description("Heading setpoint")]
//        //[FieldOffset(27)]
//        public byte target_heading;

//        /// <summary>Throttle  [%] </summary>
//        [Units("[%]")]
//        [Description("Throttle")]
//        //[FieldOffset(28)]
//        public byte throttle;

//        /// <summary>Airspeed  [m/s*5] </summary>
//        [Units("[m/s*5]")]
//        [Description("Airspeed")]
//        //[FieldOffset(29)]
//        public byte airspeed;

//        /// <summary>Airspeed setpoint  [m/s*5] </summary>
//        [Units("[m/s*5]")]
//        [Description("Airspeed setpoint")]
//        //[FieldOffset(30)]
//        public byte airspeed_sp;

//        /// <summary>Groundspeed  [m/s*5] </summary>
//        [Units("[m/s*5]")]
//        [Description("Groundspeed")]
//        //[FieldOffset(31)]
//        public byte groundspeed;

//        /// <summary>Windspeed  [m/s*5] </summary>
//        [Units("[m/s*5]")]
//        [Description("Windspeed")]
//        //[FieldOffset(32)]
//        public byte windspeed;

//        /// <summary>Wind heading  [deg/2] </summary>
//        [Units("[deg/2]")]
//        [Description("Wind heading")]
//        //[FieldOffset(33)]
//        public byte wind_heading;

//        /// <summary>Maximum error horizontal position since last message  [dm] </summary>
//        [Units("[dm]")]
//        [Description("Maximum error horizontal position since last message")]
//        //[FieldOffset(34)]
//        public byte eph;

//        /// <summary>Maximum error vertical position since last message  [dm] </summary>
//        [Units("[dm]")]
//        [Description("Maximum error vertical position since last message")]
//        //[FieldOffset(35)]
//        public byte epv;

//        /// <summary>Air temperature from airspeed sensor  [degC] </summary>
//        [Units("[degC]")]
//        [Description("Air temperature from airspeed sensor")]
//        //[FieldOffset(36)]
//        public sbyte temperature_air;

//        /// <summary>Maximum climb rate magnitude since last message  [dm/s] </summary>
//        [Units("[dm/s]")]
//        [Description("Maximum climb rate magnitude since last message")]
//        //[FieldOffset(37)]
//        public sbyte climb_rate;

//        /// <summary>Battery level (-1 if field not provided).  [%] </summary>
//        [Units("[%]")]
//        [Description("Battery level (-1 if field not provided).")]
//        //[FieldOffset(38)]
//        public sbyte battery;

//        /// <summary>Field for custom payload.   </summary>
//        [Units("")]
//        [Description("Field for custom payload.")]
//        //[FieldOffset(39)]
//        public sbyte custom0;

//        /// <summary>Field for custom payload.   </summary>
//        [Units("")]
//        [Description("Field for custom payload.")]
//        //[FieldOffset(40)]
//        public sbyte custom1;

//        /// <summary>Field for custom payload.   </summary>
//        [Units("")]
//        [Description("Field for custom payload.")]
//        //[FieldOffset(41)]
//        public sbyte custom2;
//    };


//}