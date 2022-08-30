
//using System;
//using System.Diagnostics;
//using System.Runtime.InteropServices;
//using static LibDrone.Frames.Text;

//public partial class MAVLink {


//    /// extensions_start 0
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 6)]
//    ///<summary> Request a data stream. </summary>
//    public struct mavlink_request_data_stream_t
//    {
//        public mavlink_request_data_stream_t(ushort req_message_rate, byte target_system, byte target_component, byte req_stream_id, byte start_stop)
//        {
//            this.req_message_rate = req_message_rate;
//            this.target_system = target_system;
//            this.target_component = target_component;
//            this.req_stream_id = req_stream_id;
//            this.start_stop = start_stop;

//        }

//        /// <summary>The requested message rate  [Hz] </summary>
//        [Units("[Hz]")]
//        [Description("The requested message rate")]
//        //[FieldOffset(0)]
//        public ushort req_message_rate;

//        /// <summary>The target requested to send the message stream.   </summary>
//        [Units("")]
//        [Description("The target requested to send the message stream.")]
//        //[FieldOffset(2)]
//        public byte target_system;

//        /// <summary>The target requested to send the message stream.   </summary>
//        [Units("")]
//        [Description("The target requested to send the message stream.")]
//        //[FieldOffset(3)]
//        public byte target_component;

//        /// <summary>The ID of the requested data stream   </summary>
//        [Units("")]
//        [Description("The ID of the requested data stream")]
//        //[FieldOffset(4)]
//        public byte req_stream_id;

//        /// <summary>1 to start sending, 0 to stop sending.   </summary>
//        [Units("")]
//        [Description("1 to start sending, 0 to stop sending.")]
//        //[FieldOffset(5)]
//        public byte start_stop;
//    };

//    /// extensions_start 0
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 53)]
//    ///<summary> Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84). Used by an external controller to command the vehicle (manual controller or other system). </summary>
//    public struct mavlink_set_position_target_global_int_t {
//        public mavlink_set_position_target_global_int_t(uint time_boot_ms, int lat_int, int lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate,/*POSITION_TARGET_TYPEMASK*/ushort type_mask, byte target_system, byte target_component,/*MAV_FRAME*/byte coordinate_frame) {
//            this.time_boot_ms = time_boot_ms;
//            this.lat_int = lat_int;
//            this.lon_int = lon_int;
//            this.alt = alt;
//            this.vx = vx;
//            this.vy = vy;
//            this.vz = vz;
//            this.afx = afx;
//            this.afy = afy;
//            this.afz = afz;
//            this.yaw = yaw;
//            this.yaw_rate = yaw_rate;
//            this.type_mask = type_mask;
//            this.target_system = target_system;
//            this.target_component = target_component;
//            this.coordinate_frame = coordinate_frame;

//        }

//        /// <summary>Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.  [ms] </summary>
//        [Units("[ms]")]
//        [Description("Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.")]
//        //[FieldOffset(0)]
//        public uint time_boot_ms;

//        /// <summary>X Position in WGS84 frame  [degE7] </summary>
//        [Units("[degE7]")]
//        [Description("X Position in WGS84 frame")]
//        //[FieldOffset(4)]
//        public int lat_int;

//        /// <summary>Y Position in WGS84 frame  [degE7] </summary>
//        [Units("[degE7]")]
//        [Description("Y Position in WGS84 frame")]
//        //[FieldOffset(8)]
//        public int lon_int;

//        /// <summary>Altitude (MSL, Relative to home, or AGL - depending on frame)  [m] </summary>
//        [Units("[m]")]
//        [Description("Altitude (MSL, Relative to home, or AGL - depending on frame)")]
//        //[FieldOffset(12)]
//        public float alt;

//        /// <summary>X velocity in NED frame  [m/s] </summary>
//        [Units("[m/s]")]
//        [Description("X velocity in NED frame")]
//        //[FieldOffset(16)]
//        public float vx;

//        /// <summary>Y velocity in NED frame  [m/s] </summary>
//        [Units("[m/s]")]
//        [Description("Y velocity in NED frame")]
//        //[FieldOffset(20)]
//        public float vy;

//        /// <summary>Z velocity in NED frame  [m/s] </summary>
//        [Units("[m/s]")]
//        [Description("Z velocity in NED frame")]
//        //[FieldOffset(24)]
//        public float vz;

//        /// <summary>X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N  [m/s/s] </summary>
//        [Units("[m/s/s]")]
//        [Description("X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N")]
//        //[FieldOffset(28)]
//        public float afx;

//        /// <summary>Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N  [m/s/s] </summary>
//        [Units("[m/s/s]")]
//        [Description("Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N")]
//        //[FieldOffset(32)]
//        public float afy;

//        /// <summary>Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N  [m/s/s] </summary>
//        [Units("[m/s/s]")]
//        [Description("Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N")]
//        //[FieldOffset(36)]
//        public float afz;

//        /// <summary>yaw setpoint  [rad] </summary>
//        [Units("[rad]")]
//        [Description("yaw setpoint")]
//        //[FieldOffset(40)]
//        public float yaw;

//        /// <summary>yaw rate setpoint  [rad/s] </summary>
//        [Units("[rad/s]")]
//        [Description("yaw rate setpoint")]
//        //[FieldOffset(44)]
//        public float yaw_rate;

//        /// <summary>Bitmap to indicate which dimensions should be ignored by the vehicle. POSITION_TARGET_TYPEMASK  bitmask</summary>
//        [Units("")]
//        [Description("Bitmap to indicate which dimensions should be ignored by the vehicle.")]
//        //[FieldOffset(48)]
//        public  /*POSITION_TARGET_TYPEMASK*/ushort type_mask;

//        /// <summary>System ID   </summary>
//        [Units("")]
//        [Description("System ID")]
//        //[FieldOffset(50)]
//        public byte target_system;

//        /// <summary>Component ID   </summary>
//        [Units("")]
//        [Description("Component ID")]
//        //[FieldOffset(51)]
//        public byte target_component;

//        /// <summary>Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11 MAV_FRAME  </summary>
//        [Units("")]
//        [Description("Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11")]
//        //[FieldOffset(52)]
//        public  /*MAV_FRAME*/byte coordinate_frame;
//    };


//    ///<summary> Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/. </summary>
//    public enum MAV_SEVERITY : byte {
//        ///<summary> System is unusable. This is a 'panic' condition. | </summary>
//        [Description("System is unusable. This is a 'panic' condition.")]
//        EMERGENCY = 0,
//        ///<summary> Action should be taken immediately. Indicates error in non-critical systems. | </summary>
//        [Description("Action should be taken immediately. Indicates error in non-critical systems.")]
//        ALERT = 1,
//        ///<summary> Action must be taken immediately. Indicates failure in a primary system. | </summary>
//        [Description("Action must be taken immediately. Indicates failure in a primary system.")]
//        CRITICAL = 2,
//        ///<summary> Indicates an error in secondary/redundant systems. | </summary>
//        [Description("Indicates an error in secondary/redundant systems.")]
//        ERROR = 3,
//        ///<summary> Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. | </summary>
//        [Description("Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.")]
//        WARNING = 4,
//        ///<summary> An unusual event has occurred, though not an error condition. This should be investigated for the root cause. | </summary>
//        [Description("An unusual event has occurred, though not an error condition. This should be investigated for the root cause.")]
//        NOTICE = 5,
//        ///<summary> Normal operational messages. Useful for logging. No action is required for these messages. | </summary>
//        [Description("Normal operational messages. Useful for logging. No action is required for these messages.")]
//        INFO = 6,
//        ///<summary> Useful non-operational messages that can assist in debugging. These should not occur during normal operation. | </summary>
//        [Description("Useful non-operational messages that can assist in debugging. These should not occur during normal operation.")]
//        DEBUG = 7,

//    };


//    /// extensions_start 2
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 54)]
//    ///<summary> Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz). </summary>
//    public struct mavlink_statustext_t {
//        public mavlink_statustext_t(/*MAV_SEVERITY*/byte severity, byte[] text, ushort id, byte chunk_seq) {
//            this.severity = severity;
//            this.text = text;
//            this.id = id;
//            this.chunk_seq = chunk_seq;

//        }

//        /// <summary>Severity of status. Relies on the definitions within RFC-5424. MAV_SEVERITY  </summary>
//        [Units("")]
//        [Description("Severity of status. Relies on the definitions within RFC-5424.")]
//        //[FieldOffset(0)]
//        public  /*MAV_SEVERITY*/byte severity;

//        /// <summary>Status text message, without null termination character   </summary>
//        [Units("")]
//        [Description("Status text message, without null termination character")]
//        //[FieldOffset(1)]
//        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 50)]
//        public byte[] text;

//        /// <summary>Unique (opaque) identifier for this statustext message.  May be used to reassemble a logical long-statustext message from a sequence of chunks.  A value of zero indicates this is the only chunk in the sequence and the message can be emitted immediately.   </summary>
//        [Units("")]
//        [Description("Unique (opaque) identifier for this statustext message.  May be used to reassemble a logical long-statustext message from a sequence of chunks.  A value of zero indicates this is the only chunk in the sequence and the message can be emitted immediately.")]
//        //[FieldOffset(51)]
//        public ushort id;

//        /// <summary>This chunk's sequence number; indexing is from zero.  Any null character in the text field is taken to mean this was the last chunk.   </summary>
//        [Units("")]
//        [Description("This chunk's sequence number; indexing is from zero.  Any null character in the text field is taken to mean this was the last chunk.")]
//        //[FieldOffset(53)]
//        public byte chunk_seq;
//    };

//    /// extensions_start 0
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 116)]
//    ///<summary> General information describing a particular UAVCAN node. Please refer to the definition of the UAVCAN service 'uavcan.protocol.GetNodeInfo' for the background information. This message should be emitted by the system whenever a new node appears online, or an existing node reboots. Additionally, it can be emitted upon request from the other end of the MAVLink channel (see MAV_CMD_UAVCAN_GET_NODE_INFO). It is also not prohibited to emit this message unconditionally at a low frequency. The UAVCAN specification is available at http://uavcan.org. </summary>
//    public struct mavlink_uavcan_node_info_t {
//        public mavlink_uavcan_node_info_t(ulong time_usec, uint uptime_sec, uint sw_vcs_commit, byte[] name, byte hw_version_major, byte hw_version_minor, byte[] hw_unique_id, byte sw_version_major, byte sw_version_minor) {
//            this.time_usec = time_usec;
//            this.uptime_sec = uptime_sec;
//            this.sw_vcs_commit = sw_vcs_commit;
//            this.name = name;
//            this.hw_version_major = hw_version_major;
//            this.hw_version_minor = hw_version_minor;
//            this.hw_unique_id = hw_unique_id;
//            this.sw_version_major = sw_version_major;
//            this.sw_version_minor = sw_version_minor;

//        }

//        /// <summary>Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.  [us] </summary>
//        [Units("[us]")]
//        [Description("Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.")]
//        //[FieldOffset(0)]
//        public ulong time_usec;

//        /// <summary>Time since the start-up of the node.  [s] </summary>
//        [Units("[s]")]
//        [Description("Time since the start-up of the node.")]
//        //[FieldOffset(8)]
//        public uint uptime_sec;

//        /// <summary>Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.   </summary>
//        [Units("")]
//        [Description("Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.")]
//        //[FieldOffset(12)]
//        public uint sw_vcs_commit;

//        /// <summary>Node name string. For example, 'sapog.px4.io'.   </summary>
//        [Units("")]
//        [Description("Node name string. For example, 'sapog.px4.io'.")]
//        //[FieldOffset(16)]
//        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 80)]
//        public byte[] name;

//        /// <summary>Hardware major version number.   </summary>
//        [Units("")]
//        [Description("Hardware major version number.")]
//        //[FieldOffset(96)]
//        public byte hw_version_major;

//        /// <summary>Hardware minor version number.   </summary>
//        [Units("")]
//        [Description("Hardware minor version number.")]
//        //[FieldOffset(97)]
//        public byte hw_version_minor;

//        /// <summary>Hardware unique 128-bit ID.   </summary>
//        [Units("")]
//        [Description("Hardware unique 128-bit ID.")]
//        //[FieldOffset(98)]
//        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
//        public byte[] hw_unique_id;

//        /// <summary>Software major version number.   </summary>
//        [Units("")]
//        [Description("Software major version number.")]
//        //[FieldOffset(114)]
//        public byte sw_version_major;

//        /// <summary>Software minor version number.   </summary>
//        [Units("")]
//        [Description("Software minor version number.")]
//        //[FieldOffset(115)]
//        public byte sw_version_minor;
//    };


//    /// extensions_start 0
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 17)]
//    ///<summary> General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message 'uavcan.protocol.NodeStatus' for the background information. The UAVCAN specification is available at http://uavcan.org. </summary>
//    public struct mavlink_uavcan_node_status_t {
//        public mavlink_uavcan_node_status_t(ulong time_usec, uint uptime_sec, ushort vendor_specific_status_code,/*UAVCAN_NODE_HEALTH*/byte health,/*UAVCAN_NODE_MODE*/byte mode, byte sub_mode) {
//            this.time_usec = time_usec;
//            this.uptime_sec = uptime_sec;
//            this.vendor_specific_status_code = vendor_specific_status_code;
//            this.health = health;
//            this.mode = mode;
//            this.sub_mode = sub_mode;

//        }

//        /// <summary>Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.  [us] </summary>
//        [Units("[us]")]
//        [Description("Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.")]
//        //[FieldOffset(0)]
//        public ulong time_usec;

//        /// <summary>Time since the start-up of the node.  [s] </summary>
//        [Units("[s]")]
//        [Description("Time since the start-up of the node.")]
//        //[FieldOffset(8)]
//        public uint uptime_sec;

//        /// <summary>Vendor-specific status information.   </summary>
//        [Units("")]
//        [Description("Vendor-specific status information.")]
//        //[FieldOffset(12)]
//        public ushort vendor_specific_status_code;

//        /// <summary>Generalized node health status. UAVCAN_NODE_HEALTH  </summary>
//        [Units("")]
//        [Description("Generalized node health status.")]
//        //[FieldOffset(14)]
//        public  /*UAVCAN_NODE_HEALTH*/byte health;

//        /// <summary>Generalized operating mode. UAVCAN_NODE_MODE  </summary>
//        [Units("")]
//        [Description("Generalized operating mode.")]
//        //[FieldOffset(15)]
//        public  /*UAVCAN_NODE_MODE*/byte mode;

//        /// <summary>Not used currently.   </summary>
//        [Units("")]
//        [Description("Not used currently.")]
//        //[FieldOffset(16)]
//        public byte sub_mode;
//    };

//    ///<summary> Possible actions an aircraft can take to avoid a collision. </summary>
//    public enum MAV_COLLISION_ACTION : byte {
//        ///<summary> Ignore any potential collisions | </summary>
//        [Description("Ignore any potential collisions")]
//        NONE = 0,
//        ///<summary> Report potential collision | </summary>
//        [Description("Report potential collision")]
//        REPORT = 1,
//        ///<summary> Ascend or Descend to avoid threat | </summary>
//        [Description("Ascend or Descend to avoid threat")]
//        ASCEND_OR_DESCEND = 2,
//        ///<summary> Move horizontally to avoid threat | </summary>
//        [Description("Move horizontally to avoid threat")]
//        MOVE_HORIZONTALLY = 3,
//        ///<summary> Aircraft to move perpendicular to the collision's velocity vector | </summary>
//        [Description("Aircraft to move perpendicular to the collision's velocity vector")]
//        MOVE_PERPENDICULAR = 4,
//        ///<summary> Aircraft to fly directly back to its launch point | </summary>
//        [Description("Aircraft to fly directly back to its launch point")]
//        RTL = 5,
//        ///<summary> Aircraft to stop in place | </summary>
//        [Description("Aircraft to stop in place")]
//        HOVER = 6,

//    };

//    ///<summary> Aircraft-rated danger from this threat. </summary>
//    public enum MAV_COLLISION_THREAT_LEVEL : byte {
//        ///<summary> Not a threat | </summary>
//        [Description("Not a threat")]
//        NONE = 0,
//        ///<summary> Craft is mildly concerned about this threat | </summary>
//        [Description("Craft is mildly concerned about this threat")]
//        LOW = 1,
//        ///<summary> Craft is panicking, and may take actions to avoid threat | </summary>
//        [Description("Craft is panicking, and may take actions to avoid threat")]
//        HIGH = 2,

//    };


//    ///<summary> Source of information about this collision. </summary>
//    public enum MAV_COLLISION_SRC : byte {
//        ///<summary> ID field references ADSB_VEHICLE packets | </summary>
//        [Description("ID field references ADSB_VEHICLE packets")]
//        ADSB = 0,
//        ///<summary> ID field references MAVLink SRC ID | </summary>
//        [Description("ID field references MAVLink SRC ID")]
//        MAVLINK_GPS_GLOBAL_INT = 1,

//    };

//    /// extensions_start 0
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 19)]
//    ///<summary> Information about a potential collision </summary>
//    public struct mavlink_collision_t {
//        public mavlink_collision_t(uint id, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta,/*MAV_COLLISION_SRC*/byte src,/*MAV_COLLISION_ACTION*/byte action,/*MAV_COLLISION_THREAT_LEVEL*/byte threat_level) {
//            this.id = id;
//            this.time_to_minimum_delta = time_to_minimum_delta;
//            this.altitude_minimum_delta = altitude_minimum_delta;
//            this.horizontal_minimum_delta = horizontal_minimum_delta;
//            this.src = src;
//            this.action = action;
//            this.threat_level = threat_level;

//        }

//        /// <summary>Unique identifier, domain based on src field   </summary>
//        [Units("")]
//        [Description("Unique identifier, domain based on src field")]
//        //[FieldOffset(0)]
//        public uint id;

//        /// <summary>Estimated time until collision occurs  [s] </summary>
//        [Units("[s]")]
//        [Description("Estimated time until collision occurs")]
//        //[FieldOffset(4)]
//        public float time_to_minimum_delta;

//        /// <summary>Closest vertical distance between vehicle and object  [m] </summary>
//        [Units("[m]")]
//        [Description("Closest vertical distance between vehicle and object")]
//        //[FieldOffset(8)]
//        public float altitude_minimum_delta;

//        /// <summary>Closest horizontal distance between vehicle and object  [m] </summary>
//        [Units("[m]")]
//        [Description("Closest horizontal distance between vehicle and object")]
//        //[FieldOffset(12)]
//        public float horizontal_minimum_delta;

//        /// <summary>Collision data source MAV_COLLISION_SRC  </summary>
//        [Units("")]
//        [Description("Collision data source")]
//        //[FieldOffset(16)]
//        public  /*MAV_COLLISION_SRC*/byte src;

//        /// <summary>Action that is being taken to avoid this collision MAV_COLLISION_ACTION  </summary>
//        [Units("")]
//        [Description("Action that is being taken to avoid this collision")]
//        //[FieldOffset(17)]
//        public  /*MAV_COLLISION_ACTION*/byte action;

//        /// <summary>How concerned the aircraft is about this collision MAV_COLLISION_THREAT_LEVEL  </summary>
//        [Units("")]
//        [Description("How concerned the aircraft is about this collision")]
//        //[FieldOffset(18)]
//        public  /*MAV_COLLISION_THREAT_LEVEL*/byte threat_level;
//    };

//    /// extensions_start 0
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 38)]
//    ///<summary> The location and information of an ADSB vehicle </summary>
//    public struct mavlink_adsb_vehicle_t {
//        public mavlink_adsb_vehicle_t(uint ICAO_address, int lat, int lon, int altitude, ushort heading, ushort hor_velocity, short ver_velocity,/*ADSB_FLAGS*/ushort flags, ushort squawk,/*ADSB_ALTITUDE_TYPE*/byte altitude_type, byte[] callsign,/*ADSB_EMITTER_TYPE*/byte emitter_type, byte tslc) {
//            this.ICAO_address = ICAO_address;
//            this.lat = lat;
//            this.lon = lon;
//            this.altitude = altitude;
//            this.heading = heading;
//            this.hor_velocity = hor_velocity;
//            this.ver_velocity = ver_velocity;
//            this.flags = flags;
//            this.squawk = squawk;
//            this.altitude_type = altitude_type;
//            this.callsign = callsign;
//            this.emitter_type = emitter_type;
//            this.tslc = tslc;

//        }

//        /// <summary>ICAO address   </summary>
//        [Units("")]
//        [Description("ICAO address")]
//        //[FieldOffset(0)]
//        public uint ICAO_address;

//        /// <summary>Latitude  [degE7] </summary>
//        [Units("[degE7]")]
//        [Description("Latitude")]
//        //[FieldOffset(4)]
//        public int lat;

//        /// <summary>Longitude  [degE7] </summary>
//        [Units("[degE7]")]
//        [Description("Longitude")]
//        //[FieldOffset(8)]
//        public int lon;

//        /// <summary>Altitude(ASL)  [mm] </summary>
//        [Units("[mm]")]
//        [Description("Altitude(ASL)")]
//        //[FieldOffset(12)]
//        public int altitude;

//        /// <summary>Course over ground  [cdeg] </summary>
//        [Units("[cdeg]")]
//        [Description("Course over ground")]
//        //[FieldOffset(16)]
//        public ushort heading;

//        /// <summary>The horizontal velocity  [cm/s] </summary>
//        [Units("[cm/s]")]
//        [Description("The horizontal velocity")]
//        //[FieldOffset(18)]
//        public ushort hor_velocity;

//        /// <summary>The vertical velocity. Positive is up  [cm/s] </summary>
//        [Units("[cm/s]")]
//        [Description("The vertical velocity. Positive is up")]
//        //[FieldOffset(20)]
//        public short ver_velocity;

//        /// <summary>Bitmap to indicate various statuses including valid data fields ADSB_FLAGS  bitmask</summary>
//        [Units("")]
//        [Description("Bitmap to indicate various statuses including valid data fields")]
//        //[FieldOffset(22)]
//        public  /*ADSB_FLAGS*/ushort flags;

//        /// <summary>Squawk code   </summary>
//        [Units("")]
//        [Description("Squawk code")]
//        //[FieldOffset(24)]
//        public ushort squawk;

//        /// <summary>ADSB altitude type. ADSB_ALTITUDE_TYPE  </summary>
//        [Units("")]
//        [Description("ADSB altitude type.")]
//        //[FieldOffset(26)]
//        public  /*ADSB_ALTITUDE_TYPE*/byte altitude_type;

//        /// <summary>The callsign, 8+null   </summary>
//        [Units("")]
//        [Description("The callsign, 8+null")]
//        //[FieldOffset(27)]
//        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)]
//        public byte[] callsign;

//        /// <summary>ADSB emitter type. ADSB_EMITTER_TYPE  </summary>
//        [Units("")]
//        [Description("ADSB emitter type.")]
//        //[FieldOffset(36)]
//        public  /*ADSB_EMITTER_TYPE*/byte emitter_type;

//        /// <summary>Time since last communication in seconds  [s] </summary>
//        [Units("[s]")]
//        [Description("Time since last communication in seconds")]
//        //[FieldOffset(37)]
//        public byte tslc;
//    };


//    /// extensions_start 0
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 16)]
//    ///<summary> Time synchronization message. </summary>
//    public struct mavlink_timesync_t {
//        public mavlink_timesync_t(long tc1, long ts1) {
//            this.tc1 = tc1;
//            this.ts1 = ts1;

//        }

//        /// <summary>Time sync timestamp 1   </summary>
//        [Units("")]
//        [Description("Time sync timestamp 1")]
//        //[FieldOffset(0)]
//        public long tc1;

//        /// <summary>Time sync timestamp 2   </summary>
//        [Units("")]
//        [Description("Time sync timestamp 2")]
//        //[FieldOffset(8)]
//        public long ts1;
//    };

//    /// extensions_start 0
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 12)]
//    ///<summary> A fence point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS. </summary>
//    public struct mavlink_fence_point_t {
//        public mavlink_fence_point_t(float lat, float lng, byte target_system, byte target_component, byte idx, byte count) {
//            this.lat = lat;
//            this.lng = lng;
//            this.target_system = target_system;
//            this.target_component = target_component;
//            this.idx = idx;
//            this.count = count;

//        }

//        /// <summary>Latitude of point.  [deg] </summary>
//        [Units("[deg]")]
//        [Description("Latitude of point.")]
//        //[FieldOffset(0)]
//        public float lat;

//        /// <summary>Longitude of point.  [deg] </summary>
//        [Units("[deg]")]
//        [Description("Longitude of point.")]
//        //[FieldOffset(4)]
//        public float lng;

//        /// <summary>System ID.   </summary>
//        [Units("")]
//        [Description("System ID.")]
//        //[FieldOffset(8)]
//        public byte target_system;

//        /// <summary>Component ID.   </summary>
//        [Units("")]
//        [Description("Component ID.")]
//        //[FieldOffset(9)]
//        public byte target_component;

//        /// <summary>Point index (first point is 1, 0 is for return point).   </summary>
//        [Units("")]
//        [Description("Point index (first point is 1, 0 is for return point).")]
//        //[FieldOffset(10)]
//        public byte idx;

//        /// <summary>Total number of points (for sanity checking).   </summary>
//        [Units("")]
//        [Description("Total number of points (for sanity checking).")]
//        //[FieldOffset(11)]
//        public byte count;
//    };

//    /// extensions_start 0
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 19)]
//    ///<summary> A rally point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS. </summary>
//    public struct mavlink_rally_point_t {
//        public mavlink_rally_point_t(int lat, int lng, short alt, short break_alt, ushort land_dir, byte target_system, byte target_component, byte idx, byte count,/*RALLY_FLAGS*/byte flags) {
//            this.lat = lat;
//            this.lng = lng;
//            this.alt = alt;
//            this.break_alt = break_alt;
//            this.land_dir = land_dir;
//            this.target_system = target_system;
//            this.target_component = target_component;
//            this.idx = idx;
//            this.count = count;
//            this.flags = flags;

//        }

//        /// <summary>Latitude of point.  [degE7] </summary>
//        [Units("[degE7]")]
//        [Description("Latitude of point.")]
//        //[FieldOffset(0)]
//        public int lat;

//        /// <summary>Longitude of point.  [degE7] </summary>
//        [Units("[degE7]")]
//        [Description("Longitude of point.")]
//        //[FieldOffset(4)]
//        public int lng;

//        /// <summary>Transit / loiter altitude relative to home.  [m] </summary>
//        [Units("[m]")]
//        [Description("Transit / loiter altitude relative to home.")]
//        //[FieldOffset(8)]
//        public short alt;

//        /// <summary>Break altitude relative to home.  [m] </summary>
//        [Units("[m]")]
//        [Description("Break altitude relative to home.")]
//        //[FieldOffset(10)]
//        public short break_alt;

//        /// <summary>Heading to aim for when landing.  [cdeg] </summary>
//        [Units("[cdeg]")]
//        [Description("Heading to aim for when landing.")]
//        //[FieldOffset(12)]
//        public ushort land_dir;

//        /// <summary>System ID.   </summary>
//        [Units("")]
//        [Description("System ID.")]
//        //[FieldOffset(14)]
//        public byte target_system;

//        /// <summary>Component ID.   </summary>
//        [Units("")]
//        [Description("Component ID.")]
//        //[FieldOffset(15)]
//        public byte target_component;

//        /// <summary>Point index (first point is 0).   </summary>
//        [Units("")]
//        [Description("Point index (first point is 0).")]
//        //[FieldOffset(16)]
//        public byte idx;

//        /// <summary>Total number of points (for sanity checking).   </summary>
//        [Units("")]
//        [Description("Total number of points (for sanity checking).")]
//        //[FieldOffset(17)]
//        public byte count;

//        /// <summary>Configuration flags. RALLY_FLAGS  bitmask</summary>
//        [Units("")]
//        [Description("Configuration flags.")]
//        //[FieldOffset(18)]
//        public  /*RALLY_FLAGS*/byte flags;
//    };

//    /// extensions_start 10
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 60)]
//    ///<summary> This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitly set by the operator before or after. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector. </summary>
//    public struct mavlink_home_position_t {
//        public mavlink_home_position_t(int latitude, int longitude, int altitude, float x, float y, float z, float[] q, float approach_x, float approach_y, float approach_z, ulong time_usec) {
//            this.latitude = latitude;
//            this.longitude = longitude;
//            this.altitude = altitude;
//            this.x = x;
//            this.y = y;
//            this.z = z;
//            this.q = q;
//            this.approach_x = approach_x;
//            this.approach_y = approach_y;
//            this.approach_z = approach_z;
//            this.time_usec = time_usec;

//        }

//        /// <summary>Latitude (WGS84)  [degE7] </summary>
//        [Units("[degE7]")]
//        [Description("Latitude (WGS84)")]
//        //[FieldOffset(0)]
//        public int latitude;

//        /// <summary>Longitude (WGS84)  [degE7] </summary>
//        [Units("[degE7]")]
//        [Description("Longitude (WGS84)")]
//        //[FieldOffset(4)]
//        public int longitude;

//        /// <summary>Altitude (MSL). Positive for up.  [mm] </summary>
//        [Units("[mm]")]
//        [Description("Altitude (MSL). Positive for up.")]
//        //[FieldOffset(8)]
//        public int altitude;

//        /// <summary>Local X position of this position in the local coordinate frame  [m] </summary>
//        [Units("[m]")]
//        [Description("Local X position of this position in the local coordinate frame")]
//        //[FieldOffset(12)]
//        public float x;

//        /// <summary>Local Y position of this position in the local coordinate frame  [m] </summary>
//        [Units("[m]")]
//        [Description("Local Y position of this position in the local coordinate frame")]
//        //[FieldOffset(16)]
//        public float y;

//        /// <summary>Local Z position of this position in the local coordinate frame  [m] </summary>
//        [Units("[m]")]
//        [Description("Local Z position of this position in the local coordinate frame")]
//        //[FieldOffset(20)]
//        public float z;

//        /// <summary>World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground   </summary>
//        [Units("")]
//        [Description("World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground")]
//        //[FieldOffset(24)]
//        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
//        public float[] q;

//        /// <summary>Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.  [m] </summary>
//        [Units("[m]")]
//        [Description("Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.")]
//        //[FieldOffset(40)]
//        public float approach_x;

//        /// <summary>Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.  [m] </summary>
//        [Units("[m]")]
//        [Description("Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.")]
//        //[FieldOffset(44)]
//        public float approach_y;

//        /// <summary>Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.  [m] </summary>
//        [Units("[m]")]
//        [Description("Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.")]
//        //[FieldOffset(48)]
//        public float approach_z;

//        /// <summary>Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.  [us] </summary>
//        [Units("[us]")]
//        [Description("Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.")]
//        //[FieldOffset(52)]
//        public ulong time_usec;
//    };

//    [Obsolete]
//    /// extensions_start 14
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 38)]
//    ///<summary> Message encoding a mission item. This message is emitted to announce                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). NaN may be used to indicate an optional/default value (e.g. to use the system's current latitude or yaw rather than a specific value). See also https://mavlink.io/en/services/mission.html. </summary>
//    public struct mavlink_mission_item_t {
//        public mavlink_mission_item_t(float param1, float param2, float param3, float param4, float x, float y, float z, ushort seq,/*MAV_CMD*/ushort command, byte target_system, byte target_component,/*MAV_FRAME*/byte frame, byte current, byte autocontinue,/*MAV_MISSION_TYPE*/byte mission_type) {
//            this.param1 = param1;
//            this.param2 = param2;
//            this.param3 = param3;
//            this.param4 = param4;
//            this.x = x;
//            this.y = y;
//            this.z = z;
//            this.seq = seq;
//            this.command = command;
//            this.target_system = target_system;
//            this.target_component = target_component;
//            this.frame = frame;
//            this.current = current;
//            this.autocontinue = autocontinue;
//            this.mission_type = mission_type;

//        }

//        /// <summary>PARAM1, see MAV_CMD enum   </summary>
//        [Units("")]
//        [Description("PARAM1, see MAV_CMD enum")]
//        //[FieldOffset(0)]
//        public float param1;

//        /// <summary>PARAM2, see MAV_CMD enum   </summary>
//        [Units("")]
//        [Description("PARAM2, see MAV_CMD enum")]
//        //[FieldOffset(4)]
//        public float param2;

//        /// <summary>PARAM3, see MAV_CMD enum   </summary>
//        [Units("")]
//        [Description("PARAM3, see MAV_CMD enum")]
//        //[FieldOffset(8)]
//        public float param3;

//        /// <summary>PARAM4, see MAV_CMD enum   </summary>
//        [Units("")]
//        [Description("PARAM4, see MAV_CMD enum")]
//        //[FieldOffset(12)]
//        public float param4;

//        /// <summary>PARAM5 / local: X coordinate, global: latitude   </summary>
//        [Units("")]
//        [Description("PARAM5 / local: X coordinate, global: latitude")]
//        //[FieldOffset(16)]
//        public float x;

//        /// <summary>PARAM6 / local: Y coordinate, global: longitude   </summary>
//        [Units("")]
//        [Description("PARAM6 / local: Y coordinate, global: longitude")]
//        //[FieldOffset(20)]
//        public float y;

//        /// <summary>PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).   </summary>
//        [Units("")]
//        [Description("PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).")]
//        //[FieldOffset(24)]
//        public float z;

//        /// <summary>Sequence   </summary>
//        [Units("")]
//        [Description("Sequence")]
//        //[FieldOffset(28)]
//        public ushort seq;

//        /// <summary>The scheduled action for the waypoint. MAV_CMD  </summary>
//        [Units("")]
//        [Description("The scheduled action for the waypoint.")]
//        //[FieldOffset(30)]
//        public  /*MAV_CMD*/ushort command;

//        /// <summary>System ID   </summary>
//        [Units("")]
//        [Description("System ID")]
//        //[FieldOffset(32)]
//        public byte target_system;

//        /// <summary>Component ID   </summary>
//        [Units("")]
//        [Description("Component ID")]
//        //[FieldOffset(33)]
//        public byte target_component;

//        /// <summary>The coordinate system of the waypoint. MAV_FRAME  </summary>
//        [Units("")]
//        [Description("The coordinate system of the waypoint.")]
//        //[FieldOffset(34)]
//        public  /*MAV_FRAME*/byte frame;

//        /// <summary>false:0, true:1   </summary>
//        [Units("")]
//        [Description("false:0, true:1")]
//        //[FieldOffset(35)]
//        public byte current;

//        /// <summary>Autocontinue to next waypoint   </summary>
//        [Units("")]
//        [Description("Autocontinue to next waypoint")]
//        //[FieldOffset(36)]
//        public byte autocontinue;

//        /// <summary>Mission type. MAV_MISSION_TYPE  </summary>
//        [Units("")]
//        [Description("Mission type.")]
//        //[FieldOffset(37)]
//        public  /*MAV_MISSION_TYPE*/byte mission_type;
//    };

//    public enum MAV_MISSION_TYPE : byte {
//        ///<summary> Items are mission commands for main mission. | </summary>
//        [Description("Items are mission commands for main mission.")]
//        MISSION = 0,
//        ///<summary> Specifies GeoFence area(s). Items are MAV_CMD_NAV_FENCE_ GeoFence items. | </summary>
//        [Description("Specifies GeoFence area(s). Items are MAV_CMD_NAV_FENCE_ GeoFence items.")]
//        FENCE = 1,
//        ///<summary> Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_NAV_RALLY_POINT rally point items. | </summary>
//        [Description("Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_NAV_RALLY_POINT rally point items.")]
//        RALLY = 2,
//        ///<summary> Only used in MISSION_CLEAR_ALL to clear all mission types. | </summary>
//        [Description("Only used in MISSION_CLEAR_ALL to clear all mission types.")]
//        ALL = 255,

//    };

//    /// extensions_start 3
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 5)]
//    ///<summary> This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of waypoints. </summary>
//    public struct mavlink_mission_count_t {
//        public mavlink_mission_count_t(ushort count, byte target_system, byte target_component,/*MAV_MISSION_TYPE*/byte mission_type) {
//            this.count = count;
//            this.target_system = target_system;
//            this.target_component = target_component;
//            this.mission_type = mission_type;

//        }

//        /// <summary>Number of mission items in the sequence   </summary>
//        [Units("")]
//        [Description("Number of mission items in the sequence")]
//        //[FieldOffset(0)]
//        public ushort count;

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

//    /// extensions_start 2
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 13)]
//    ///<summary> Settings of a camera. Can be requested with a MAV_CMD_REQUEST_MESSAGE command. </summary>
//    public struct mavlink_camera_settings_t {
//        public mavlink_camera_settings_t(uint time_boot_ms,/*CAMERA_MODE*/byte mode_id, float zoomLevel, float focusLevel) {
//            this.time_boot_ms = time_boot_ms;
//            this.mode_id = mode_id;
//            this.zoomLevel = zoomLevel;
//            this.focusLevel = focusLevel;

//        }

//        /// <summary>Timestamp (time since system boot).  [ms] </summary>
//        [Units("[ms]")]
//        [Description("Timestamp (time since system boot).")]
//        //[FieldOffset(0)]
//        public uint time_boot_ms;

//        /// <summary>Camera mode CAMERA_MODE  </summary>
//        [Units("")]
//        [Description("Camera mode")]
//        //[FieldOffset(4)]
//        public  /*CAMERA_MODE*/byte mode_id;

//        /// <summary>Current zoom level (0.0 to 100.0, NaN if not known)   </summary>
//        [Units("")]
//        [Description("Current zoom level (0.0 to 100.0, NaN if not known)")]
//        //[FieldOffset(5)]
//        public float zoomLevel;

//        /// <summary>Current focus level (0.0 to 100.0, NaN if not known)   </summary>
//        [Units("")]
//        [Description("Current focus level (0.0 to 100.0, NaN if not known)")]
//        //[FieldOffset(9)]
//        public float focusLevel;
//    };

//    /// extensions_start 0
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 19)]
//    ///<summary> Information about the status of a video stream. It may be requested using MAV_CMD_REQUEST_MESSAGE. </summary>
//    public struct mavlink_video_stream_status_t {
//        public mavlink_video_stream_status_t(float framerate, uint bitrate,/*VIDEO_STREAM_STATUS_FLAGS*/ushort flags, ushort resolution_h, ushort resolution_v, ushort rotation, ushort hfov, byte stream_id) {
//            this.framerate = framerate;
//            this.bitrate = bitrate;
//            this.flags = flags;
//            this.resolution_h = resolution_h;
//            this.resolution_v = resolution_v;
//            this.rotation = rotation;
//            this.hfov = hfov;
//            this.stream_id = stream_id;

//        }

//        /// <summary>Frame rate  [Hz] </summary>
//        [Units("[Hz]")]
//        [Description("Frame rate")]
//        //[FieldOffset(0)]
//        public float framerate;

//        /// <summary>Bit rate  [bits/s] </summary>
//        [Units("[bits/s]")]
//        [Description("Bit rate")]
//        //[FieldOffset(4)]
//        public uint bitrate;

//        /// <summary>Bitmap of stream status flags VIDEO_STREAM_STATUS_FLAGS  </summary>
//        [Units("")]
//        [Description("Bitmap of stream status flags")]
//        //[FieldOffset(8)]
//        public  /*VIDEO_STREAM_STATUS_FLAGS*/ushort flags;

//        /// <summary>Horizontal resolution  [pix] </summary>
//        [Units("[pix]")]
//        [Description("Horizontal resolution")]
//        //[FieldOffset(10)]
//        public ushort resolution_h;

//        /// <summary>Vertical resolution  [pix] </summary>
//        [Units("[pix]")]
//        [Description("Vertical resolution")]
//        //[FieldOffset(12)]
//        public ushort resolution_v;

//        /// <summary>Video image rotation clockwise  [deg] </summary>
//        [Units("[deg]")]
//        [Description("Video image rotation clockwise")]
//        //[FieldOffset(14)]
//        public ushort rotation;

//        /// <summary>Horizontal Field of view  [deg] </summary>
//        [Units("[deg]")]
//        [Description("Horizontal Field of view")]
//        //[FieldOffset(16)]
//        public ushort hfov;

//        /// <summary>Video Stream ID (1 for first, 2 for second, etc.)   </summary>
//        [Units("")]
//        [Description("Video Stream ID (1 for first, 2 for second, etc.)")]
//        //[FieldOffset(18)]
//        public byte stream_id;
//    };

//    /// extensions_start 6
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 22)]
//    ///<summary> Information about the status of a capture. Can be requested with a MAV_CMD_REQUEST_MESSAGE command. </summary>
//    public struct mavlink_camera_capture_status_t {
//        public mavlink_camera_capture_status_t(uint time_boot_ms, float image_interval, uint recording_time_ms, float available_capacity, byte image_status, byte video_status, int image_count) {
//            this.time_boot_ms = time_boot_ms;
//            this.image_interval = image_interval;
//            this.recording_time_ms = recording_time_ms;
//            this.available_capacity = available_capacity;
//            this.image_status = image_status;
//            this.video_status = video_status;
//            this.image_count = image_count;

//        }

//        /// <summary>Timestamp (time since system boot).  [ms] </summary>
//        [Units("[ms]")]
//        [Description("Timestamp (time since system boot).")]
//        //[FieldOffset(0)]
//        public uint time_boot_ms;

//        /// <summary>Image capture interval  [s] </summary>
//        [Units("[s]")]
//        [Description("Image capture interval")]
//        //[FieldOffset(4)]
//        public float image_interval;

//        /// <summary>Time since recording started  [ms] </summary>
//        [Units("[ms]")]
//        [Description("Time since recording started")]
//        //[FieldOffset(8)]
//        public uint recording_time_ms;

//        /// <summary>Available storage capacity.  [MiB] </summary>
//        [Units("[MiB]")]
//        [Description("Available storage capacity.")]
//        //[FieldOffset(12)]
//        public float available_capacity;

//        /// <summary>Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)   </summary>
//        [Units("")]
//        [Description("Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)")]
//        //[FieldOffset(16)]
//        public byte image_status;

//        /// <summary>Current status of video capturing (0: idle, 1: capture in progress)   </summary>
//        [Units("")]
//        [Description("Current status of video capturing (0: idle, 1: capture in progress)")]
//        //[FieldOffset(17)]
//        public byte video_status;

//        /// <summary>Total number of images captured ('forever', or until reset using MAV_CMD_STORAGE_FORMAT).   </summary>
//        [Units("")]
//        [Description("Total number of images captured ('forever', or until reset using MAV_CMD_STORAGE_FORMAT).")]
//        //[FieldOffset(18)]
//        public int image_count;
//    };

//    /// extensions_start 0
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 255)]
//    ///<summary> Information about a captured image. This is emitted every time a message is captured. It may be re-requested using MAV_CMD_REQUEST_MESSAGE, using param2 to indicate the sequence number for the missing image. </summary>
//    public struct mavlink_camera_image_captured_t {
//        public mavlink_camera_image_captured_t(ulong time_utc, uint time_boot_ms, int lat, int lon, int alt, int relative_alt, float[] q, int image_index, byte camera_id, sbyte capture_result, byte[] file_url) {
//            this.time_utc = time_utc;
//            this.time_boot_ms = time_boot_ms;
//            this.lat = lat;
//            this.lon = lon;
//            this.alt = alt;
//            this.relative_alt = relative_alt;
//            this.q = q;
//            this.image_index = image_index;
//            this.camera_id = camera_id;
//            this.capture_result = capture_result;
//            this.file_url = file_url;

//        }

//        /// <summary>Timestamp (time since UNIX epoch) in UTC. 0 for unknown.  [us] </summary>
//        [Units("[us]")]
//        [Description("Timestamp (time since UNIX epoch) in UTC. 0 for unknown.")]
//        //[FieldOffset(0)]
//        public ulong time_utc;

//        /// <summary>Timestamp (time since system boot).  [ms] </summary>
//        [Units("[ms]")]
//        [Description("Timestamp (time since system boot).")]
//        //[FieldOffset(8)]
//        public uint time_boot_ms;

//        /// <summary>Latitude where image was taken  [degE7] </summary>
//        [Units("[degE7]")]
//        [Description("Latitude where image was taken")]
//        //[FieldOffset(12)]
//        public int lat;

//        /// <summary>Longitude where capture was taken  [degE7] </summary>
//        [Units("[degE7]")]
//        [Description("Longitude where capture was taken")]
//        //[FieldOffset(16)]
//        public int lon;

//        /// <summary>Altitude (MSL) where image was taken  [mm] </summary>
//        [Units("[mm]")]
//        [Description("Altitude (MSL) where image was taken")]
//        //[FieldOffset(20)]
//        public int alt;

//        /// <summary>Altitude above ground  [mm] </summary>
//        [Units("[mm]")]
//        [Description("Altitude above ground")]
//        //[FieldOffset(24)]
//        public int relative_alt;

//        /// <summary>Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)   </summary>
//        [Units("")]
//        [Description("Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)")]
//        //[FieldOffset(28)]
//        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
//        public float[] q;

//        /// <summary>Zero based index of this image (i.e. a new image will have index CAMERA_CAPTURE_STATUS.image count -1)   </summary>
//        [Units("")]
//        [Description("Zero based index of this image (i.e. a new image will have index CAMERA_CAPTURE_STATUS.image count -1)")]
//        //[FieldOffset(44)]
//        public int image_index;

//        /// <summary>Deprecated/unused. Component IDs are used to differentiate multiple cameras.   </summary>
//        [Units("")]
//        [Description("Deprecated/unused. Component IDs are used to differentiate multiple cameras.")]
//        //[FieldOffset(48)]
//        public byte camera_id;

//        /// <summary>Boolean indicating success (1) or failure (0) while capturing this image.   </summary>
//        [Units("")]
//        [Description("Boolean indicating success (1) or failure (0) while capturing this image.")]
//        //[FieldOffset(49)]
//        public sbyte capture_result;

//        /// <summary>URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.   </summary>
//        [Units("")]
//        [Description("URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.")]
//        //[FieldOffset(50)]
//        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 205)]
//        public byte[] file_url;
//    };

//    /// extensions_start 0
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 235)]
//    ///<summary> Information about a camera. Can be requested with a MAV_CMD_REQUEST_MESSAGE command. </summary>
//    public struct mavlink_camera_information_t {
//        public mavlink_camera_information_t(uint time_boot_ms, uint firmware_version, float focal_length, float sensor_size_h, float sensor_size_v,/*CAMERA_CAP_FLAGS*/uint flags, ushort resolution_h, ushort resolution_v, ushort cam_definition_version, byte[] vendor_name, byte[] model_name, byte lens_id, byte[] cam_definition_uri) {
//            this.time_boot_ms = time_boot_ms;
//            this.firmware_version = firmware_version;
//            this.focal_length = focal_length;
//            this.sensor_size_h = sensor_size_h;
//            this.sensor_size_v = sensor_size_v;
//            this.flags = flags;
//            this.resolution_h = resolution_h;
//            this.resolution_v = resolution_v;
//            this.cam_definition_version = cam_definition_version;
//            this.vendor_name = vendor_name;
//            this.model_name = model_name;
//            this.lens_id = lens_id;
//            this.cam_definition_uri = cam_definition_uri;

//        }

//        /// <summary>Timestamp (time since system boot).  [ms] </summary>
//        [Units("[ms]")]
//        [Description("Timestamp (time since system boot).")]
//        //[FieldOffset(0)]
//        public uint time_boot_ms;

//        /// <summary>Version of the camera firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff)   </summary>
//        [Units("")]
//        [Description("Version of the camera firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff)")]
//        //[FieldOffset(4)]
//        public uint firmware_version;

//        /// <summary>Focal length  [mm] </summary>
//        [Units("[mm]")]
//        [Description("Focal length")]
//        //[FieldOffset(8)]
//        public float focal_length;

//        /// <summary>Image sensor size horizontal  [mm] </summary>
//        [Units("[mm]")]
//        [Description("Image sensor size horizontal")]
//        //[FieldOffset(12)]
//        public float sensor_size_h;

//        /// <summary>Image sensor size vertical  [mm] </summary>
//        [Units("[mm]")]
//        [Description("Image sensor size vertical")]
//        //[FieldOffset(16)]
//        public float sensor_size_v;

//        /// <summary>Bitmap of camera capability flags. CAMERA_CAP_FLAGS  bitmask</summary>
//        [Units("")]
//        [Description("Bitmap of camera capability flags.")]
//        //[FieldOffset(20)]
//        public  /*CAMERA_CAP_FLAGS*/uint flags;

//        /// <summary>Horizontal image resolution  [pix] </summary>
//        [Units("[pix]")]
//        [Description("Horizontal image resolution")]
//        //[FieldOffset(24)]
//        public ushort resolution_h;

//        /// <summary>Vertical image resolution  [pix] </summary>
//        [Units("[pix]")]
//        [Description("Vertical image resolution")]
//        //[FieldOffset(26)]
//        public ushort resolution_v;

//        /// <summary>Camera definition version (iteration)   </summary>
//        [Units("")]
//        [Description("Camera definition version (iteration)")]
//        //[FieldOffset(28)]
//        public ushort cam_definition_version;

//        /// <summary>Name of the camera vendor   </summary>
//        [Units("")]
//        [Description("Name of the camera vendor")]
//        //[FieldOffset(30)]
//        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
//        public byte[] vendor_name;

//        /// <summary>Name of the camera model   </summary>
//        [Units("")]
//        [Description("Name of the camera model")]
//        //[FieldOffset(62)]
//        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
//        public byte[] model_name;

//        /// <summary>Reserved for a lens ID   </summary>
//        [Units("")]
//        [Description("Reserved for a lens ID")]
//        //[FieldOffset(94)]
//        public byte lens_id;

//        /// <summary>Camera definition URI (if any, otherwise only basic functions will be available). HTTP- (http://) and MAVLink FTP- (mavlinkftp://) formatted URIs are allowed (and both must be supported by any GCS that implements the Camera Protocol).   </summary>
//        [Units("")]
//        [Description("Camera definition URI (if any, otherwise only basic functions will be available). HTTP- (http://) and MAVLink FTP- (mavlinkftp://) formatted URIs are allowed (and both must be supported by any GCS that implements the Camera Protocol).")]
//        //[FieldOffset(95)]
//        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 140)]
//        public byte[] cam_definition_uri;
//    };

//    /// extensions_start 0
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 213)]
//    ///<summary> Information about video stream. It may be requested using MAV_CMD_REQUEST_MESSAGE, where param2 indicates the video stream id: 0 for all streams, 1 for first, 2 for second, etc. </summary>
//    public struct mavlink_video_stream_information_t {
//        public mavlink_video_stream_information_t(float framerate, uint bitrate,/*VIDEO_STREAM_STATUS_FLAGS*/ushort flags, ushort resolution_h, ushort resolution_v, ushort rotation, ushort hfov, byte stream_id, byte count,/*VIDEO_STREAM_TYPE*/byte type, byte[] name, byte[] uri) {
//            this.framerate = framerate;
//            this.bitrate = bitrate;
//            this.flags = flags;
//            this.resolution_h = resolution_h;
//            this.resolution_v = resolution_v;
//            this.rotation = rotation;
//            this.hfov = hfov;
//            this.stream_id = stream_id;
//            this.count = count;
//            this.type = type;
//            this.name = name;
//            this.uri = uri;

//        }

//        /// <summary>Frame rate.  [Hz] </summary>
//        [Units("[Hz]")]
//        [Description("Frame rate.")]
//        //[FieldOffset(0)]
//        public float framerate;

//        /// <summary>Bit rate.  [bits/s] </summary>
//        [Units("[bits/s]")]
//        [Description("Bit rate.")]
//        //[FieldOffset(4)]
//        public uint bitrate;

//        /// <summary>Bitmap of stream status flags. VIDEO_STREAM_STATUS_FLAGS  </summary>
//        [Units("")]
//        [Description("Bitmap of stream status flags.")]
//        //[FieldOffset(8)]
//        public  /*VIDEO_STREAM_STATUS_FLAGS*/ushort flags;

//        /// <summary>Horizontal resolution.  [pix] </summary>
//        [Units("[pix]")]
//        [Description("Horizontal resolution.")]
//        //[FieldOffset(10)]
//        public ushort resolution_h;

//        /// <summary>Vertical resolution.  [pix] </summary>
//        [Units("[pix]")]
//        [Description("Vertical resolution.")]
//        //[FieldOffset(12)]
//        public ushort resolution_v;

//        /// <summary>Video image rotation clockwise.  [deg] </summary>
//        [Units("[deg]")]
//        [Description("Video image rotation clockwise.")]
//        //[FieldOffset(14)]
//        public ushort rotation;

//        /// <summary>Horizontal Field of view.  [deg] </summary>
//        [Units("[deg]")]
//        [Description("Horizontal Field of view.")]
//        //[FieldOffset(16)]
//        public ushort hfov;

//        /// <summary>Video Stream ID (1 for first, 2 for second, etc.)   </summary>
//        [Units("")]
//        [Description("Video Stream ID (1 for first, 2 for second, etc.)")]
//        //[FieldOffset(18)]
//        public byte stream_id;

//        /// <summary>Number of streams available.   </summary>
//        [Units("")]
//        [Description("Number of streams available.")]
//        //[FieldOffset(19)]
//        public byte count;

//        /// <summary>Type of stream. VIDEO_STREAM_TYPE  </summary>
//        [Units("")]
//        [Description("Type of stream.")]
//        //[FieldOffset(20)]
//        public  /*VIDEO_STREAM_TYPE*/byte type;

//        /// <summary>Stream name.   </summary>
//        [Units("")]
//        [Description("Stream name.")]
//        //[FieldOffset(21)]
//        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
//        public byte[] name;

//        /// <summary>Video stream URI (TCP or RTSP URI ground station should connect to) or port number (UDP port ground station should listen to).   </summary>
//        [Units("")]
//        [Description("Video stream URI (TCP or RTSP URI ground station should connect to) or port number (UDP port ground station should listen to).")]
//        //[FieldOffset(53)]
//        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 160)]
//        public byte[] uri;
//    };

//    ///<summary> Camera capability flags (Bitmap) </summary>
//    [Flags]
//    public enum CAMERA_CAP_FLAGS : uint {
//        ///<summary> Camera is able to record video | </summary>
//        [Description("Camera is able to record video")]
//        CAPTURE_VIDEO = 1,
//        ///<summary> Camera is able to capture images | </summary>
//        [Description("Camera is able to capture images")]
//        CAPTURE_IMAGE = 2,
//        ///<summary> Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE) | </summary>
//        [Description("Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)")]
//        HAS_MODES = 4,
//        ///<summary> Camera can capture images while in video mode | </summary>
//        [Description("Camera can capture images while in video mode")]
//        CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = 8,
//        ///<summary> Camera can capture videos while in Photo/Image mode | </summary>
//        [Description("Camera can capture videos while in Photo/Image mode")]
//        CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = 16,
//        ///<summary> Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE) | </summary>
//        [Description("Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)")]
//        HAS_IMAGE_SURVEY_MODE = 32,
//        ///<summary> Camera has basic zoom control (MAV_CMD_SET_CAMERA_ZOOM) | </summary>
//        [Description("Camera has basic zoom control (MAV_CMD_SET_CAMERA_ZOOM)")]
//        HAS_BASIC_ZOOM = 64,
//        ///<summary> Camera has basic focus control (MAV_CMD_SET_CAMERA_FOCUS) | </summary>
//        [Description("Camera has basic focus control (MAV_CMD_SET_CAMERA_FOCUS)")]
//        HAS_BASIC_FOCUS = 128,
//        ///<summary> Camera has video streaming capabilities (request VIDEO_STREAM_INFORMATION with MAV_CMD_REQUEST_MESSAGE for video streaming info) | </summary>
//        [Description("Camera has video streaming capabilities (request VIDEO_STREAM_INFORMATION with MAV_CMD_REQUEST_MESSAGE for video streaming info)")]
//        HAS_VIDEO_STREAM = 256,
//        ///<summary> Camera supports tracking of a point on the camera view. | </summary>
//        [Description("Camera supports tracking of a point on the camera view.")]
//        HAS_TRACKING_POINT = 512,
//        ///<summary> Camera supports tracking of a selection rectangle on the camera view. | </summary>
//        [Description("Camera supports tracking of a selection rectangle on the camera view.")]
//        HAS_TRACKING_RECTANGLE = 1024,
//        ///<summary> Camera supports tracking geo status (CAMERA_TRACKING_GEO_STATUS). | </summary>
//        [Description("Camera supports tracking geo status (CAMERA_TRACKING_GEO_STATUS).")]
//        HAS_TRACKING_GEO_STATUS = 2048,

//    };

//    /// extensions_start 2
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 10)]
//    ///<summary> Report status of a command. Includes feedback whether the command was executed. The command microservice is documented at https://mavlink.io/en/services/command.html </summary>
//    public struct mavlink_command_ack_t {
//        public mavlink_command_ack_t(/*MAV_CMD*/ushort command,/*MAV_RESULT*/byte result, byte progress, int result_param2, byte target_system, byte target_component) {
//            this.command = command;
//            this.result = result;
//            this.progress = progress;
//            this.result_param2 = result_param2;
//            this.target_system = target_system;
//            this.target_component = target_component;

//        }

//        /// <summary>Command ID (of acknowledged command). MAV_CMD  </summary>
//        [Units("")]
//        [Description("Command ID (of acknowledged command).")]
//        //[FieldOffset(0)]
//        public  /*MAV_CMD*/ushort command;

//        /// <summary>Result of command. MAV_RESULT  </summary>
//        [Units("")]
//        [Description("Result of command.")]
//        //[FieldOffset(2)]
//        public  /*MAV_RESULT*/byte result;

//        /// <summary>Also used as result_param1, it can be set with a enum containing the errors reasons of why the command was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS.   </summary>
//        [Units("")]
//        [Description("Also used as result_param1, it can be set with a enum containing the errors reasons of why the command was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS.")]
//        //[FieldOffset(3)]
//        public byte progress;

//        /// <summary>Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be denied.   </summary>
//        [Units("")]
//        [Description("Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be denied.")]
//        //[FieldOffset(4)]
//        public int result_param2;

//        /// <summary>System which requested the command to be executed   </summary>
//        [Units("")]
//        [Description("System which requested the command to be executed")]
//        //[FieldOffset(8)]
//        public byte target_system;

//        /// <summary>Component which requested the command to be executed   </summary>
//        [Units("")]
//        [Description("Component which requested the command to be executed")]
//        //[FieldOffset(9)]
//        public byte target_component;
//    };

//    ///<summary> Result from a MAVLink command (MAV_CMD) </summary>
//    public enum MAV_RESULT : byte {
//        ///<summary> Command is valid (is supported and has valid parameters), and was executed. | </summary>
//        [Description("Command is valid (is supported and has valid parameters), and was executed.")]
//        ACCEPTED = 0,
//        ///<summary> Command is valid, but cannot be executed at this time. This is used to indicate a problem that should be fixed just by waiting (e.g. a state machine is busy, can't arm because have not got GPS lock, etc.). Retrying later should work. | </summary>
//        [Description("Command is valid, but cannot be executed at this time. This is used to indicate a problem that should be fixed just by waiting (e.g. a state machine is busy, can't arm because have not got GPS lock, etc.). Retrying later should work.")]
//        TEMPORARILY_REJECTED = 1,
//        ///<summary> Command is invalid (is supported but has invalid parameters). Retrying same command and parameters will not work. | </summary>
//        [Description("Command is invalid (is supported but has invalid parameters). Retrying same command and parameters will not work.")]
//        DENIED = 2,
//        ///<summary> Command is not supported (unknown). | </summary>
//        [Description("Command is not supported (unknown).")]
//        UNSUPPORTED = 3,
//        ///<summary> Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected problem, i.e. any problem that must be fixed before the command can succeed/be retried. For example, attempting to write a file when out of memory, attempting to arm when sensors are not calibrated, etc. | </summary>
//        [Description("Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected problem, i.e. any problem that must be fixed before the command can succeed/be retried. For example, attempting to write a file when out of memory, attempting to arm when sensors are not calibrated, etc.")]
//        FAILED = 4,
//        ///<summary> Command is valid and is being executed. This will be followed by further progress updates, i.e. the component may send further COMMAND_ACK messages with result MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation), and must terminate by sending a COMMAND_ACK message with final result of the operation. The COMMAND_ACK.progress field can be used to indicate the progress of the operation. There is no need for the sender to retry the command, but if done during execution, the component will return MAV_RESULT_IN_PROGRESS with an updated progress. | </summary>
//        [Description("Command is valid and is being executed. This will be followed by further progress updates, i.e. the component may send further COMMAND_ACK messages with result MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation), and must terminate by sending a COMMAND_ACK message with final result of the operation. The COMMAND_ACK.progress field can be used to indicate the progress of the operation. There is no need for the sender to retry the command, but if done during execution, the component will return MAV_RESULT_IN_PROGRESS with an updated progress.")]
//        IN_PROGRESS = 5,

//    };

//    /// extensions_start 0
//    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 33)]
//    ///<summary> Send a command with up to seven parameters to the MAV. The command microservice is documented at https://mavlink.io/en/services/command.html </summary>
//    public struct mavlink_command_long_t {
//        public mavlink_command_long_t(float param1, float param2, float param3, float param4, float param5, float param6, float param7,/*MAV_CMD*/ushort command, byte target_system, byte target_component, byte confirmation) {
//            this.param1 = param1;
//            this.param2 = param2;
//            this.param3 = param3;
//            this.param4 = param4;
//            this.param5 = param5;
//            this.param6 = param6;
//            this.param7 = param7;
//            this.command = command;
//            this.target_system = target_system;
//            this.target_component = target_component;
//            this.confirmation = confirmation;

//        }

//        /// <summary>Parameter 1 (for the specific command).   </summary>
//        [Units("")]
//        [Description("Parameter 1 (for the specific command).")]
//        //[FieldOffset(0)]
//        public float param1;

//        /// <summary>Parameter 2 (for the specific command).   </summary>
//        [Units("")]
//        [Description("Parameter 2 (for the specific command).")]
//        //[FieldOffset(4)]
//        public float param2;

//        /// <summary>Parameter 3 (for the specific command).   </summary>
//        [Units("")]
//        [Description("Parameter 3 (for the specific command).")]
//        //[FieldOffset(8)]
//        public float param3;

//        /// <summary>Parameter 4 (for the specific command).   </summary>
//        [Units("")]
//        [Description("Parameter 4 (for the specific command).")]
//        //[FieldOffset(12)]
//        public float param4;

//        /// <summary>Parameter 5 (for the specific command).   </summary>
//        [Units("")]
//        [Description("Parameter 5 (for the specific command).")]
//        //[FieldOffset(16)]
//        public float param5;

//        /// <summary>Parameter 6 (for the specific command).   </summary>
//        [Units("")]
//        [Description("Parameter 6 (for the specific command).")]
//        //[FieldOffset(20)]
//        public float param6;

//        /// <summary>Parameter 7 (for the specific command).   </summary>
//        [Units("")]
//        [Description("Parameter 7 (for the specific command).")]
//        //[FieldOffset(24)]
//        public float param7;

//        /// <summary>Command ID (of command to send). MAV_CMD  </summary>
//        [Units("")]
//        [Description("Command ID (of command to send).")]
//        //[FieldOffset(28)]
//        public  /*MAV_CMD*/ushort command;

//        /// <summary>System which should execute the command   </summary>
//        [Units("")]
//        [Description("System which should execute the command")]
//        //[FieldOffset(30)]
//        public byte target_system;

//        /// <summary>Component which should execute the command, 0 for all components   </summary>
//        [Units("")]
//        [Description("Component which should execute the command, 0 for all components")]
//        //[FieldOffset(31)]
//        public byte target_component;

//        /// <summary>0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)   </summary>
//        [Units("")]
//        [Description("0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)")]
//        //[FieldOffset(32)]
//        public byte confirmation;
//    };




//    ///<summary> Flags for gimbal device (lower level) operation. </summary>
//    [Flags]
//    public enum GIMBAL_DEVICE_FLAGS : ushort {
//        ///<summary> Set to retracted safe position (no stabilization), takes presedence over all other flags. | </summary>
//        [Description("Set to retracted safe position (no stabilization), takes presedence over all other flags.")]
//        RETRACT = 1,
//        ///<summary> Set to neutral/default position, taking precedence over all other flags except RETRACT. Neutral is commonly forward-facing and horizontal (pitch=yaw=0) but may be any orientation. | </summary>
//        [Description("Set to neutral/default position, taking precedence over all other flags except RETRACT. Neutral is commonly forward-facing and horizontal (pitch=yaw=0) but may be any orientation.")]
//        NEUTRAL = 2,
//        ///<summary> Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the default with a stabilizing gimbal. | </summary>
//        [Description("Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the default with a stabilizing gimbal.")]
//        ROLL_LOCK = 4,
//        ///<summary> Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally the default. | </summary>
//        [Description("Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally the default.")]
//        PITCH_LOCK = 8,
//        ///<summary> Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the quaternion is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not set, the quaternion frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle). | </summary>
//        [Description("Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the quaternion is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not set, the quaternion frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle).")]
//        YAW_LOCK = 16,

//    };

//    public struct mavlink_gimbal_device_set_attitude_t
//    {
//        public mavlink_gimbal_device_set_attitude_t(float[] q,float angular_velocity_x,float angular_velocity_y,float angular_velocity_z,/*GIMBAL_DEVICE_FLAGS*/ushort flags,byte target_system,byte target_component) 
//        {
//            this.q = q;
//            this.angular_velocity_x = angular_velocity_x;
//            this.angular_velocity_y = angular_velocity_y;
//            this.angular_velocity_z = angular_velocity_z;
//            this.flags = flags;
//            this.target_system = target_system;
//            this.target_component = target_component;
            
//        }

//        /// <summary>Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set, set all fields to NaN if only angular velocity should be used)   </summary>
//        [Units("")]
//        [Description("Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set, set all fields to NaN if only angular velocity should be used)")]
//        //[FieldOffset(0)]
//        [System.Runtime.InteropServices.MarshalAs(UnmanagedType.ByValArray,SizeConst=4)]
//		public float[] q;

//        /// <summary>X component of angular velocity, positive is rolling to the right, NaN to be ignored.  [rad/s] </summary>
//        [Units("[rad/s]")]
//        [Description("X component of angular velocity, positive is rolling to the right, NaN to be ignored.")]
//        //[FieldOffset(16)]
//        public  float angular_velocity_x;

//        /// <summary>Y component of angular velocity, positive is pitching up, NaN to be ignored.  [rad/s] </summary>
//        [Units("[rad/s]")]
//        [Description("Y component of angular velocity, positive is pitching up, NaN to be ignored.")]
//        //[FieldOffset(20)]
//        public  float angular_velocity_y;

//        /// <summary>Z component of angular velocity, positive is yawing to the right, NaN to be ignored.  [rad/s] </summary>
//        [Units("[rad/s]")]
//        [Description("Z component of angular velocity, positive is yawing to the right, NaN to be ignored.")]
//        //[FieldOffset(24)]
//        public  float angular_velocity_z;

//        /// <summary>Low level gimbal flags. GIMBAL_DEVICE_FLAGS  </summary>
//        [Units("")]
//        [Description("Low level gimbal flags.")]
//        //[FieldOffset(28)]
//        public  /*GIMBAL_DEVICE_FLAGS*/ushort flags;

//        /// <summary>System ID   </summary>
//        [Units("")]
//        [Description("System ID")]
//        //[FieldOffset(30)]
//        public  byte target_system;

//        /// <summary>Component ID   </summary>
//        [Units("")]
//        [Description("Component ID")]
//        //[FieldOffset(31)]
//        public  byte target_component;
//    };
//    public class MAVLinkMessage {
//        public static readonly MAVLinkMessage Invalid = new MAVLinkMessage();
//        object _locker = new object();

//        private byte[] _buffer;

//        public byte[] buffer {
//            get { return _buffer; }
//            set {
//                _buffer = value;
//                processBuffer(_buffer);
//            }
//        }

//        public DateTime rxtime { get; set; }
//        public byte header { get; internal set; }
//        public byte payloadlength { get; internal set; }

//        public byte incompat_flags { get; internal set; }
//        public byte compat_flags { get; internal set; }

//        public byte seq { get; internal set; }
//        public byte sysid { get; internal set; }
//        public byte compid { get; internal set; }

//        public uint msgid { get; internal set; }


//        public bool ismavlink2 {
//            get {
//                if (buffer != null && buffer.Length > 0)
//                    return (buffer[0] == MAVLINK_STX);

//                return false;
//            }
//        }

//        public string msgtypename {
//            get { return MAVLINK_MESSAGE_INFOS.GetMessageInfo(msgid).name; }
//        }

//        object _data;
//        public object data {
//            get {
//                // lock the entire creation of the packet. to prevent returning a incomplete packet.
//                lock (_locker) {
//                    if (_data != null)
//                        return _data;

//                    var typeinfo = MAVLINK_MESSAGE_INFOS.GetMessageInfo(msgid);

//                    if (typeinfo.type == null)
//                        return null;

//                    _data = Activator.CreateInstance(typeinfo.type);

//                    try {
//                        if (payloadlength == 0)
//                            return _data;
//                        // fill in the data of the object
//                        if (ismavlink2) {
//                           // _data = MavlinkUtil.ByteArrayToStructureGC(buffer, typeinfo.type, MAVLINK_NUM_HEADER_BYTES,
//                             //   payloadlength);
//                            //MavlinkUtil.ByteArrayToStructure(buffer, ref _data, MAVLINK_NUM_HEADER_BYTES, payloadlength);
//                        }
//                        else {
//                            _data = MavlinkUtil.ByteArrayToStructureGC(buffer, typeinfo.type, 6, payloadlength);
//                        }
//                    }
//                    catch (Exception ex) {
//                        // should not happen
//                        if (Debugger.IsAttached)
//                            Debugger.Break();
//                        System.Diagnostics.Debug.WriteLine(ex);
//                    }
//                }

//                return _data;
//            }
//        }

//        public T ToStructure<T>() {
//            return (T)data;
//        }

//        public ushort crc16 { get; internal set; }

//        public byte[] sig { get; internal set; }

//        public byte sigLinkid {
//            get {
//                if (sig != null) {
//                    return sig[0];
//                }

//                return 0;
//            }
//        }

//        public ulong sigTimestamp {
//            get {
//                if (sig != null) {
//                    byte[] temp = new byte[8];
//                    Array.Copy(sig, 1, temp, 0, 6);
//                    return BitConverter.ToUInt64(temp, 0);
//                }

//                return 0;
//            }
//        }

//        public int Length {
//            get {
//                if (buffer == null) return 0;
//                return buffer.Length;
//            }
//        }

//        public MAVLinkMessage() {
//            this.rxtime = DateTime.MinValue;
//        }

//        public MAVLinkMessage(byte[] buffer) : this(buffer, DateTime.UtcNow) {
//        }

//        public MAVLinkMessage(byte[] buffer, DateTime rxTime) {
//            this.buffer = buffer;
//            this.rxtime = rxTime;
//        }

//        internal void processBuffer(byte[] buffer) {
//            _data = null;

//            if (buffer[0] == MAVLINK_STX) {
//                if (buffer.Length < 10) {
//                    return;
//                }
//                header = buffer[0];
//                payloadlength = buffer[1];
//                incompat_flags = buffer[2];
//                compat_flags = buffer[3];
//                seq = buffer[4];
//                sysid = buffer[5];
//                compid = buffer[6];
//                msgid = (uint)((buffer[9] << 16) + (buffer[8] << 8) + buffer[7]);

//                var crc1 = MAVLINK_CORE_HEADER_LEN + payloadlength + 1;
//                var crc2 = MAVLINK_CORE_HEADER_LEN + payloadlength + 2;

//                crc16 = (ushort)((buffer[crc2] << 8) + buffer[crc1]);

//                if ((incompat_flags & MAVLINK_IFLAG_SIGNED) > 0) {
//                    sig = new byte[MAVLINK_SIGNATURE_BLOCK_LEN];
//                    Array.ConstrainedCopy(buffer, buffer.Length - MAVLINK_SIGNATURE_BLOCK_LEN, sig, 0,
//                        MAVLINK_SIGNATURE_BLOCK_LEN);
//                }
//            }
//            else if (buffer[0] == MAVLINK_STX_MAVLINK1) {
//                if (buffer.Length < 6) {
//                    return;
//                }
//                header = buffer[0];
//                payloadlength = buffer[1];
//                seq = buffer[2];
//                sysid = buffer[3];
//                compid = buffer[4];
//                msgid = buffer[5];

//                var crc1 = MAVLINK_CORE_HEADER_MAVLINK1_LEN + payloadlength + 1;
//                var crc2 = MAVLINK_CORE_HEADER_MAVLINK1_LEN + payloadlength + 2;

//                crc16 = (ushort)((buffer[crc2] << 8) + buffer[crc1]);
//            }
//        }

//        public override string ToString() {
//            return String.Format("{5},{4},{0},{1},{2},{3}", sysid, compid, msgid, msgtypename, ismavlink2, rxtime);
//        }
//    }
//}
