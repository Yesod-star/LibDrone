//using System;
//using System.Runtime.InteropServices;
//using static LibDrone.Frames.Text;


//        /// extensions_start 0
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 53)]
//        ///<summary> Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller to command the vehicle (manual controller or other system). </summary>
//        public struct mavlink_set_position_target_local_ned_t {

//            public mavlink_set_position_target_local_ned_t(uint time_boot_ms, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate,/*POSITION_TARGET_TYPEMASK*/ushort type_mask, byte target_system, byte target_component,/*MAV_FRAME*/byte coordinate_frame) {
//                this.time_boot_ms = time_boot_ms;
//                this.x = x;
//                this.y = y;
//                this.z = z;
//                this.vx = vx;
//                this.vy = vy;
//                this.vz = vz;
//                this.afx = afx;
//                this.afy = afy;
//                this.afz = afz;
//                this.yaw = yaw;
//                this.yaw_rate = yaw_rate;
//                this.type_mask = type_mask;
//                this.target_system = target_system;
//                this.target_component = target_component;
//                this.coordinate_frame = coordinate_frame;

//            }

//            /// <summary>Timestamp (time since system boot).  [ms] </summary>
//            [Units("[ms]")]
//            [Description("Timestamp (time since system boot).")]
//            //[FieldOffset(0)]
//            public uint time_boot_ms;

//            /// <summary>X Position in NED frame  [m] </summary>
//            [Units("[m]")]
//            [Description("X Position in NED frame")]
//            //[FieldOffset(4)]
//            public float x;

//            /// <summary>Y Position in NED frame  [m] </summary>
//            [Units("[m]")]
//            [Description("Y Position in NED frame")]
//            //[FieldOffset(8)]
//            public float y;

//            /// <summary>Z Position in NED frame (note, altitude is negative in NED)  [m] </summary>
//            [Units("[m]")]
//            [Description("Z Position in NED frame (note, altitude is negative in NED)")]
//            //[FieldOffset(12)]
//            public float z;

//            /// <summary>X velocity in NED frame  [m/s] </summary>
//            [Units("[m/s]")]
//            [Description("X velocity in NED frame")]
//            //[FieldOffset(16)]
//            public float vx;

//            /// <summary>Y velocity in NED frame  [m/s] </summary>
//            [Units("[m/s]")]
//            [Description("Y velocity in NED frame")]
//            //[FieldOffset(20)]
//            public float vy;

//            /// <summary>Z velocity in NED frame  [m/s] </summary>
//            [Units("[m/s]")]
//            [Description("Z velocity in NED frame")]
//            //[FieldOffset(24)]
//            public float vz;

//            /// <summary>X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N  [m/s/s] </summary>
//            [Units("[m/s/s]")]
//            [Description("X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N")]
//            //[FieldOffset(28)]
//            public float afx;

//            /// <summary>Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N  [m/s/s] </summary>
//            [Units("[m/s/s]")]
//            [Description("Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N")]
//            //[FieldOffset(32)]
//            public float afy;

//            /// <summary>Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N  [m/s/s] </summary>
//            [Units("[m/s/s]")]
//            [Description("Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N")]
//            //[FieldOffset(36)]
//            public float afz;

//            /// <summary>yaw setpoint  [rad] </summary>
//            [Units("[rad]")]
//            [Description("yaw setpoint")]
//            //[FieldOffset(40)]
//            public float yaw;

//            /// <summary>yaw rate setpoint  [rad/s] </summary>
//            [Units("[rad/s]")]
//            [Description("yaw rate setpoint")]
//            //[FieldOffset(44)]
//            public float yaw_rate;

//            /// <summary>Bitmap to indicate which dimensions should be ignored by the vehicle. POSITION_TARGET_TYPEMASK  bitmask</summary>
//            [Units("")]
//            [Description("Bitmap to indicate which dimensions should be ignored by the vehicle.")]
//            //[FieldOffset(48)]
//            public  /*POSITION_TARGET_TYPEMASK*/ushort type_mask;

//            /// <summary>System ID   </summary>
//            [Units("")]
//            [Description("System ID")]
//            //[FieldOffset(50)]
//            public byte target_system;

//            /// <summary>Component ID   </summary>
//            [Units("")]
//            [Description("Component ID")]
//            //[FieldOffset(51)]
//            public byte target_component;

//            /// <summary>Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9 MAV_FRAME  </summary>
//            [Units("")]
//            [Description("Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9")]
//            //[FieldOffset(52)]
//            public  /*MAV_FRAME*/byte coordinate_frame;
//        };

//        /// extensions_start 0
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 51)]
//        ///<summary> Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled this way. </summary>
//        public struct mavlink_position_target_local_ned_t {
//            public mavlink_position_target_local_ned_t(uint time_boot_ms, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate,/*POSITION_TARGET_TYPEMASK*/ushort type_mask,/*MAV_FRAME*/byte coordinate_frame) {
//                this.time_boot_ms = time_boot_ms;
//                this.x = x;
//                this.y = y;
//                this.z = z;
//                this.vx = vx;
//                this.vy = vy;
//                this.vz = vz;
//                this.afx = afx;
//                this.afy = afy;
//                this.afz = afz;
//                this.yaw = yaw;
//                this.yaw_rate = yaw_rate;
//                this.type_mask = type_mask;
//                this.coordinate_frame = coordinate_frame;

//            }

//            /// <summary>Timestamp (time since system boot).  [ms] </summary>
//            [Units("[ms]")]
//            [Description("Timestamp (time since system boot).")]
//            //[FieldOffset(0)]
//            public uint time_boot_ms;

//            /// <summary>X Position in NED frame  [m] </summary>
//            [Units("[m]")]
//            [Description("X Position in NED frame")]
//            //[FieldOffset(4)]
//            public float x;

//            /// <summary>Y Position in NED frame  [m] </summary>
//            [Units("[m]")]
//            [Description("Y Position in NED frame")]
//            //[FieldOffset(8)]
//            public float y;

//            /// <summary>Z Position in NED frame (note, altitude is negative in NED)  [m] </summary>
//            [Units("[m]")]
//            [Description("Z Position in NED frame (note, altitude is negative in NED)")]
//            //[FieldOffset(12)]
//            public float z;

//            /// <summary>X velocity in NED frame  [m/s] </summary>
//            [Units("[m/s]")]
//            [Description("X velocity in NED frame")]
//            //[FieldOffset(16)]
//            public float vx;

//            /// <summary>Y velocity in NED frame  [m/s] </summary>
//            [Units("[m/s]")]
//            [Description("Y velocity in NED frame")]
//            //[FieldOffset(20)]
//            public float vy;

//            /// <summary>Z velocity in NED frame  [m/s] </summary>
//            [Units("[m/s]")]
//            [Description("Z velocity in NED frame")]
//            //[FieldOffset(24)]
//            public float vz;

//            /// <summary>X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N  [m/s/s] </summary>
//            [Units("[m/s/s]")]
//            [Description("X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N")]
//            //[FieldOffset(28)]
//            public float afx;

//            /// <summary>Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N  [m/s/s] </summary>
//            [Units("[m/s/s]")]
//            [Description("Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N")]
//            //[FieldOffset(32)]
//            public float afy;

//            /// <summary>Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N  [m/s/s] </summary>
//            [Units("[m/s/s]")]
//            [Description("Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N")]
//            //[FieldOffset(36)]
//            public float afz;

//            /// <summary>yaw setpoint  [rad] </summary>
//            [Units("[rad]")]
//            [Description("yaw setpoint")]
//            //[FieldOffset(40)]
//            public float yaw;

//            /// <summary>yaw rate setpoint  [rad/s] </summary>
//            [Units("[rad/s]")]
//            [Description("yaw rate setpoint")]
//            //[FieldOffset(44)]
//            public float yaw_rate;

//            /// <summary>Bitmap to indicate which dimensions should be ignored by the vehicle. POSITION_TARGET_TYPEMASK  bitmask</summary>
//            [Units("")]
//            [Description("Bitmap to indicate which dimensions should be ignored by the vehicle.")]
//            //[FieldOffset(48)]
//            public  /*POSITION_TARGET_TYPEMASK*/ushort type_mask;

//            /// <summary>Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9 MAV_FRAME  </summary>
//            [Units("")]
//            [Description("Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9")]
//            //[FieldOffset(50)]
//            public  /*MAV_FRAME*/byte coordinate_frame;
//        };

//        /// extensions_start 0
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 53)]
//        ///<summary> Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84). Used by an external controller to command the vehicle (manual controller or other system). </summary>
//        public struct mavlink_set_position_target_global_int_t {
//            public mavlink_set_position_target_global_int_t(uint time_boot_ms, int lat_int, int lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate,/*POSITION_TARGET_TYPEMASK*/ushort type_mask, byte target_system, byte target_component,/*MAV_FRAME*/byte coordinate_frame) {
//                this.time_boot_ms = time_boot_ms;
//                this.lat_int = lat_int;
//                this.lon_int = lon_int;
//                this.alt = alt;
//                this.vx = vx;
//                this.vy = vy;
//                this.vz = vz;
//                this.afx = afx;
//                this.afy = afy;
//                this.afz = afz;
//                this.yaw = yaw;
//                this.yaw_rate = yaw_rate;
//                this.type_mask = type_mask;
//                this.target_system = target_system;
//                this.target_component = target_component;
//                this.coordinate_frame = coordinate_frame;

//            }

//            /// <summary>Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.  [ms] </summary>
//            [Units("[ms]")]
//            [Description("Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.")]
//            //[FieldOffset(0)]
//            public uint time_boot_ms;

//            /// <summary>X Position in WGS84 frame  [degE7] </summary>
//            [Units("[degE7]")]
//            [Description("X Position in WGS84 frame")]
//            //[FieldOffset(4)]
//            public int lat_int;

//            /// <summary>Y Position in WGS84 frame  [degE7] </summary>
//            [Units("[degE7]")]
//            [Description("Y Position in WGS84 frame")]
//            //[FieldOffset(8)]
//            public int lon_int;

//            /// <summary>Altitude (MSL, Relative to home, or AGL - depending on frame)  [m] </summary>
//            [Units("[m]")]
//            [Description("Altitude (MSL, Relative to home, or AGL - depending on frame)")]
//            //[FieldOffset(12)]
//            public float alt;

//            /// <summary>X velocity in NED frame  [m/s] </summary>
//            [Units("[m/s]")]
//            [Description("X velocity in NED frame")]
//            //[FieldOffset(16)]
//            public float vx;

//            /// <summary>Y velocity in NED frame  [m/s] </summary>
//            [Units("[m/s]")]
//            [Description("Y velocity in NED frame")]
//            //[FieldOffset(20)]
//            public float vy;

//            /// <summary>Z velocity in NED frame  [m/s] </summary>
//            [Units("[m/s]")]
//            [Description("Z velocity in NED frame")]
//            //[FieldOffset(24)]
//            public float vz;

//            /// <summary>X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N  [m/s/s] </summary>
//            [Units("[m/s/s]")]
//            [Description("X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N")]
//            //[FieldOffset(28)]
//            public float afx;

//            /// <summary>Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N  [m/s/s] </summary>
//            [Units("[m/s/s]")]
//            [Description("Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N")]
//            //[FieldOffset(32)]
//            public float afy;

//            /// <summary>Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N  [m/s/s] </summary>
//            [Units("[m/s/s]")]
//            [Description("Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N")]
//            //[FieldOffset(36)]
//            public float afz;

//            /// <summary>yaw setpoint  [rad] </summary>
//            [Units("[rad]")]
//            [Description("yaw setpoint")]
//            //[FieldOffset(40)]
//            public float yaw;

//            /// <summary>yaw rate setpoint  [rad/s] </summary>
//            [Units("[rad/s]")]
//            [Description("yaw rate setpoint")]
//            //[FieldOffset(44)]
//            public float yaw_rate;

//            /// <summary>Bitmap to indicate which dimensions should be ignored by the vehicle. POSITION_TARGET_TYPEMASK  bitmask</summary>
//            [Units("")]
//            [Description("Bitmap to indicate which dimensions should be ignored by the vehicle.")]
//            //[FieldOffset(48)]
//            public  /*POSITION_TARGET_TYPEMASK*/ushort type_mask;

//            /// <summary>System ID   </summary>
//            [Units("")]
//            [Description("System ID")]
//            //[FieldOffset(50)]
//            public byte target_system;

//            /// <summary>Component ID   </summary>
//            [Units("")]
//            [Description("Component ID")]
//            //[FieldOffset(51)]
//            public byte target_component;

//            /// <summary>Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11 MAV_FRAME  </summary>
//            [Units("")]
//            [Description("Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11")]
//            //[FieldOffset(52)]
//            public  /*MAV_FRAME*/byte coordinate_frame;
//        };

//        /// extensions_start 0
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 51)]
//        ///<summary> Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled this way. </summary>
//        public struct mavlink_position_target_global_int_t {
//            public mavlink_position_target_global_int_t(uint time_boot_ms, int lat_int, int lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate,/*POSITION_TARGET_TYPEMASK*/ushort type_mask,/*MAV_FRAME*/byte coordinate_frame) {
//                this.time_boot_ms = time_boot_ms;
//                this.lat_int = lat_int;
//                this.lon_int = lon_int;
//                this.alt = alt;
//                this.vx = vx;
//                this.vy = vy;
//                this.vz = vz;
//                this.afx = afx;
//                this.afy = afy;
//                this.afz = afz;
//                this.yaw = yaw;
//                this.yaw_rate = yaw_rate;
//                this.type_mask = type_mask;
//                this.coordinate_frame = coordinate_frame;

//            }

//            /// <summary>Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.  [ms] </summary>
//            [Units("[ms]")]
//            [Description("Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.")]
//            //[FieldOffset(0)]
//            public uint time_boot_ms;

//            /// <summary>X Position in WGS84 frame  [degE7] </summary>
//            [Units("[degE7]")]
//            [Description("X Position in WGS84 frame")]
//            //[FieldOffset(4)]
//            public int lat_int;

//            /// <summary>Y Position in WGS84 frame  [degE7] </summary>
//            [Units("[degE7]")]
//            [Description("Y Position in WGS84 frame")]
//            //[FieldOffset(8)]
//            public int lon_int;

//            /// <summary>Altitude (MSL, AGL or relative to home altitude, depending on frame)  [m] </summary>
//            [Units("[m]")]
//            [Description("Altitude (MSL, AGL or relative to home altitude, depending on frame)")]
//            //[FieldOffset(12)]
//            public float alt;

//            /// <summary>X velocity in NED frame  [m/s] </summary>
//            [Units("[m/s]")]
//            [Description("X velocity in NED frame")]
//            //[FieldOffset(16)]
//            public float vx;

//            /// <summary>Y velocity in NED frame  [m/s] </summary>
//            [Units("[m/s]")]
//            [Description("Y velocity in NED frame")]
//            //[FieldOffset(20)]
//            public float vy;

//            /// <summary>Z velocity in NED frame  [m/s] </summary>
//            [Units("[m/s]")]
//            [Description("Z velocity in NED frame")]
//            //[FieldOffset(24)]
//            public float vz;

//            /// <summary>X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N  [m/s/s] </summary>
//            [Units("[m/s/s]")]
//            [Description("X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N")]
//            //[FieldOffset(28)]
//            public float afx;

//            /// <summary>Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N  [m/s/s] </summary>
//            [Units("[m/s/s]")]
//            [Description("Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N")]
//            //[FieldOffset(32)]
//            public float afy;

//            /// <summary>Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N  [m/s/s] </summary>
//            [Units("[m/s/s]")]
//            [Description("Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N")]
//            //[FieldOffset(36)]
//            public float afz;

//            /// <summary>yaw setpoint  [rad] </summary>
//            [Units("[rad]")]
//            [Description("yaw setpoint")]
//            //[FieldOffset(40)]
//            public float yaw;

//            /// <summary>yaw rate setpoint  [rad/s] </summary>
//            [Units("[rad/s]")]
//            [Description("yaw rate setpoint")]
//            //[FieldOffset(44)]
//            public float yaw_rate;

//            /// <summary>Bitmap to indicate which dimensions should be ignored by the vehicle. POSITION_TARGET_TYPEMASK  bitmask</summary>
//            [Units("")]
//            [Description("Bitmap to indicate which dimensions should be ignored by the vehicle.")]
//            //[FieldOffset(48)]
//            public  /*POSITION_TARGET_TYPEMASK*/ushort type_mask;

//            /// <summary>Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11 MAV_FRAME  </summary>
//            [Units("")]
//            [Description("Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11")]
//            //[FieldOffset(50)]
//            public  /*MAV_FRAME*/byte coordinate_frame;
//        };

//        /// extensions_start 0
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 9)]
//        ///<summary> The heartbeat message shows that a system or component is present and responding. The type and autopilot fields (along with the message component id), allow the receiving system to treat further messages from this system appropriately (e.g. by laying out the user interface based on the autopilot). This microservice is documented at https://mavlink.io/en/services/heartbeat.html </summary>
//        public struct mavlink_heartbeat_t {
//            public mavlink_heartbeat_t(uint custom_mode,/*MAV_TYPE*/byte type,/*MAV_AUTOPILOT*/byte autopilot,/*MAV_MODE_FLAG*/byte base_mode,/*MAV_STATE*/byte system_status, byte mavlink_version) {
//                this.custom_mode = custom_mode;
//                this.type = type;
//                this.autopilot = autopilot;
//                this.base_mode = base_mode;
//                this.system_status = system_status;
//                this.mavlink_version = mavlink_version;

//            }

//            /// <summary>A bitfield for use for autopilot-specific flags   </summary>
//            [Units("")]
//            [Description("A bitfield for use for autopilot-specific flags")]
//            //[FieldOffset(0)]
//            public uint custom_mode;

//            /// <summary>Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type. MAV_TYPE  </summary>
//            [Units("")]
//            [Description("Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.")]
//            //[FieldOffset(4)]
//            public  /*MAV_TYPE*/byte type;

//            /// <summary>Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers. MAV_AUTOPILOT  </summary>
//            [Units("")]
//            [Description("Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.")]
//            //[FieldOffset(5)]
//            public  /*MAV_AUTOPILOT*/byte autopilot;

//            /// <summary>System mode bitmap. MAV_MODE_FLAG  bitmask</summary>
//            [Units("")]
//            [Description("System mode bitmap.")]
//            //[FieldOffset(6)]
//            public  /*MAV_MODE_FLAG*/byte base_mode;

//            /// <summary>System status flag. MAV_STATE  </summary>
//            [Units("")]
//            [Description("System status flag.")]
//            //[FieldOffset(7)]
//            public  /*MAV_STATE*/byte system_status;

//            /// <summary>MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version   </summary>
//            [Units("")]
//            [Description("MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version")]
//            //[FieldOffset(8)]
//            public byte mavlink_version;
//        };
