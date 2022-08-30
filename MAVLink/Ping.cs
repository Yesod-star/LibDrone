//using System;
//using System.Runtime.InteropServices;
//using static LibDrone.Frames.Text;

//        /// extensions_start 0
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 31)]
//        ///<summary> The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows whether the system is currently active or not and if an emergency occurred. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occurred it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout. </summary>
//        public struct mavlink_sys_status_t {
//            public mavlink_sys_status_t(/*MAV_SYS_STATUS_SENSOR*/uint onboard_control_sensors_present,/*MAV_SYS_STATUS_SENSOR*/uint onboard_control_sensors_enabled,/*MAV_SYS_STATUS_SENSOR*/uint onboard_control_sensors_health, ushort load, ushort voltage_battery, short current_battery, ushort drop_rate_comm, ushort errors_comm, ushort errors_count1, ushort errors_count2, ushort errors_count3, ushort errors_count4, sbyte battery_remaining) {
//                this.onboard_control_sensors_present = onboard_control_sensors_present;
//                this.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
//                this.onboard_control_sensors_health = onboard_control_sensors_health;
//                this.load = load;
//                this.voltage_battery = voltage_battery;
//                this.current_battery = current_battery;
//                this.drop_rate_comm = drop_rate_comm;
//                this.errors_comm = errors_comm;
//                this.errors_count1 = errors_count1;
//                this.errors_count2 = errors_count2;
//                this.errors_count3 = errors_count3;
//                this.errors_count4 = errors_count4;
//                this.battery_remaining = battery_remaining;

//            }

//            /// <summary>Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. MAV_SYS_STATUS_SENSOR  bitmask</summary>
//            [Units("")]
//            [Description("Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.")]
//            //[FieldOffset(0)]
//            public  /*MAV_SYS_STATUS_SENSOR*/uint onboard_control_sensors_present;

//            /// <summary>Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. MAV_SYS_STATUS_SENSOR  bitmask</summary>
//            [Units("")]
//            [Description("Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.")]
//            //[FieldOffset(4)]
//            public  /*MAV_SYS_STATUS_SENSOR*/uint onboard_control_sensors_enabled;

//            /// <summary>Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy. MAV_SYS_STATUS_SENSOR  bitmask</summary>
//            [Units("")]
//            [Description("Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy.")]
//            //[FieldOffset(8)]
//            public  /*MAV_SYS_STATUS_SENSOR*/uint onboard_control_sensors_health;

//            /// <summary>Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000  [d%] </summary>
//            [Units("[d%]")]
//            [Description("Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000")]
//            //[FieldOffset(12)]
//            public ushort load;

//            /// <summary>Battery voltage, UINT16_MAX: Voltage not sent by autopilot  [mV] </summary>
//            [Units("[mV]")]
//            [Description("Battery voltage, UINT16_MAX: Voltage not sent by autopilot")]
//            //[FieldOffset(14)]
//            public ushort voltage_battery;

//            /// <summary>Battery current, -1: Current not sent by autopilot  [cA] </summary>
//            [Units("[cA]")]
//            [Description("Battery current, -1: Current not sent by autopilot")]
//            //[FieldOffset(16)]
//            public short current_battery;

//            /// <summary>Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)  [c%] </summary>
//            [Units("[c%]")]
//            [Description("Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)")]
//            //[FieldOffset(18)]
//            public ushort drop_rate_comm;

//            /// <summary>Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)   </summary>
//            [Units("")]
//            [Description("Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)")]
//            //[FieldOffset(20)]
//            public ushort errors_comm;

//            /// <summary>Autopilot-specific errors   </summary>
//            [Units("")]
//            [Description("Autopilot-specific errors")]
//            //[FieldOffset(22)]
//            public ushort errors_count1;

//            /// <summary>Autopilot-specific errors   </summary>
//            [Units("")]
//            [Description("Autopilot-specific errors")]
//            //[FieldOffset(24)]
//            public ushort errors_count2;

//            /// <summary>Autopilot-specific errors   </summary>
//            [Units("")]
//            [Description("Autopilot-specific errors")]
//            //[FieldOffset(26)]
//            public ushort errors_count3;

//            /// <summary>Autopilot-specific errors   </summary>
//            [Units("")]
//            [Description("Autopilot-specific errors")]
//            //[FieldOffset(28)]
//            public ushort errors_count4;

//            /// <summary>Battery energy remaining, -1: Battery remaining energy not sent by autopilot  [%] </summary>
//            [Units("[%]")]
//            [Description("Battery energy remaining, -1: Battery remaining energy not sent by autopilot")]
//            //[FieldOffset(30)]
//            public sbyte battery_remaining;
//        };

//        /// extensions_start 0
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 12)]
//        ///<summary> The system time is the time of the master clock, typically the computer clock of the main onboard computer. </summary>
//        public struct mavlink_system_time_t {
//            public mavlink_system_time_t(ulong time_unix_usec, uint time_boot_ms) {
//                this.time_unix_usec = time_unix_usec;
//                this.time_boot_ms = time_boot_ms;

//            }

//            /// <summary>Timestamp (UNIX epoch time).  [us] </summary>
//            [Units("[us]")]
//            [Description("Timestamp (UNIX epoch time).")]
//            //[FieldOffset(0)]
//            public ulong time_unix_usec;

//            /// <summary>Timestamp (time since system boot).  [ms] </summary>
//            [Units("[ms]")]
//            [Description("Timestamp (time since system boot).")]
//            //[FieldOffset(8)]
//            public uint time_boot_ms;
//        };

//        [Obsolete]
//        /// extensions_start 0
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 14)]
//        ///<summary> A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections. The ping microservice is documented at https://mavlink.io/en/services/ping.html </summary>
//        public struct mavlink_ping_t {
//            public mavlink_ping_t(ulong time_usec, uint seq, byte target_system, byte target_component) {
//                this.time_usec = time_usec;
//                this.seq = seq;
//                this.target_system = target_system;
//                this.target_component = target_component;

//            }

//            /// <summary>Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.  [us] </summary>
//            [Units("[us]")]
//            [Description("Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.")]
//            //[FieldOffset(0)]
//            public ulong time_usec;

//            /// <summary>PING sequence   </summary>
//            [Units("")]
//            [Description("PING sequence")]
//            //[FieldOffset(8)]
//            public uint seq;

//            /// <summary>0: request ping from all receiving systems. If greater than 0: message is a ping response and number is the system id of the requesting system   </summary>
//            [Units("")]
//            [Description("0: request ping from all receiving systems. If greater than 0: message is a ping response and number is the system id of the requesting system")]
//            //[FieldOffset(12)]
//            public byte target_system;

//            /// <summary>0: request ping from all receiving components. If greater than 0: message is a ping response and number is the component id of the requesting component.   </summary>
//            [Units("")]
//            [Description("0: request ping from all receiving components. If greater than 0: message is a ping response and number is the component id of the requesting component.")]
//            //[FieldOffset(13)]
//            public byte target_component;
//        };

//        /// extensions_start 0
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 3)]
//        ///<summary> Accept / deny control of this MAV </summary>
//        public struct mavlink_change_operator_control_ack_t {
//            public mavlink_change_operator_control_ack_t(byte gcs_system_id, byte control_request, byte ack) {
//                this.gcs_system_id = gcs_system_id;
//                this.control_request = control_request;
//                this.ack = ack;

//            }

//            /// <summary>ID of the GCS this message    </summary>
//            [Units("")]
//            [Description("ID of the GCS this message ")]
//            //[FieldOffset(0)]
//            public byte gcs_system_id;

//            /// <summary>0: request control of this MAV, 1: Release control of this MAV   </summary>
//            [Units("")]
//            [Description("0: request control of this MAV, 1: Release control of this MAV")]
//            //[FieldOffset(1)]
//            public byte control_request;

//            /// <summary>0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control   </summary>
//            [Units("")]
//            [Description("0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control")]
//            //[FieldOffset(2)]
//            public byte ack;
//        };

//        /// extensions_start 0
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 28)]
//        ///<summary> Request to control this MAV </summary>
//        public struct mavlink_change_operator_control_t {
//            public mavlink_change_operator_control_t(byte target_system, byte control_request, byte version, byte[] passkey) {
//                this.target_system = target_system;
//                this.control_request = control_request;
//                this.version = version;
//                this.passkey = passkey;

//            }

//            /// <summary>System the GCS requests control for   </summary>
//            [Units("")]
//            [Description("System the GCS requests control for")]
//            //[FieldOffset(0)]
//            public byte target_system;

//            /// <summary>0: request control of this MAV, 1: Release control of this MAV   </summary>
//            [Units("")]
//            [Description("0: request control of this MAV, 1: Release control of this MAV")]
//            //[FieldOffset(1)]
//            public byte control_request;

//            /// <summary>0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.  [rad] </summary>
//            [Units("[rad]")]
//            [Description("0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.")]
//            //[FieldOffset(2)]
//            public byte version;

//            /// <summary>Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and '!?,.-'   </summary>
//            [Units("")]
//            [Description("Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and '!?,.-'")]
//            //[FieldOffset(3)]
//            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 25)]
//            public byte[] passkey;
//        };

//        /// extensions_start 0
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 32)]
//        ///<summary> Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety. </summary>
//        public struct mavlink_auth_key_t {
//            public mavlink_auth_key_t(byte[] key) {
//                this.key = key;

//            }

//            /// <summary>key   </summary>
//            [Units("")]
//            [Description("key")]
//            //[FieldOffset(0)]
//            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
//            public byte[] key;
//        };