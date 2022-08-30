//using System;
//using System.Runtime.InteropServices;
//using static LibDrone.Frames.Text;

//        [Obsolete]
//        /// extensions_start 0
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 6)]
//        ///<summary> Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component. </summary>
//        public struct mavlink_set_mode_t {
//            public mavlink_set_mode_t(uint custom_mode, byte target_system,/*MAV_MODE*/byte base_mode) {
//                this.custom_mode = custom_mode;
//                this.target_system = target_system;
//                this.base_mode = base_mode;

//            }

//            /// <summary>The new autopilot-specific mode. This field can be ignored by an autopilot.   </summary>
//            [Units("")]
//            [Description("The new autopilot-specific mode. This field can be ignored by an autopilot.")]
//            //[FieldOffset(0)]
//            public uint custom_mode;

//            /// <summary>The system setting the mode   </summary>
//            [Units("")]
//            [Description("The system setting the mode")]
//            //[FieldOffset(4)]
//            public byte target_system;

//            /// <summary>The new base mode. MAV_MODE  </summary>
//            [Units("")]
//            [Description("The new base mode.")]
//            //[FieldOffset(5)]
//            public  /*MAV_MODE*/byte base_mode;
//        };

//        /// extensions_start 0
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 20)]
//        ///<summary> Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also https://mavlink.io/en/services/parameter.html for a full documentation of QGroundControl and IMU code. </summary>
//        public struct mavlink_param_request_read_t {
//            public mavlink_param_request_read_t(short param_index, byte target_system, byte target_component, byte[] param_id) {
//                this.param_index = param_index;
//                this.target_system = target_system;
//                this.target_component = target_component;
//                this.param_id = param_id;

//            }

//            /// <summary>Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)   </summary>
//            [Units("")]
//            [Description("Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)")]
//            //[FieldOffset(0)]
//            public short param_index;

//            /// <summary>System ID   </summary>
//            [Units("")]
//            [Description("System ID")]
//            //[FieldOffset(2)]
//            public byte target_system;

//            /// <summary>Component ID   </summary>
//            [Units("")]
//            [Description("Component ID")]
//            //[FieldOffset(3)]
//            public byte target_component;

//            /// <summary>Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string   </summary>
//            [Units("")]
//            [Description("Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string")]
//            //[FieldOffset(4)]
//            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
//            public byte[] param_id;
//        };

//        /// extensions_start 0
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 2)]
//        ///<summary> Request all parameters of this component. After this request, all parameters are emitted. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html </summary>
//        public struct mavlink_param_request_list_t {
//            public mavlink_param_request_list_t(byte target_system, byte target_component) {
//                this.target_system = target_system;
//                this.target_component = target_component;

//            }

//            /// <summary>System ID   </summary>
//            [Units("")]
//            [Description("System ID")]
//            //[FieldOffset(0)]
//            public byte target_system;

//            /// <summary>Component ID   </summary>
//            [Units("")]
//            [Description("Component ID")]
//            //[FieldOffset(1)]
//            public byte target_component;
//        };

//        /// extensions_start 0
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 25)]
//        ///<summary> Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html </summary>
//        public struct mavlink_param_value_t {
//            public mavlink_param_value_t(float param_value, ushort param_count, ushort param_index, byte[] param_id,/*MAV_PARAM_TYPE*/byte param_type) {
//                this.param_value = param_value;
//                this.param_count = param_count;
//                this.param_index = param_index;
//                this.param_id = param_id;
//                this.param_type = param_type;

//            }

//            /// <summary>Onboard parameter value   </summary>
//            [Units("")]
//            [Description("Onboard parameter value")]
//            //[FieldOffset(0)]
//            public float param_value;

//            /// <summary>Total number of onboard parameters   </summary>
//            [Units("")]
//            [Description("Total number of onboard parameters")]
//            //[FieldOffset(4)]
//            public ushort param_count;

//            /// <summary>Index of this onboard parameter   </summary>
//            [Units("")]
//            [Description("Index of this onboard parameter")]
//            //[FieldOffset(6)]
//            public ushort param_index;

//            /// <summary>Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string   </summary>
//            [Units("")]
//            [Description("Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string")]
//            //[FieldOffset(8)]
//            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
//            public byte[] param_id;

//            /// <summary>Onboard parameter type. MAV_PARAM_TYPE  </summary>
//            [Units("")]
//            [Description("Onboard parameter type.")]
//            //[FieldOffset(24)]
//            public  /*MAV_PARAM_TYPE*/byte param_type;
//        };

//        /// extensions_start 0
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 23)]
//        ///<summary> Set a parameter value (write new value to permanent storage).         The receiving component should acknowledge the new parameter value by broadcasting a PARAM_VALUE message (broadcasting ensures that multiple GCS all have an up-to-date list of all parameters). If the sending GCS did not receive a PARAM_VALUE within its timeout time, it should re-send the PARAM_SET message. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html.          </summary>
//        public struct mavlink_param_set_t {
//            public mavlink_param_set_t(float param_value, byte target_system, byte target_component, byte[] param_id,/*MAV_PARAM_TYPE*/byte param_type) {
//                this.param_value = param_value;
//                this.target_system = target_system;
//                this.target_component = target_component;
//                this.param_id = param_id;
//                this.param_type = param_type;

//            }

//            /// <summary>Onboard parameter value   </summary>
//            [Units("")]
//            [Description("Onboard parameter value")]
//            //[FieldOffset(0)]
//            public float param_value;

//            /// <summary>System ID   </summary>
//            [Units("")]
//            [Description("System ID")]
//            //[FieldOffset(4)]
//            public byte target_system;

//            /// <summary>Component ID   </summary>
//            [Units("")]
//            [Description("Component ID")]
//            //[FieldOffset(5)]
//            public byte target_component;

//            /// <summary>Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string   </summary>
//            [Units("")]
//            [Description("Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string")]
//            //[FieldOffset(6)]
//            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
//            public byte[] param_id;

//            /// <summary>Onboard parameter type. MAV_PARAM_TYPE  </summary>
//            [Units("")]
//            [Description("Onboard parameter type.")]
//            //[FieldOffset(22)]
//            public  /*MAV_PARAM_TYPE*/byte param_type;
//        };