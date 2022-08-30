//using System;
//using System.Runtime.InteropServices;
//using static LibDrone.Frames.Text;


//        /// extensions_start 8
//        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 60)]
//        ///<summary> The location of a landing target. See: https://mavlink.io/en/services/landing_target.html </summary>
//        public struct mavlink_landing_target_t {
//            public mavlink_landing_target_t(ulong time_usec, float angle_x, float angle_y, float distance, float size_x, float size_y, byte target_num,/*MAV_FRAME*/byte frame, float x, float y, float z, float[] q,/*LANDING_TARGET_TYPE*/byte type, byte position_valid) {
//                this.time_usec = time_usec;
//                this.angle_x = angle_x;
//                this.angle_y = angle_y;
//                this.distance = distance;
//                this.size_x = size_x;
//                this.size_y = size_y;
//                this.target_num = target_num;
//                this.frame = frame;
//                this.x = x;
//                this.y = y;
//                this.z = z;
//                this.q = q;
//                this.type = type;
//                this.position_valid = position_valid;

//            }

//            /// <summary>Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.  [us] </summary>
//            [Units("[us]")]
//            [Description("Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.")]
//            //[FieldOffset(0)]
//            public ulong time_usec;

//            /// <summary>X-axis angular offset of the target from the center of the image  [rad] </summary>
//            [Units("[rad]")]
//            [Description("X-axis angular offset of the target from the center of the image")]
//            //[FieldOffset(8)]
//            public float angle_x;

//            /// <summary>Y-axis angular offset of the target from the center of the image  [rad] </summary>
//            [Units("[rad]")]
//            [Description("Y-axis angular offset of the target from the center of the image")]
//            //[FieldOffset(12)]
//            public float angle_y;

//            /// <summary>Distance to the target from the vehicle  [m] </summary>
//            [Units("[m]")]
//            [Description("Distance to the target from the vehicle")]
//            //[FieldOffset(16)]
//            public float distance;

//            /// <summary>Size of target along x-axis  [rad] </summary>
//            [Units("[rad]")]
//            [Description("Size of target along x-axis")]
//            //[FieldOffset(20)]
//            public float size_x;

//            /// <summary>Size of target along y-axis  [rad] </summary>
//            [Units("[rad]")]
//            [Description("Size of target along y-axis")]
//            //[FieldOffset(24)]
//            public float size_y;

//            /// <summary>The ID of the target if multiple targets are present   </summary>
//            [Units("")]
//            [Description("The ID of the target if multiple targets are present")]
//            //[FieldOffset(28)]
//            public byte target_num;

//            /// <summary>Coordinate frame used for following fields. MAV_FRAME  </summary>
//            [Units("")]
//            [Description("Coordinate frame used for following fields.")]
//            //[FieldOffset(29)]
//            public  /*MAV_FRAME*/byte frame;

//            /// <summary>X Position of the landing target in MAV_FRAME  [m] </summary>
//            [Units("[m]")]
//            [Description("X Position of the landing target in MAV_FRAME")]
//            //[FieldOffset(30)]
//            public float x;

//            /// <summary>Y Position of the landing target in MAV_FRAME  [m] </summary>
//            [Units("[m]")]
//            [Description("Y Position of the landing target in MAV_FRAME")]
//            //[FieldOffset(34)]
//            public float y;

//            /// <summary>Z Position of the landing target in MAV_FRAME  [m] </summary>
//            [Units("[m]")]
//            [Description("Z Position of the landing target in MAV_FRAME")]
//            //[FieldOffset(38)]
//            public float z;

//            /// <summary>Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)   </summary>
//            [Units("")]
//            [Description("Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)")]
//            //[FieldOffset(42)]
//            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
//            public float[] q;

//            /// <summary>Type of landing target LANDING_TARGET_TYPE  </summary>
//            [Units("")]
//            [Description("Type of landing target")]
//            //[FieldOffset(58)]
//            public  /*LANDING_TARGET_TYPE*/byte type;

//            /// <summary>Boolean indicating whether the position fields (x, y, z, q, type) contain valid target position information (valid: 1, invalid: 0). Default is 0 (invalid).   </summary>
//            [Units("")]
//            [Description("Boolean indicating whether the position fields (x, y, z, q, type) contain valid target position information (valid: 1, invalid: 0). Default is 0 (invalid).")]
//            //[FieldOffset(59)]
//            public byte position_valid;
//        };