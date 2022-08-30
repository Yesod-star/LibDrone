using System;
using System.Runtime.InteropServices;
using static LibDrone.Frames.Text;

public partial class MAVLink
{
    /// extensions_start 14
    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 38)]
    ///<summary> Message encoding a mission item. This message is emitted to announce                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). NaN or INT32_MAX may be used in float/integer params (respectively) to indicate optional/default values (e.g. to use the component's current latitude, yaw rather than a specific value). See also https://mavlink.io/en/services/mission.html. </summary>
    public struct mavlink_mission_item_int_t
    {
        public mavlink_mission_item_int_t(float param1, float param2, float param3, float param4, int x, int y, float z, ushort seq,/*MAV_CMD*/ushort command, byte target_system, byte target_component,/*MAV_FRAME*/byte frame, byte current, byte autocontinue,/*MAV_MISSION_TYPE*/byte mission_type)
        {
            this.param1 = param1;
            this.param2 = param2;
            this.param3 = param3;
            this.param4 = param4;
            this.x = x;
            this.y = y;
            this.z = z;
            this.seq = seq;
            this.command = command;
            this.target_system = target_system;
            this.target_component = target_component;
            this.frame = frame;
            this.current = current;
            this.autocontinue = autocontinue;
            this.mission_type = mission_type;

        }

        /// <summary>PARAM1, see MAV_CMD enum   </summary>
        [Units("")]
        [Description("PARAM1, see MAV_CMD enum")]
        //[FieldOffset(0)]
        public float param1;

        /// <summary>PARAM2, see MAV_CMD enum   </summary>
        [Units("")]
        [Description("PARAM2, see MAV_CMD enum")]
        //[FieldOffset(4)]
        public float param2;

        /// <summary>PARAM3, see MAV_CMD enum   </summary>
        [Units("")]
        [Description("PARAM3, see MAV_CMD enum")]
        //[FieldOffset(8)]
        public float param3;

        /// <summary>PARAM4, see MAV_CMD enum   </summary>
        [Units("")]
        [Description("PARAM4, see MAV_CMD enum")]
        //[FieldOffset(12)]
        public float param4;

        /// <summary>PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7   </summary>
        [Units("")]
        [Description("PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7")]
        //[FieldOffset(16)]
        public int x;

        /// <summary>PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7   </summary>
        [Units("")]
        [Description("PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7")]
        //[FieldOffset(20)]
        public int y;

        /// <summary>PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.   </summary>
        [Units("")]
        [Description("PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.")]
        //[FieldOffset(24)]
        public float z;

        /// <summary>Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).   </summary>
        [Units("")]
        [Description("Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).")]
        //[FieldOffset(28)]
        public ushort seq;

        /// <summary>The scheduled action for the waypoint. MAV_CMD  </summary>
        [Units("")]
        [Description("The scheduled action for the waypoint.")]
        //[FieldOffset(30)]
        public  /*MAV_CMD*/ushort command;

        /// <summary>System ID   </summary>
        [Units("")]
        [Description("System ID")]
        //[FieldOffset(32)]
        public byte target_system;

        /// <summary>Component ID   </summary>
        [Units("")]
        [Description("Component ID")]
        //[FieldOffset(33)]
        public byte target_component;

        /// <summary>The coordinate system of the waypoint. MAV_FRAME  </summary>
        [Units("")]
        [Description("The coordinate system of the waypoint.")]
        //[FieldOffset(34)]
        public  /*MAV_FRAME*/byte frame;

        /// <summary>false:0, true:1   </summary>
        [Units("")]
        [Description("false:0, true:1")]
        //[FieldOffset(35)]
        public byte current;

        /// <summary>Autocontinue to next waypoint   </summary>
        [Units("")]
        [Description("Autocontinue to next waypoint")]
        //[FieldOffset(36)]
        public byte autocontinue;

        /// <summary>Mission type. MAV_MISSION_TYPE  </summary>
        [Units("")]
        [Description("Mission type.")]
        //[FieldOffset(37)]
        public  /*MAV_MISSION_TYPE*/byte mission_type;
    };

    /// extensions_start 0
    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 9)]
    ///<summary> The heartbeat message shows that a system or component is present and responding. The type and autopilot fields (along with the message component id), allow the receiving system to treat further messages from this system appropriately (e.g. by laying out the user interface based on the autopilot). This microservice is documented at https://mavlink.io/en/services/heartbeat.html </summary>
    public struct mavlink_heartbeat_t
    {
        public mavlink_heartbeat_t(uint custom_mode,/*MAV_TYPE*/byte type,/*MAV_AUTOPILOT*/byte autopilot,/*MAV_MODE_FLAG*/byte base_mode,/*MAV_STATE*/byte system_status, byte mavlink_version)
        {
            this.custom_mode = custom_mode;
            this.type = type;
            this.autopilot = autopilot;
            this.base_mode = base_mode;
            this.system_status = system_status;
            this.mavlink_version = mavlink_version;

        }

        /// <summary>A bitfield for use for autopilot-specific flags   </summary>
        [Units("")]
        [Description("A bitfield for use for autopilot-specific flags")]
        //[FieldOffset(0)]
        public uint custom_mode;

        /// <summary>Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type. MAV_TYPE  </summary>
        [Units("")]
        [Description("Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.")]
        //[FieldOffset(4)]
        public  /*MAV_TYPE*/byte type;

        /// <summary>Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers. MAV_AUTOPILOT  </summary>
        [Units("")]
        [Description("Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.")]
        //[FieldOffset(5)]
        public  /*MAV_AUTOPILOT*/byte autopilot;

        /// <summary>System mode bitmap. MAV_MODE_FLAG  bitmask</summary>
        [Units("")]
        [Description("System mode bitmap.")]
        //[FieldOffset(6)]
        public  /*MAV_MODE_FLAG*/byte base_mode;

        /// <summary>System status flag. MAV_STATE  </summary>
        [Units("")]
        [Description("System status flag.")]
        //[FieldOffset(7)]
        public  /*MAV_STATE*/byte system_status;

        /// <summary>MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version   </summary>
        [Units("")]
        [Description("MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version")]
        //[FieldOffset(8)]
        public byte mavlink_version;
    };
}