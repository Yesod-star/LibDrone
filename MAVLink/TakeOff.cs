using System.Runtime.InteropServices;
using static LibDrone.Frames.Text;


/// extensions_start 0
[System.Runtime.InteropServices.StructLayout(LayoutKind.Sequential, Pack = 1, Size = 28)]
///<summary> The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right). </summary>
public struct mavlink_attitude_t
{

    public mavlink_attitude_t(uint time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
    {
        this.time_boot_ms = time_boot_ms;
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
        this.rollspeed = rollspeed;
        this.pitchspeed = pitchspeed;
        this.yawspeed = yawspeed;

    }

    /// <summary>Timestamp (time since system boot).  [ms] </summary>
    [Units("[ms]")]
    [Description("Timestamp (time since system boot).")]
    //[FieldOffset(0)]
    public uint time_boot_ms;

    /// <summary>Roll angle (-pi..+pi)  [rad] </summary>
    [Units("[rad]")]
    [Description("Roll angle (-pi..+pi)")]
    //[FieldOffset(4)]
    public float roll;

    /// <summary>Pitch angle (-pi..+pi)  [rad] </summary>
    [Units("[rad]")]
    [Description("Pitch angle (-pi..+pi)")]
    //[FieldOffset(8)]
    public float pitch;

    /// <summary>Yaw angle (-pi..+pi)  [rad] </summary>
    [Units("[rad]")]
    [Description("Yaw angle (-pi..+pi)")]
    //[FieldOffset(12)]
    public float yaw;

    /// <summary>Roll angular speed  [rad/s] </summary>
    [Units("[rad/s]")]
    [Description("Roll angular speed")]
    //[FieldOffset(16)]
    public float rollspeed;

    /// <summary>Pitch angular speed  [rad/s] </summary>
    [Units("[rad/s]")]
    [Description("Pitch angular speed")]
    //[FieldOffset(20)]
    public float pitchspeed;

    /// <summary>Yaw angular speed  [rad/s] </summary>
    [Units("[rad/s]")]
    [Description("Yaw angular speed")]
    //[FieldOffset(24)]
    public float yawspeed;
};


/// extensions_start 0
[StructLayout(LayoutKind.Sequential, Pack = 1, Size = 37)]
///<summary> Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way. </summary>
public struct mavlink_attitude_target_t
{
    public mavlink_attitude_target_t(uint time_boot_ms, float[] q, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust,/*ATTITUDE_TARGET_TYPEMASK*/byte type_mask)
    {
        this.time_boot_ms = time_boot_ms;
        this.q = q;
        this.body_roll_rate = body_roll_rate;
        this.body_pitch_rate = body_pitch_rate;
        this.body_yaw_rate = body_yaw_rate;
        this.thrust = thrust;
        this.type_mask = type_mask;

    }

    /// <summary>Timestamp (time since system boot).  [ms] </summary>
    [Units("[ms]")]
    [Description("Timestamp (time since system boot).")]
    //[FieldOffset(0)]
    public uint time_boot_ms;

    /// <summary>Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)   </summary>
    [Units("")]
    [Description("Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)")]
    //[FieldOffset(4)]
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
    public float[] q;

    /// <summary>Body roll rate  [rad/s] </summary>
    [Units("[rad/s]")]
    [Description("Body roll rate")]
    //[FieldOffset(20)]
    public float body_roll_rate;

    /// <summary>Body pitch rate  [rad/s] </summary>
    [Units("[rad/s]")]
    [Description("Body pitch rate")]
    //[FieldOffset(24)]
    public float body_pitch_rate;

    /// <summary>Body yaw rate  [rad/s] </summary>
    [Units("[rad/s]")]
    [Description("Body yaw rate")]
    //[FieldOffset(28)]
    public float body_yaw_rate;

    /// <summary>Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)   </summary>
    [Units("")]
    [Description("Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)")]
    //[FieldOffset(32)]
    public float thrust;

    /// <summary>Bitmap to indicate which dimensions should be ignored by the vehicle. ATTITUDE_TARGET_TYPEMASK  bitmask</summary>
    [Units("")]
    [Description("Bitmap to indicate which dimensions should be ignored by the vehicle.")]
    //[FieldOffset(36)]
    public  /*ATTITUDE_TARGET_TYPEMASK*/byte type_mask;
};

/// extensions_start 8
[StructLayout(LayoutKind.Sequential, Pack = 1, Size = 48)]
///<summary> The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0). </summary>
public struct mavlink_attitude_quaternion_t
{
    public mavlink_attitude_quaternion_t(uint time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed, float[] repr_offset_q)
    {
        this.time_boot_ms = time_boot_ms;
        this.q1 = q1;
        this.q2 = q2;
        this.q3 = q3;
        this.q4 = q4;
        this.rollspeed = rollspeed;
        this.pitchspeed = pitchspeed;
        this.yawspeed = yawspeed;
        this.repr_offset_q = repr_offset_q;

    }

    /// <summary>Timestamp (time since system boot).  [ms] </summary>
    [Units("[ms]")]
    [Description("Timestamp (time since system boot).")]
    //[FieldOffset(0)]
    public uint time_boot_ms;

    /// <summary>Quaternion component 1, w (1 in null-rotation)   </summary>
    [Units("")]
    [Description("Quaternion component 1, w (1 in null-rotation)")]
    //[FieldOffset(4)]
    public float q1;

    /// <summary>Quaternion component 2, x (0 in null-rotation)   </summary>
    [Units("")]
    [Description("Quaternion component 2, x (0 in null-rotation)")]
    //[FieldOffset(8)]
    public float q2;

    /// <summary>Quaternion component 3, y (0 in null-rotation)   </summary>
    [Units("")]
    [Description("Quaternion component 3, y (0 in null-rotation)")]
    //[FieldOffset(12)]
    public float q3;

    /// <summary>Quaternion component 4, z (0 in null-rotation)   </summary>
    [Units("")]
    [Description("Quaternion component 4, z (0 in null-rotation)")]
    //[FieldOffset(16)]
    public float q4;

    /// <summary>Roll angular speed  [rad/s] </summary>
    [Units("[rad/s]")]
    [Description("Roll angular speed")]
    //[FieldOffset(20)]
    public float rollspeed;

    /// <summary>Pitch angular speed  [rad/s] </summary>
    [Units("[rad/s]")]
    [Description("Pitch angular speed")]
    //[FieldOffset(24)]
    public float pitchspeed;

    /// <summary>Yaw angular speed  [rad/s] </summary>
    [Units("[rad/s]")]
    [Description("Yaw angular speed")]
    //[FieldOffset(28)]
    public float yawspeed;

    /// <summary>Rotation offset by which the attitude quaternion and angular speed vector should be rotated for user display (quaternion with [w, x, y, z] order, zero-rotation is [1, 0, 0, 0], send [0, 0, 0, 0] if field not supported). This field is intended for systems in which the reference attitude may change during flight. For example, tailsitters VTOLs rotate their reference attitude by 90 degrees between hover mode and fixed wing mode, thus repr_offset_q is equal to [1, 0, 0, 0] in hover mode and equal to [0.7071, 0, 0.7071, 0] in fixed wing mode.   </summary>
    [Units("")]
    [Description("Rotation offset by which the attitude quaternion and angular speed vector should be rotated for user display (quaternion with [w, x, y, z] order, zero-rotation is [1, 0, 0, 0], send [0, 0, 0, 0] if field not supported). This field is intended for systems in which the reference attitude may change during flight. For example, tailsitters VTOLs rotate their reference attitude by 90 degrees between hover mode and fixed wing mode, thus repr_offset_q is equal to [1, 0, 0, 0] in hover mode and equal to [0.7071, 0, 0.7071, 0] in fixed wing mode.")]
    //[FieldOffset(32)]
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
    public float[] repr_offset_q;
};

/// extensions_start 0
[StructLayout(LayoutKind.Sequential, Pack = 1, Size = 39)]
///<summary> Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller or other system). </summary>
public struct mavlink_set_attitude_target_t
{
    public mavlink_set_attitude_target_t(uint time_boot_ms, float[] q, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust, byte target_system, byte target_component,/*ATTITUDE_TARGET_TYPEMASK*/byte type_mask)
    {
        this.time_boot_ms = time_boot_ms;
        this.q = q;
        this.body_roll_rate = body_roll_rate;
        this.body_pitch_rate = body_pitch_rate;
        this.body_yaw_rate = body_yaw_rate;
        this.thrust = thrust;
        this.target_system = target_system;
        this.target_component = target_component;
        this.type_mask = type_mask;

    }

    /// <summary>Timestamp (time since system boot).  [ms] </summary>
    [Units("[ms]")]
    [Description("Timestamp (time since system boot).")]
    //[FieldOffset(0)]
    public uint time_boot_ms;

    /// <summary>Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)   </summary>
    [Units("")]
    [Description("Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)")]
    //[FieldOffset(4)]
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
    public float[] q;

    /// <summary>Body roll rate  [rad/s] </summary>
    [Units("[rad/s]")]
    [Description("Body roll rate")]
    //[FieldOffset(20)]
    public float body_roll_rate;

    /// <summary>Body pitch rate  [rad/s] </summary>
    [Units("[rad/s]")]
    [Description("Body pitch rate")]
    //[FieldOffset(24)]
    public float body_pitch_rate;

    /// <summary>Body yaw rate  [rad/s] </summary>
    [Units("[rad/s]")]
    [Description("Body yaw rate")]
    //[FieldOffset(28)]
    public float body_yaw_rate;

    /// <summary>Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)   </summary>
    [Units("")]
    [Description("Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)")]
    //[FieldOffset(32)]
    public float thrust;

    /// <summary>System ID   </summary>
    [Units("")]
    [Description("System ID")]
    //[FieldOffset(36)]
    public byte target_system;

    /// <summary>Component ID   </summary>
    [Units("")]
    [Description("Component ID")]
    //[FieldOffset(37)]
    public byte target_component;

    /// <summary>Bitmap to indicate which dimensions should be ignored by the vehicle. ATTITUDE_TARGET_TYPEMASK  bitmask</summary>
    [Units("")]
    [Description("Bitmap to indicate which dimensions should be ignored by the vehicle.")]
    //[FieldOffset(38)]
    public  /*ATTITUDE_TARGET_TYPEMASK*/byte type_mask;
};