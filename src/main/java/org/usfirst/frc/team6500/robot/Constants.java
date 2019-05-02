package org.usfirst.frc.team6500.robot;


import org.usfirst.frc.team6500.trc.util.TRCTypes.*;


public class Constants
{
    // Input Constants
    public static final int INPUT_DRIVER_PORT           = 0;
    public static final int INPUT_GUNNER_PORT           = 1;
    public static final int INPUT_PORTS[]               = {INPUT_DRIVER_PORT, INPUT_GUNNER_PORT};
    public static final ControllerType INPUT_TYPES[]    = {ControllerType.Xbox360, ControllerType.Extreme3D};

    public static final int INPUT_ARM_UP_BUTTON         = 6;
    public static final int INPUT_ARM_DOWN_BUTTON       = 4;
    public static final Object INPUT_ARM_BUTTONS[]      = {INPUT_ARM_UP_BUTTON, INPUT_ARM_DOWN_BUTTON};

    public static final int INPUT_LIFT_ELEVATE_BUTTON   = 1;
    public static final int INPUT_LIFT_DESCEND_BUTTON   = 2;
    public static final Object INPUT_LIFT_BUTTONS[]     = {INPUT_LIFT_ELEVATE_BUTTON, INPUT_LIFT_DESCEND_BUTTON};

    public static final int INPUT_GRABBER_INTAKE_BUTTON = 7;
    public static final int INPUT_GRABBER_EXPEL_BUTTON  = 8;
    public static final Object INPUT_GRABBER_BUTTONS[]  = {INPUT_GRABBER_INTAKE_BUTTON, INPUT_GRABBER_EXPEL_BUTTON};

    public static final int INPUT_POKIE_EXTEND_BUTTON   = 3;
    public static final int INPUT_POKIE_RETRACT_BUTTON  = 5;

    public static final double INPUT_DRIVE_DEADBAND     = 0.1;
    // ternary ftw
    public static final int INPUT_DRIVE_SLOW            = (INPUT_TYPES[INPUT_DRIVER_PORT] == ControllerType.Xbox360) ? 5 : 2;
    public static final Object INPUT_DRIVE_BUTTONS[]    = {INPUT_DRIVE_SLOW};

    public static final int INPUT_AUTO_LINE_BUTTON      = 12;
    public static final int INPUT_AUTO_KILL_BUTTON      = 11;
    public static final int INPUT_AUTO_CARGO_BUTTON     = 12;
    public static final int INPUT_ARM_HATCH_BUTTON      = 10;
    public static final int INPUT_ARM_EASE_BUTTON       = 9;
    // public static final int INPUT_AUTO_GET_PANEL      = 3;
    // public static final int INPUT_AUTO_GET_CARGO      = 4;
    // public static final int INPUT_AUTO_L1_PANEL       = 11;
    // public static final int INPUT_AUTO_L2_PANEL       = 9;
    // public static final int INPUT_AUTO_L3_PANEL       = 7;
    // public static final int INPUT_AUTO_L1_CARGO       = 12;
    // public static final int INPUT_AUTO_L2_CARGO       = 10;
    // public static final int INPUT_AUTO_L3_CARGO       = 8;


    // Speed Constants
    public static final double SPEED_BASE              = 0.85;
    public static final double SPEED_BOOST             = 0.95;
    public static final double SPEED_AUTO_TAPE         = 0.30;
    public static final double SPEED_AUTO_LINE         = 0.275;
    public static final double SPEED_AUTO_LINE_STRAFE  = 0.325;
    public static final double SPEED_AUTO_LINE_TURN    = 0.2;


    // Drive Constants
    public final static int DRIVE_WHEEL_FRONTLEFT               = 2;
    public final static int DRIVE_WHEEL_REARLEFT                = 4;
	public final static int DRIVE_WHEEL_FRONTRIGHT              = 1;
    public final static int DRIVE_WHEEL_REARRIGHT               = 3;
    public final static int DRIVE_WHEEL_PORTS[]                 = {DRIVE_WHEEL_FRONTLEFT, DRIVE_WHEEL_REARLEFT, DRIVE_WHEEL_FRONTRIGHT, DRIVE_WHEEL_REARRIGHT};
    public final static SpeedControllerType DRIVE_WHEEL_TYPES[] = {SpeedControllerType.CANTalonSRX, SpeedControllerType.CANTalonSRX, SpeedControllerType.CANTalonSRX, SpeedControllerType.CANTalonSRX};
    public final static boolean DRIVE_WHEEL_INVERTS[]           = {false, true, false, true};

                                                                                                                                     
    // Directional System (Grabber/Lift/Arm/Pokie) Constants
    public final static int LIFT_MOTOR_LEFT                       = 6;
    public final static int LIFT_MOTOR_RIGHT                      = 5;
    public final static int LIFT_MOTORS[]                         = {LIFT_MOTOR_LEFT, LIFT_MOTOR_RIGHT};
    public final static SpeedControllerType LIFT_MOTOR_TYPES[]    = {SpeedControllerType.CANTalonSRX, SpeedControllerType.CANTalonSRX};
    public final static double LIFT_SPEED_UP                      = -0.75;
    public final static double LIFT_SPEED_DOWN                    = 0.3;
    public final static int LIFT_SWITCH_PORTS[]                   = {0, 1, 2, 3, 4};
    public final static int LIFT_MAX_STALL_CURRENT                = 5;
    public final static double LIFT_TARGET_HEIGHTS[]              = {0, -17000};
    public final static int LIFT_BOTTOM_SWITCH                    = 9;
    public final static double LIFT_TIME_CARGO                    = 1.25;
    
    public final static int GRABBER_MOTOR_LEFT                    = 8;
    public final static int GRABBER_MOTOR_RIGHT                   = 9;
    public final static int GRABBER_MOTORS[]                      = {GRABBER_MOTOR_LEFT, GRABBER_MOTOR_RIGHT};
    public final static SpeedControllerType GRABBER_MOTOR_TYPES[] = {SpeedControllerType.CANTalonSRX, SpeedControllerType.CANTalonSRX};
    public final static double GRABBER_SPEED_EXPEL                = 1.0;
    public final static double GRABBER_SPEED_INTAKE               = -0.5;
    public final static double GRABBER_TIME_CARGO                 = 1.25;

    public final static int ARM_MOTOR                             = 7;
    public final static int ARM_MOTORS[]                          = {ARM_MOTOR};
    public final static SpeedControllerType ARM_MOTOR_TYPES[]     = {SpeedControllerType.CANTalonSRX};
    public final static double ARM_DISTANCE_PER_PULSE             = 0.0;
    public final static double ARM_POSITION_UP                    = 7000.0;
    public final static double ARM_POSITION_DOWN                  = 850.0;
    public final static double ARM_POSITION_HATCH                 = 3000.0;
    public final static double ARM_POSITION_EASE                  = -50.0;
    public final static double ARM_SPEED_UP                       = -0.5;
    public final static double ARM_SPEED_DOWN                     = 0.25;
    public final static int ARM_MAX_STALL_CURRENT                 = 5;

    public final static int POKIE_PORT_EXTEND                     = 0;
    public final static int POKIE_PORT_RETRACT                    = 1;
    public final static int POKIE_PORTS[]                         = {POKIE_PORT_EXTEND, POKIE_PORT_RETRACT};


    // Encoder Constants
    public final static int ENCODER_INPUTS[]                 = {DRIVE_WHEEL_REARRIGHT, 0, DRIVE_WHEEL_FRONTRIGHT, 0, DRIVE_WHEEL_REARLEFT, 0, DRIVE_WHEEL_FRONTLEFT, 0};
    public final static EncoderType ENCODER_TYPES[]          = {EncoderType.Talon, EncoderType.Talon, EncoderType.Talon, EncoderType.Talon};
    public final static double[] ENCODER_DISTANCES_PER_PULSE = {-0.00605914, -0.00607312, -0.00612032, -0.006049241};
  

    // Servos (Ramps) Constants
    public final static int RAMP_LEFT_SERVO_CHANNEL   = 0;
    public final static int RAMP_RIGHT_SERVO_CHANNEL  = 1;
    public final static int RAMP_LEFT_SERVO_ANGLE     = 1;
    public final static int RAMP_RIGHT_SERVO_ANGLE    = 1;


    // Proximity Sensor Constants
    public final static int PROXIMITY_LEFT            = 0;
    public final static int PROXIMITY_RIGHT           = 1;
    public final static double PROXIMITY_THRESHOLD_MM = 350.0;


    // Pneumatics Constants
    public final static int PNEUMATICS_PCM_ID = 10;


    // PDP Constants
    public final static int PDP_CAN_ID = 0;


    public static enum CargoPositionType
    {
        Close,
        Middle,
        Far
    }
}