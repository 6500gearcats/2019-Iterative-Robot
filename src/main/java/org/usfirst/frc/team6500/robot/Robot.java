package org.usfirst.frc.team6500.robot;


import org.usfirst.frc.team6500.robot.Constants;
import org.usfirst.frc.team6500.trc.auto.TRCDirectionalSystemAction;
import org.usfirst.frc.team6500.trc.auto.TRCDrivePID;
import org.usfirst.frc.team6500.trc.auto.TRCPneumaticSystemAction;
import org.usfirst.frc.team6500.trc.sensors.TRCCamera;
import org.usfirst.frc.team6500.trc.sensors.TRCNetworkVision;
import org.usfirst.frc.team6500.trc.systems.TRCDirectionalSystem;
import org.usfirst.frc.team6500.trc.systems.TRCDriveInput;
import org.usfirst.frc.team6500.trc.systems.TRCPneumaticSystem;
import org.usfirst.frc.team6500.trc.util.TRCNetworkData;
import org.usfirst.frc.team6500.trc.util.TRCTypes;
import org.usfirst.frc.team6500.trc.util.TRCDriveParams;
import org.usfirst.frc.team6500.trc.wrappers.sensors.TRCEncoderSet;
import org.usfirst.frc.team6500.trc.wrappers.sensors.TRCGyroBase;
import org.usfirst.frc.team6500.trc.wrappers.systems.drives.TRCMecanumDrive;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotBase;


public class Robot extends TimedRobot
{
    // Robot member definitions
    TRCGyroBase gyro;
    TRCEncoderSet encoders;
    TRCMecanumDrive drive;
    TRCDirectionalSystem lift, grabber, arm;
    TRCPneumaticSystem pokie;
    int positionOptionID, targetOptionID;


    /**
     * Code here will run once as soon as the robot starts
     */
    @Override
    public void robotInit()
    {
        // Setup: Communications
        TRCNetworkData.initializeNetworkData(TRCTypes.DataInterfaceType.Board);
        TRCNetworkVision.initializeVision();
        //TRCCamera.initializeCamera();


        // Setup: Systems: Drivetrain
        drive = new TRCMecanumDrive(Constants.DRIVE_WHEEL_PORTS, Constants.DRIVE_WHEEL_TYPES, Constants.DRIVE_WHEEL_INVERTS, true);
 
        // Setup: Systems: Directional
        lift    = new Lift(Constants.LIFT_MOTORS, Constants.LIFT_MOTOR_TYPES);
        grabber = new Grabber(Constants.GRABBER_MOTORS, Constants.GRABBER_MOTOR_TYPES);
        arm     = new Arm(Constants.ARM_MOTORS, Constants.ARM_MOTOR_TYPES);
        TRCDirectionalSystemAction.registerSystem("Lift", lift);
        TRCDirectionalSystemAction.registerSystem("Grabber", grabber);
        TRCDirectionalSystemAction.registerSystem("Arm", arm);

        TRCPneumaticSystem.setupPneumatics(Constants.PNEUMATICS_PCM_ID);
        pokie = new TRCPneumaticSystem(Constants.POKIE_PORTS, true);
        TRCPneumaticSystemAction.registerSystem("Pokie", pokie);

        Ramps.initilizeRamps();


        // Setup: Systems: Sensors
        gyro = new TRCGyroBase(TRCTypes.GyroType.NavX);
        encoders = new TRCEncoderSet(Constants.ENCODER_INPUTS, Constants.ENCODER_DISTANCES_PER_PULSE, false, 4, Constants.ENCODER_TYPES);
        AnalogInput leftProx  = new AnalogInput(Constants.PROXIMITY_LEFT);
        AnalogInput rightProx = new AnalogInput(Constants.PROXIMITY_RIGHT);


        // Setup: Autonomous
        TRCDrivePID.initializeTRCDrivePID(encoders, gyro, drive, TRCTypes.DriveType.Mecanum, Constants.SPEED_AUTO_TAPE);
        AutoAlign.setupAlignment(drive, leftProx, rightProx);


        // Setup: Input
        TRCDriveInput.initializeDriveInput(Constants.INPUT_PORTS, Constants.INPUT_TYPES, Constants.SPEED_BASE, Constants.SPEED_BOOST);

        // Setup: Input: Button Bindings: Autonomous Functions
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT, Constants.INPUT_AUTO_GET_PANEL, AutoProcess::obtainPanel);
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT, Constants.INPUT_AUTO_GET_CARGO, AutoProcess::obtainCargo);
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT, Constants.INPUT_AUTO_L1_PANEL, AutoProcess::levelOnePanel);
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT, Constants.INPUT_AUTO_L2_PANEL, AutoProcess::levelTwoPanel);
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT, Constants.INPUT_AUTO_L3_PANEL, AutoProcess::levelThreePanel);
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT, Constants.INPUT_AUTO_L1_CARGO, AutoProcess::levelOneCargo);
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT, Constants.INPUT_AUTO_L2_CARGO, AutoProcess::levelTwoCargo);
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT, Constants.INPUT_AUTO_L3_CARGO, AutoProcess::levelThreeCargo);

        // Setup: Input: Button Bindings: Ramps
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT, Constants.INPUT_RAMP_RELEASE_BUTTON, Ramps::releaseRamps);
    }

    /**
     * Code here will run once at the start of autonomous
     */
    @Override
    public void autonomousInit()
    {
        encoders.resetAllEncoders();
        gyro.reset();
    }

    /**
     * Code here will run continously during autonomous
     */
    @Override
    public void autonomousPeriodic()
    {
        // Check all inputs
        TRCDriveInput.checkButtonBindings();
        // And drive the robot
        drive.driveCartesian(TRCDriveInput.getStickDriveParams(Constants.INPUT_DRIVER_PORT));
    }

    /**
     * Code here will run once at the start of teleop
     */
    @Override
    public void teleopInit()
    {
        // Nothing to do here ¯\_(ツ)_/¯
    }

    /**
     * Code here will run continously during teleop
     */
    @Override
    public void teleopPeriodic()
    {
        // Check all inputs
        TRCDriveInput.checkButtonBindings();
        // And drive the robot
        TRCDriveParams input = TRCDriveInput.getStickDriveParams(Constants.INPUT_DRIVER_PORT);
        double x = input.getRawX();
        double z = input.getRawZ();
        input.setRawX(z);
        input.setRawZ(x);
        drive.driveCartesian(input);
    }

    public static void main(String... args)
    {
        RobotBase.startRobot(Robot::new);
    }
}