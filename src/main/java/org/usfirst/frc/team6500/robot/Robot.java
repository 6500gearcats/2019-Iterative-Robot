package org.usfirst.frc.team6500.robot;

import org.usfirst.frc.team6500.robot.Constants;
import org.usfirst.frc.team6500.robot.Constants.CargoPositionType;
import org.usfirst.frc.team6500.trc.auto.TRCDirectionalSystemAction;
import org.usfirst.frc.team6500.trc.auto.TRCDrivePID;
import org.usfirst.frc.team6500.trc.auto.TRCDriveContinuous;
import org.usfirst.frc.team6500.trc.auto.TRCDriveSync;
import org.usfirst.frc.team6500.trc.auto.TRCPneumaticSystemAction;
import org.usfirst.frc.team6500.trc.sensors.TRCCamera;
import org.usfirst.frc.team6500.trc.sensors.TRCNetworkVision;
import org.usfirst.frc.team6500.trc.systems.TRCDirectionalSystem;
import org.usfirst.frc.team6500.trc.systems.TRCDriveInput;
import org.usfirst.frc.team6500.trc.systems.TRCPneumaticSystem;
import org.usfirst.frc.team6500.trc.util.TRCNetworkData;
import org.usfirst.frc.team6500.trc.util.TRCTypes.*;
import org.usfirst.frc.team6500.trc.util.TRCDriveParams;
import org.usfirst.frc.team6500.trc.wrappers.sensors.TRCEncoderSet;
import org.usfirst.frc.team6500.trc.wrappers.sensors.TRCGyroBase;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotBase;

public class Robot extends TimedRobot {
    // Robot member definitions
    TRCGyroBase gyro;
    TRCEncoderSet encoders;
    Drive drive;
    TRCDirectionalSystem lift, grabber;
    Arm arm;
    TRCPneumaticSystem pokie;
    AnalogInput leftProx, rightProx;
    DigitalInput liftBottom;
    int positionOptionID, targetOptionID;

    /**
     * Code here will run once as soon as the robot starts
     */
    @Override
    public void robotInit() {
        // Setup: Communications
        TRCNetworkData.initializeNetworkData(DataInterfaceType.Board);
        TRCNetworkData.createDataPoint("Encoder Output");
        TRCNetworkData.createDataPoint("Gyro");
        TRCNetworkData.createDataPoint("Switch");
        TRCNetworkData.createDataPoint("Arm");
        TRCNetworkData.createDataPoint("LFC");
        TRCNetworkData.createDataPoint("Line Visible?");
        TRCNetworkData.createDataPoint("Left Proximity");
        TRCNetworkData.createDataPoint("Right Proximity");
        TRCNetworkVision.initializeVision();
        TRCCamera.initializeCamera();

        // Setup: Systems: Drivetrain
        drive = new Drive(Constants.DRIVE_WHEEL_PORTS, Constants.DRIVE_WHEEL_TYPES, Constants.DRIVE_WHEEL_INVERTS,
                true);
        TRCDriveSync.initializeTRCDriveSync();

        // Setup: Systems: Directional
        lift = new Lift(Constants.LIFT_MOTORS, Constants.LIFT_MOTOR_TYPES);
        grabber = new Grabber(Constants.GRABBER_MOTORS, Constants.GRABBER_MOTOR_TYPES);
        arm = new Arm(Constants.ARM_MOTORS, Constants.ARM_MOTOR_TYPES);
        TRCDirectionalSystemAction.registerSystem("Lift", lift);
        TRCDirectionalSystemAction.registerSystem("Grabber", grabber);
        TRCDirectionalSystemAction.registerSystem("Arm", arm);

        TRCPneumaticSystem.setupPneumatics(Constants.PNEUMATICS_PCM_ID);
        pokie = new TRCPneumaticSystem(Constants.POKIE_PORTS, true);
        TRCPneumaticSystemAction.registerSystem("Pokie", pokie);

        // Setup: Systems: Sensors
        gyro = new TRCGyroBase(GyroType.ADXRS450);
        encoders = new TRCEncoderSet(Constants.ENCODER_INPUTS, Constants.ENCODER_DISTANCES_PER_PULSE, false, 4,
                Constants.ENCODER_TYPES);
        encoders.resetAllEncoders();
        leftProx = new AnalogInput(Constants.PROXIMITY_LEFT);
        rightProx = new AnalogInput(Constants.PROXIMITY_RIGHT);

        liftBottom = new DigitalInput(Constants.LIFT_BOTTOM_SWITCH);

        // Setup: Autonomous
        TRCDrivePID.initializeTRCDrivePID(encoders, gyro, drive, DriveType.Mecanum, Constants.SPEED_AUTO_TAPE);
        AutoAlign.setupAlignment(drive, leftProx, rightProx);
        TRCDriveContinuous.initializeTRCDriveContinuous(drive, DriveType.Mecanum, Constants.SPEED_AUTO_TAPE);
        TRCDriveSync.requestChangeState(DriveSyncState.Teleop);

        AssistedControl.initializeAssistedControl(8);
        AssistedControl.startCommunications();
        AssistedControl.pauseCommunications();

        // Setup: Input
        TRCDriveInput.initializeDriveInput(Constants.INPUT_PORTS, Constants.INPUT_TYPES, Constants.SPEED_BASE,
                Constants.SPEED_BOOST);

        // Setup: Input: Button Bindings: Autonomous Functions
        //TRCDriveInput.bindButtonPress(Constants.INPUT_GUNNER_PORT, Constants.INPUT_AUTO_LINE_BUTTON,
        //        AssistedControl::startCommunications);
        //TRCDriveInput.bindButtonPress(Constants.INPUT_GUNNER_PORT, Constants.INPUT_AUTO_KILL_BUTTON,
        //        AssistedControl::pauseCommunications);
        TRCDriveInput.bindButtonPress(Constants.INPUT_GUNNER_PORT, Constants.INPUT_AUTO_LINE_BUTTON,
                AssistedControl::startCommunications);
        TRCDriveInput.bindButtonPress(Constants.INPUT_GUNNER_PORT, Constants.INPUT_AUTO_KILL_BUTTON,
                AssistedControl::pauseCommunications);
        TRCDriveInput.bindButtonPress(Constants.INPUT_GUNNER_PORT, Constants.INPUT_ARM_HATCH_BUTTON,
                arm::armToHatch);
        TRCDriveInput.bindButtonPress(Constants.INPUT_GUNNER_PORT, Constants.INPUT_ARM_EASE_BUTTON,
                arm::atEasePrivateArm);
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT,
        // Constants.INPUT_AUTO_GET_PANEL, AutoProcess::obtainPanel);
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT,
        // Constants.INPUT_AUTO_GET_CARGO, AutoProcess::obtainCargo);
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT,
        // Constants.INPUT_AUTO_L1_PANEL, AutoProcess::levelOnePanel);
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT,
        // Constants.INPUT_AUTO_L2_PANEL, AutoProcess::levelTwoPanel);
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT,
        // Constants.INPUT_AUTO_L3_PANEL, AutoProcess::levelThreePanel);
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT,
        // Constants.INPUT_AUTO_L1_CARGO, AutoProcess::levelOneCargo);
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT,
        // Constants.INPUT_AUTO_L2_CARGO, AutoProcess::levelTwoCargo);
        // TRCDriveInput.bindButton(Constants.INPUT_DRIVER_PORT,
        // Constants.INPUT_AUTO_L3_CARGO, AutoProcess::levelThreeCargo);

        // Setup: Input: Button Bindings: Gunner Functions
        TRCDriveInput.bindButtonPress(Constants.INPUT_GUNNER_PORT, Constants.INPUT_LIFT_ELEVATE_BUTTON,
                lift::driveForward);
        TRCDriveInput.bindButtonPress(Constants.INPUT_GUNNER_PORT, Constants.INPUT_LIFT_DESCEND_BUTTON,
                lift::driveReverse);
        TRCDriveInput.bindButtonAbsence(Constants.INPUT_GUNNER_PORT, Constants.INPUT_LIFT_BUTTONS, lift::fullStop);

        TRCDriveInput.bindButtonPress(Constants.INPUT_GUNNER_PORT, Constants.INPUT_POKIE_EXTEND_BUTTON,
                pokie::fullOpen);
        TRCDriveInput.bindButtonPress(Constants.INPUT_GUNNER_PORT, Constants.INPUT_POKIE_RETRACT_BUTTON,
                pokie::fullClose);

        TRCDriveInput.bindButtonPress(Constants.INPUT_GUNNER_PORT, Constants.INPUT_GRABBER_INTAKE_BUTTON,
                grabber::driveForward);
        TRCDriveInput.bindButtonPress(Constants.INPUT_GUNNER_PORT, Constants.INPUT_GRABBER_EXPEL_BUTTON,
                grabber::driveReverse);
        TRCDriveInput.bindButtonAbsence(Constants.INPUT_GUNNER_PORT, Constants.INPUT_GRABBER_BUTTONS,
                grabber::fullStop);

        TRCDriveInput.bindButtonPress(Constants.INPUT_GUNNER_PORT, Constants.INPUT_ARM_UP_BUTTON, arm::driveReverse);
        TRCDriveInput.bindButtonPress(Constants.INPUT_GUNNER_PORT, Constants.INPUT_ARM_DOWN_BUTTON, arm::driveForward);
        TRCDriveInput.bindButtonAbsence(Constants.INPUT_GUNNER_PORT, Constants.INPUT_ARM_BUTTONS, arm::fullStop);

        // TRCDriveInput.bindButtonPress(Constants.INPUT_GUNNER_PORT, 10,
        // AutoAlign::alignWithFloorTape);

        // Setup: Input: Button Bindings: Driver Functions
        TRCDriveInput.bindButtonPress(Constants.INPUT_DRIVER_PORT, Constants.INPUT_DRIVE_SLOW, drive::setSlowOn);
        TRCDriveInput.bindButtonAbsence(Constants.INPUT_DRIVER_PORT, Constants.INPUT_DRIVE_BUTTONS, drive::setSlowOff);
    }

    /**
     * Code here will run once at the start of autonomous
     */
    @Override
    public void autonomousInit()
    {
        encoders.resetAllEncoders();
        gyro.reset();
        //AssistedControl.startCommunications();
        //AutoCargo cargo = new AutoCargo(CargoPositionType.Close, false, encoders, leftProx, rightProx);
        //cargo.run();
    }

    /**
     * Code here will run continously during autonomous
     */
    @Override
    public void autonomousPeriodic()
    {
        driveRobot();
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
        driveRobot();
    }

    public void driveRobot()
    {
        TRCDriveSync.requestChangeState(DriveSyncState.Teleop);
        // Check all inputs
        TRCDriveInput.checkButtonBindings();
        // And drive the robot
        TRCDriveParams input = TRCDriveInput.getStickDriveParams(Constants.INPUT_DRIVER_PORT);
        try
        {
            TRCDriveSync.assertTeleop();
            drive.driveCartesian(input);
        }
        catch (AssertionError e)
        {
            // System.out.println(e);
        }

        TRCNetworkData.updateDataPoint("Encoder Output", encoders.getAverageDistanceTraveled(DirectionType.ForwardBackward));
        TRCNetworkData.updateDataPoint("Gyro", gyro.getAngle());
        TRCNetworkData.updateDataPoint("Arm", arm.getArmPos());
        TRCNetworkData.updateDataPoint("Left Proximity", AutoAlign.calculateUltrasonicDistance(leftProx.getVoltage()));
        TRCNetworkData.updateDataPoint("Right Proximity", AutoAlign.calculateUltrasonicDistance(rightProx.getVoltage()));
    }

    public static void main(String... args) throws InterruptedException
    {
        RobotBase.startRobot(Robot::new);
        // request data from the Arduin
    }
}