package org.usfirst.frc.team6500.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc.team6500.trc.systems.TRCDirectionalSystem;
import org.usfirst.frc.team6500.trc.util.TRCTypes.SpeedControllerType;


public class Lift extends TRCDirectionalSystem
{
    private static TalonSRX liftLeft, liftRight;
    private static WPI_TalonSRX wLiftLeft, wLiftRight;
    private static boolean autoing;
    private static int previousPos;

    public Lift(int[] motorPorts, SpeedControllerType[] motorTypes)
    {
        super(motorPorts, motorTypes, false, Constants.LIFT_SPEED_UP, Constants.LIFT_SPEED_DOWN);

        liftLeft = new TalonSRX((Integer) this.outputMotors.keySet().toArray()[1]);
        liftRight = new TalonSRX((Integer) this.outputMotors.keySet().toArray()[0]);
        wLiftLeft = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[1]);
        wLiftRight = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[0]);

        liftRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        liftRight.setSensorPhase(false);
        liftLeft.follow(liftRight);

        liftRight.configNominalOutputForward(0.0, 30);
        liftRight.configNominalOutputReverse(0.0, 30);
        liftRight.configPeakOutputForward(1.0, 30);
        liftRight.configPeakOutputReverse(-1.0, 30);

        liftRight.config_kF(0, 1, 30);
        liftRight.config_kP(0, 1, 30);
        liftRight.config_kI(0, 0, 30);
        liftRight.config_kD(0, 0, 30);

        autoing = false;
    }


    @Override
    public void driveForward()
    {
        wLiftLeft.enableCurrentLimit(false);
        wLiftRight.enableCurrentLimit(false);
        wLiftLeft.set(ControlMode.PercentOutput, 0.0);
        wLiftRight.set(ControlMode.PercentOutput, 0.0);

        super.driveForward();
    }

    @Override
    public void driveReverse()
    {
        wLiftLeft.enableCurrentLimit(false);
        wLiftRight.enableCurrentLimit(false);
        wLiftLeft.set(ControlMode.PercentOutput, 0.0);
        wLiftRight.set(ControlMode.PercentOutput, 0.0);

        super.driveReverse();
    }

    @Override
    public void fullStop()
    {
        if (autoing) { return; }

        wLiftLeft.configContinuousCurrentLimit(Constants.LIFT_MAX_STALL_CURRENT);
        wLiftRight.configContinuousCurrentLimit(Constants.LIFT_MAX_STALL_CURRENT);
        wLiftLeft.enableCurrentLimit(true);
        wLiftRight.enableCurrentLimit(true);
        wLiftRight.set(ControlMode.Velocity, 0.0);
        wLiftLeft.set(ControlMode.Follower, (Integer) this.outputMotors.keySet().toArray()[1]);
    }

    // public void liftToLevel(int newLevel)
    // {
    //     previousPos = wLiftRight.getSelectedSensorPosition();
    //     autoing = true;

    //     while (Math.abs(wLiftRight.getSelectedSensorPosition() - Constants.LIFT_TARGET_HEIGHTS[newLevel]) > 50)
    //     {
    //         wLiftLeft.enableCurrentLimit(false);
    //         wLiftRight.enableCurrentLimit(false);
    //         wLiftLeft.set(ControlMode.PercentOutput, Constants.LIFT_SPEED_UP);
    //         wLiftRight.set(ControlMode.PercentOutput, Constants.LIFT_SPEED_UP);
    //     }

    //     autoing = false;
    // }

    // public void returnToPrevious()
    // {
    //     autoing = true;
    //     while (Math.abs(wLiftRight.getSelectedSensorPosition() - previousPos) > 200)
    //     {
    //         wLiftLeft.enableCurrentLimit(false);
    //         wLiftRight.enableCurrentLimit(false);
    //         wLiftLeft.set(ControlMode.PercentOutput, Constants.LIFT_SPEED_DOWN);
    //         wLiftRight.set(ControlMode.PercentOutput, Constants.LIFT_SPEED_DOWN);
    //     }
    //     autoing = false;
    // }
}