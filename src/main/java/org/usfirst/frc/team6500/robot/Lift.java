package org.usfirst.frc.team6500.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc.team6500.trc.systems.TRCDirectionalSystem;
import org.usfirst.frc.team6500.trc.util.TRCTypes.SpeedControllerType;


public class Lift extends TRCDirectionalSystem
{
    public Lift(int[] motorPorts, SpeedControllerType[] motorTypes)
    {
        super(motorPorts, motorTypes, false, Constants.LIFT_SPEED_UP, Constants.LIFT_SPEED_DOWN);

        TalonSRX liftLeft = new TalonSRX((Integer) this.outputMotors.keySet().toArray()[1]);
        TalonSRX liftRight = new TalonSRX((Integer) this.outputMotors.keySet().toArray()[0]);

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
    }


    @Override
    public void driveForward()
    {
        WPI_TalonSRX liftLeft = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[1]);
        WPI_TalonSRX liftRight = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[0]);

        liftLeft.enableCurrentLimit(false);
        liftRight.enableCurrentLimit(false);
        liftLeft.set(ControlMode.PercentOutput, 0.0);
        liftRight.set(ControlMode.PercentOutput, 0.0);

        super.driveForward();
    }

    @Override
    public void driveReverse()
    {
        WPI_TalonSRX liftLeft = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[1]);
        WPI_TalonSRX liftRight = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[0]);

        liftLeft.enableCurrentLimit(false);
        liftRight.enableCurrentLimit(false);
        liftLeft.set(ControlMode.PercentOutput, 0.0);
        liftRight.set(ControlMode.PercentOutput, 0.0);

        super.driveReverse();
    }

    @Override
    public void fullStop()
    {
        WPI_TalonSRX liftLeft = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[1]);
        WPI_TalonSRX liftRight = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[0]);

        liftLeft.configContinuousCurrentLimit(Constants.LIFT_MAX_STALL_CURRENT);
        liftRight.configContinuousCurrentLimit(Constants.LIFT_MAX_STALL_CURRENT);
        liftLeft.enableCurrentLimit(true);
        liftRight.enableCurrentLimit(true);
        liftRight.set(ControlMode.Velocity, 0.0);
        liftLeft.set(ControlMode.Follower, (Integer) this.outputMotors.keySet().toArray()[1]);
    }

    public void liftToLevel(int newLevel)
    {
        WPI_TalonSRX liftLeft = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[1]);
        WPI_TalonSRX liftRight = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[0]);
        
        liftRight.set(ControlMode.Position, Constants.LIFT_TARGET_HEIGHTS[newLevel]);
        liftLeft.set(ControlMode.Follower, (Integer) this.outputMotors.keySet().toArray()[1]);

        while (liftRight.getSelectedSensorPosition() - liftRight.getClosedLoopTarget() < 50)
        {
            
        }
    }
}