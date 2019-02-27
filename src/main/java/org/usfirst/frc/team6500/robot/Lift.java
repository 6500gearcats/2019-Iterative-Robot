package org.usfirst.frc.team6500.robot;


import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc.team6500.trc.systems.TRCDirectionalSystem;
import org.usfirst.frc.team6500.trc.util.TRCTypes.SpeedControllerType;

import edu.wpi.first.wpilibj.DigitalInput;


public class Lift extends TRCDirectionalSystem
{
    private static boolean isReady = false;
    private static ArrayList<DigitalInput> liftSwitches;
    private static int level = 0;
    private static int targetLevel = 0;

    public Lift(int[] motorPorts, SpeedControllerType[] motorTypes)
    {
        super(motorPorts, motorTypes, false, Constants.LIFT_SPEED_UP, Constants.LIFT_SPEED_DOWN);

        liftSwitches = new ArrayList<DigitalInput>();
        for (int i = 0; i < Constants.LIFT_SWITCH_PORTS.length; i++)
        {
            liftSwitches.add(i, new DigitalInput(Constants.LIFT_SWITCH_PORTS[i]));
        }

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

        isReady = true;
    }


    public void setTargetLevel(int newLevel)
    {
        targetLevel = newLevel;
    }

    // @Override
    // public void driveForward()
    // {
    //     if (!isReady) { return; }

    //     while (!liftSwitches.get(targetLevel).get())
    //     {
    //         if (targetLevel > level)
    //         {
    //             super.driveForward();
    //         }
    //         else if (targetLevel < level)
    //         {
    //             this.driveReverse();
    //         }
    //     }

    //     this.fullStop();
    //     level = targetLevel;
    // }
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

        // System.out.println("Velocity: " + Double.toString(liftRight.getSelectedSensorVelocity(0)) + ", Error: " + Double.toString(liftRight.getClosedLoopError(0)));

        // super.fullStop();
    }
    // @Override
    // public void fullStop()
    // {
    //     System.out.println("Stop Lift");
    //     super.fullStop();
    // }

    // @Override
    // public void driveForward()
    // {
    //     System.out.println("Forward Lift");
    //     super.driveForward();
    // }

    // @Override
    // public void driveReverse()
    // {
    //     System.out.println("Reverse Lift");
    //     super.driveReverse();
    // }

    public void liftToLevel(int newLevel)
    {
        setTargetLevel(newLevel);
        WPI_TalonSRX liftLeft = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[1]);
        WPI_TalonSRX liftRight = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[0]);
        
        liftRight.set(ControlMode.Position, Constants.LIFT_TARGET_HEIGHTS[newLevel]);
        liftLeft.set(ControlMode.Follower, (Integer) this.outputMotors.keySet().toArray()[1]);

        while (liftRight.getSelectedSensorPosition() - liftRight.getClosedLoopTarget() < 50)
        {
            
        }
    }
}