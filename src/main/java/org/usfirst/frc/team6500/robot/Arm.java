package org.usfirst.frc.team6500.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc.team6500.trc.systems.TRCDirectionalSystem;
import org.usfirst.frc.team6500.trc.util.TRCTypes.SpeedControllerType;
import org.usfirst.frc.team6500.trc.wrappers.sensors.TRCTalonEncoder;


public class Arm extends TRCDirectionalSystem
{
    private static boolean isReady = false;
    public static TRCTalonEncoder encoder;
    private static WPI_TalonSRX wArmTalon;
    private static TalonSRX armTalon;

    public Arm (int[] motorPorts, SpeedControllerType[] motorType)
    {
        super(motorPorts, motorType, true, Constants.ARM_SPEED_UP, Constants.ARM_SPEED_DOWN);
        encoder = new TRCTalonEncoder(Constants.ARM_MOTOR, Constants.ARM_DISTANCE_PER_PULSE, false);
        encoder.reset();

        armTalon = new TalonSRX((Integer) this.outputMotors.keySet().toArray()[0]);
        wArmTalon = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[0]);

        armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        armTalon.setSensorPhase(false);

        armTalon.configNominalOutputForward(0.0, 30);
        armTalon.configNominalOutputReverse(0.0, 30);
        armTalon.configPeakOutputForward(1.0, 30);
        armTalon.configPeakOutputReverse(-1.0, 30);

        armTalon.config_kF(0, 1, 30);
        armTalon.config_kP(0, 1, 30);
        armTalon.config_kI(0, 0, 30);
        armTalon.config_kD(0, 0, 30);

        isReady = true;
    }


    public double getArmPos()
    {
        return armTalon.getSelectedSensorPosition();
    }


    public void armToHatch()
    {
        if (!isReady) { return; }

        while (getArmPos() < Constants.ARM_POSITION_HATCH)
        {
            this.driveReverse();
        }

        this.fullStop();
    }

    public void atEasePrivateArm()
    {
        if (!isReady) { return; }

        while (getArmPos() > Constants.ARM_POSITION_EASE)
        {
            this.driveForward();
        }

        this.fullStop();
    }

    public boolean checkOverdistance(WPI_TalonSRX talon)
    {
        if (talon.getSelectedSensorPosition() > Constants.ARM_POSITION_UP)
        {
            talon.configContinuousCurrentLimit(Constants.ARM_MAX_STALL_CURRENT);
            talon.enableCurrentLimit(true);
            talon.set(ControlMode.Velocity, -500.0);
            return true;
        }
        else if (talon.getSelectedSensorPosition() < Constants.ARM_POSITION_DOWN)
        {
            talon.configContinuousCurrentLimit(Constants.ARM_MAX_STALL_CURRENT);
            talon.enableCurrentLimit(true);
            talon.set(ControlMode.Velocity, 500.0);
            return true;
        }

        return false;
    }

    @Override
    public void driveForward()
    {
        if (checkOverdistance(wArmTalon)) { return; }

        wArmTalon.enableCurrentLimit(false);

        super.driveForward();
    }

    @Override
    public void driveReverse()
    {
        if (checkOverdistance(wArmTalon)) { return; }

        wArmTalon.enableCurrentLimit(false);

        super.driveReverse();
    }

    @Override
    public void fullStop()
    {
        if (checkOverdistance(wArmTalon)) { return; }

        wArmTalon.configContinuousCurrentLimit(Constants.ARM_MAX_STALL_CURRENT);
        wArmTalon.enableCurrentLimit(true);
        wArmTalon.set(ControlMode.Velocity, 0.0);
    }
}
