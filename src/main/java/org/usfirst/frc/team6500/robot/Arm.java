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

    public Arm (int[] motorPorts, SpeedControllerType[] motorType)
    {
        super(motorPorts, motorType, true, Constants.ARM_SPEED_UP, Constants.ARM_SPEED_DOWN);
        encoder = new TRCTalonEncoder(Constants.ARM_MOTOR, Constants.ARM_DISTANCE_PER_PULSE, false);
        encoder.reset();

        TalonSRX armTalon = new TalonSRX((Integer) this.outputMotors.keySet().toArray()[0]);

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


    public void armToUp()
    {
        if (!isReady) { return; }

        encoder.reset();
        while (encoder.getDistance() < Constants.ARM_POSITION_UP)
        {
            this.driveForward();
        }

        this.fullStop();
    }

    public void armToDown()
    {
        if (!isReady) { return; }

        encoder.reset();
        while (encoder.getDistance() > Constants.ARM_POSITION_DOWN)
        {
            this.driveReverse();
        }
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
        WPI_TalonSRX armTalon = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[0]);
        if (checkOverdistance(armTalon)) { return; }

        armTalon.enableCurrentLimit(false);
        armTalon.set(ControlMode.PercentOutput, 0.0);

        super.driveForward();
    }

    @Override
    public void driveReverse()
    {
        WPI_TalonSRX armTalon = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[0]);
        if (checkOverdistance(armTalon)) { return; }

        armTalon.enableCurrentLimit(false);
        armTalon.set(ControlMode.PercentOutput, 0.0);

        super.driveReverse();
    }

    @Override
    public void fullStop()
    {
        WPI_TalonSRX armTalon = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[0]);
        if (checkOverdistance(armTalon)) { return; }

        armTalon.configContinuousCurrentLimit(Constants.ARM_MAX_STALL_CURRENT);
        armTalon.enableCurrentLimit(true);
        armTalon.set(ControlMode.Velocity, 0.0);
    }
}
