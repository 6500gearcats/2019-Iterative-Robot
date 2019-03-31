package org.usfirst.frc.team6500.robot;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc.team6500.trc.systems.TRCDirectionalSystem;
import org.usfirst.frc.team6500.trc.util.TRCTypes.SpeedControllerType;


public class Grabber extends TRCDirectionalSystem
{
    private static WPI_TalonSRX grabLeft;

    public Grabber(int[] motorPorts, SpeedControllerType[] motorTypes)
    {
        super(motorPorts, motorTypes, true, Constants.GRABBER_SPEED_EXPEL, Constants.GRABBER_SPEED_INTAKE);
        grabLeft = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[0]);
    }

    @Override
    public void driveReverse()
    {
        if (!grabLeft.getSensorCollection().isRevLimitSwitchClosed())
        {
            super.driveReverse();
        }
        else
        {
            super.fullStop();
        }
    }
}