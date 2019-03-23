package org.usfirst.frc.team6500.robot;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc.team6500.trc.systems.TRCDirectionalSystem;
import org.usfirst.frc.team6500.trc.util.TRCTypes.SpeedControllerType;


public class Grabber extends TRCDirectionalSystem
{
    public Grabber(int[] motorPorts, SpeedControllerType[] motorTypes)
    {
        super(motorPorts, motorTypes, true, Constants.GRABBER_SPEED_EXPEL, Constants.GRABBER_SPEED_INTAKE);
    }

    @Override
    public void driveReverse()
    {
        WPI_TalonSRX grabLeft = (WPI_TalonSRX) this.outputMotors.get(this.outputMotors.keySet().toArray()[0]);

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