package org.usfirst.frc.team6500.robot;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc.team6500.trc.systems.TRCDirectionalSystem;
import org.usfirst.frc.team6500.trc.util.TRCTypes.SpeedControllerType;


public class Arm extends TRCDirectionalSystem{
    static SpeedControllerType left, right;
    private static boolean armReady;

    public Arm (int[] motorPorts, SpeedControllerType[] motorType)
    {
        super(motorPorts, motorType, true, 1.0, -0.5);
    }
    public void armToUp()
    {
    //     while(!((WPI_TalonSRX) this.outputMotors.get(0).getsensorCollection().isFwdLimitSwitchClosed()))
    //     {
    //     this.driveForward();
    //     }
    //    this.fullstop();
    }
    public void armToDown()
    {
    //     while(!((WPI_TalonSRX) this.outputMotors.get(0).getsensorCollection().isFwdLimitSwitchClosed()))
    //     {
    //     this.driveReverse();
    //     }
    //    this.fullstop();
    }
}
