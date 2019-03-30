package org.usfirst.frc.team6500.robot;


import org.usfirst.frc.team6500.robot.Constants.*;
import org.usfirst.frc.team6500.trc.auto.TRCAutoRoute;
import org.usfirst.frc.team6500.trc.auto.TRCDrivePID;
import org.usfirst.frc.team6500.trc.auto.TRCDriveSync;
import org.usfirst.frc.team6500.trc.util.TRCTypes.*;

import org.usfirst.frc.team6500.trc.wrappers.sensors.TRCEncoderSet;

import edu.wpi.first.wpilibj.AnalogInput;

import org.usfirst.frc.team6500.trc.auto.TRCDirectionalSystemAction;


public class AutoCargo implements TRCAutoRoute
{
    private final double inches0 = -47.0;
    private final double inches1 = -48.0;
    private double inches2 = 164.0;
    private final double inchesA = 21.75;
    private final double rotation0 = 180.0;
    private double rotation1 = -90.0;
    private TRCEncoderSet encoders;
    private AnalogInput leftProximity, rightProximity;

    public AutoCargo(CargoPositionType cargoPos, boolean left, TRCEncoderSet encoderSet, AnalogInput leftProx, AnalogInput rightProx)
    {
        if (cargoPos == CargoPositionType.Middle)
        {
            inches2 += inchesA;
        }
        else if (cargoPos == CargoPositionType.Far)
        {
            inches2 += inchesA * 2;
        }

        if (left) { rotation1 *= -1; }

        encoders = encoderSet;
        leftProximity = leftProx;
        rightProximity = rightProx;
    }

    @Override
    public void run()
    {
        TRCDriveSync.requestChangeState(DriveSyncState.DrivePID);
        TRCDrivePID.run(DriveActionType.Forward, inches0);

        encoders.resetAllEncoders();
        TRCDrivePID.run(DriveActionType.Forward, inches1);
        TRCDrivePID.run(DriveActionType.Rotate, rotation0);

        encoders.resetAllEncoders();
        TRCDrivePID.run(DriveActionType.Forward, inches2);

        (new TRCDirectionalSystemAction("Lift", DirectionalSystemActionType.Forward, Constants.LIFT_TIME_CARGO, true)).start();
        (new TRCDirectionalSystemAction("Grabber", DirectionalSystemActionType.Reverse, Constants.GRABBER_TIME_CARGO, true)).start();
        TRCDrivePID.run(DriveActionType.Rotate, rotation1);
        
        TRCDriveSync.requestChangeState(DriveSyncState.DriveContinuous);
        AssistedControl.startCommunications();

        boolean running = true;
        while(running)
        {
            if (AutoAlign.calculateUltrasonicDistance(leftProximity.getVoltage()) <= Constants.PROXIMITY_THRESHOLD_MM && AutoAlign.calculateUltrasonicDistance(rightProximity.getVoltage()) <= Constants.PROXIMITY_THRESHOLD_MM)
            {
                running = false;
            }
        }

        AssistedControl.pauseCommunications();
        TRCDriveSync.requestChangeState(DriveSyncState.Teleop);
    }
}