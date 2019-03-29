package org.usfirst.frc.team6500.robot;


import java.util.concurrent.atomic.AtomicBoolean;

import org.usfirst.frc.team6500.trc.auto.TRCDriveContinuous;
import org.usfirst.frc.team6500.trc.auto.TRCDriveSync;
import org.usfirst.frc.team6500.trc.util.TRCTypes.*;

import edu.wpi.first.wpilibj.I2C;


public class AssistedControl extends I2C 
{
    private Thread readThread;
    private AtomicBoolean isReading = new AtomicBoolean(false);

    AssistedControl(int address) 
    {
        super(I2C.Port.kOnboard, address);
    }

    public void startCommunications() 
    {
        if (readThread != null) { resumeCommunications(); }

        readThread = new Thread(this::read);
        readThread.setName("Assisted Control Thread");
        readThread.start();
        isReading.set(true);
    }

    public void resumeCommunications() 
    {
        this.isReading.set(true);
    }

    public void pauseCommunications() 
    {
        this.isReading.set(false);
    }

    private int requestAction() 
    {
        byte[] input = new byte[1];
        boolean noConnection = readOnly(input, 1);
        if (noConnection)
            return -2;
        return (int) input[0];
    }

    private void read() 
    {
        TRCDriveSync.requestChangeState(DriveSyncState.DriveContinuous);
        while (true)
        {
            if (!this.isReading.get()) { continue; }

            int action = requestAction();
            DriveContinuousActionType actionType = DriveContinuousActionType.values()[action];
            System.out.println("Number " + action);
            try
            {
                Thread.sleep(1000);
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
    }
}