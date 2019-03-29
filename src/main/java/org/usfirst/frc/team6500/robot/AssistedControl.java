package org.usfirst.frc.team6500.robot;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import org.usfirst.frc.team6500.trc.auto.TRCDriveContinuous;
import org.usfirst.frc.team6500.trc.auto.TRCDriveSync;
import org.usfirst.frc.team6500.trc.util.TRCTypes.*;

import edu.wpi.first.wpilibj.I2C;


public class AssistedControl 
{
    private static Thread readThread;
    private static I2C i2c;

    private static AtomicInteger x0;
    private static AtomicInteger y0;
    private static AtomicInteger x1;
    private static AtomicInteger y1;
    private static AtomicBoolean isReading = new AtomicBoolean(false);

    public static void initializeAssistedControl(int address) 
    {
        i2c = new I2C(I2C.Port.kOnboard, address);
    }

    public static void startCommunications() 
    {
        if (readThread != null) { resumeCommunications(); }

        readThread = new Thread(AssistedControl::read);
        readThread.setName("Assisted Control Thread");
        readThread.start();
        isReading.set(true);
    }

    public static void resumeCommunications() 
    {
        isReading.set(true);
    }

    public static void pauseCommunications() 
    {
        isReading.set(false);
    }

    private static int requestAction() 
    {
        // ByteBuffer buffer = ByteBuffer.allocate(4);
    		
        // i2C.read(8, 4, buffer);
        // int[] values = buffer.asIntBuffer().array();
        //Only using x0 in the above code, may want to cut out rest. I left them in just in case.
        // x0.set(values[0]);
        // y0.set(values[1]);
        // x1.set(values[2]);
        // y1.set(values[3]);

        byte[] input = new byte[1];
        boolean noConnection = i2c.readOnly(input, 1);
        if (noConnection)
            return -2;
        return (int) input[0];
    }

    private static void read() 
    {
        TRCDriveSync.requestChangeState(DriveSyncState.DriveContinuous);
        TRCDriveContinuous.startDriveContinuous(DriveContinuousActionType.Forward);
        while (true)
        {
            if (!isReading.get()) { continue; }

            int action = requestAction();
            DriveContinuousActionType actionType = DriveContinuousActionType.values()[action];
            System.out.println("Number " + action);
            TRCDriveContinuous.setDriveContinuousActionType(actionType);
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