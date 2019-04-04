package org.usfirst.frc.team6500.robot;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import org.usfirst.frc.team6500.trc.auto.TRCDriveContinuous;
import org.usfirst.frc.team6500.trc.auto.TRCDriveSync;
import org.usfirst.frc.team6500.trc.util.TRCNetworkData;
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

    private static int[] requestData() 
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

        int[] intInput = new int[1];
        if (noConnection)
        { intInput[0] = -2; }// intInput[1] = 0; intInput[2] = 0; }
        else
        {
            intInput[0] = (int) input[0];
            // intInput[1] = (int) input[1];
            // intInput[2] = (int) input[2];
        }
        return intInput;
    }

    private static void read() 
    {
        TRCDriveSync.requestChangeState(DriveSyncState.DriveContinuous);
        TRCDriveContinuous.startDriveContinuous(DriveContinuousActionType.Forward);
        while (true)
        {
            try
            {
                Thread.sleep(100);
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }

            int[] recvData = requestData();
            int data = recvData[0];
            int threshold = 16;
            //System.out.println(data);
            //TRCNetworkData.updateDataPoint("LFC", data);
            
            if (data < 0) { continue; }

            boolean lineVisible = false;
            if (data >= threshold) { lineVisible = true; data -= threshold; }
            TRCNetworkData.updateDataPoint("LFC", data);
            TRCNetworkData.updateDataPoint("Line Visible?", lineVisible);

            if (!isReading.get()) { continue; }

            try
            {
                TRCDriveSync.assertDriveContinuous();

                DriveContinuousActionType actionType = DriveContinuousActionType.values()[0];
                try
                {
                    actionType = DriveContinuousActionType.values()[data];
                }
                catch (Exception e)
                {
                    //System.out.println(e);
                }

                TRCDriveContinuous.setDriveContinuousActionType(actionType);
            }
            catch (AssertionError e)
            {
                // System.out.println(e);
            }
        }
    }

    public static boolean getIsReading()
    {
        return isReading.get();
    }
}