package org.usfirst.frc.team6500.robot;

import java.util.ArrayList;

import edu.wpi.first.hal.PDPJNI;

public class MotorSafety
{
    private static int pdpHandle;
    private static ArrayList<ArrayList<Double>> channelVoltageHistories;

    public static void initializeMotorSafety()
    {
        pdpHandle = PDPJNI.initializePDP(Constants.PDP_CAN_ID);
        channelVoltageHistories = new ArrayList<ArrayList<Double>>();
        for (int i = 0; i < 16; i++) { channelVoltageHistories.set(i, new ArrayList<Double>()); }
    }

    public static void checkVoltages()
    {
        for (int i = 0; i < 16; i++) 
        {
            double voltage = PDPJNI.getPDPChannelCurrent((byte) i, pdpHandle);

            if (voltage > 0.25)
            {
                continue;
            }

            channelVoltageHistories.get(i).add(voltage);
        }
    }

    public static void checkLiftImbalance()
    {
        try
        {
        ArrayList<Double> channelAverages = new ArrayList<Double>();

        for (int i = 0; i < channelVoltageHistories.size(); i++)
        {
            for (int j = 0; j < channelVoltageHistories.get(i).size(); j++)
            {
                channelAverages.set(i, channelAverages.get(i) + j);
            }
            
            channelAverages.set(i, channelAverages.get(i) / channelVoltageHistories.get(i).size());
        }

        if (channelAverages.get(3) - channelAverages.get(12) > 0.1)
        {
            System.err.println("WARNING: Check lift motors!  Significant imbalanace, right motor average voltage greater by more than 0.1");
        }
        else if (channelAverages.get(12) - channelAverages.get(3) > 0.1)
        {
            System.err.println("WARNING: Check lift motors!  Significant imbalanace, left motor average voltage greater by more than 0.1");
        }
        }
        catch (Exception e)
        {
            System.out.println(e);
        }
    }
}