package org.usfirst.frc.team6500.robot;


import java.util.ArrayList;

import org.usfirst.frc.team6500.trc.systems.TRCDirectionalSystem;
import org.usfirst.frc.team6500.trc.util.TRCTypes.SpeedControllerType;

import edu.wpi.first.wpilibj.DigitalInput;


public class Lift extends TRCDirectionalSystem
{
    private static boolean isReady = false;
    private static ArrayList<DigitalInput> liftSwitches;
    private static int level = 0;
    private static int targetLevel = 0;

    public Lift(int[] motorPorts, SpeedControllerType[] motorTypes)
    {
        super(motorPorts, motorTypes, true, Constants.LIFT_SPEED_UP, Constants.LIFT_SPEED_DOWN);

        liftSwitches = new ArrayList<DigitalInput>();
        for (int i = 0; i < Constants.LIFT_SWITCH_PORTS.length; i++)
        {
            liftSwitches.add(i, new DigitalInput(Constants.LIFT_SWITCH_PORTS[i]));
        }

        isReady = true;
    }


    public void setTargetLevel(int newLevel)
    {
        targetLevel = newLevel;
    }

    @Override
    public void driveForward()
    {
        if (!isReady) { return; }

        while (!liftSwitches.get(targetLevel).get())
        {
            if (targetLevel > level)
            {
                super.driveForward();
            }
            else if (targetLevel < level)
            {
                this.driveReverse();
            }
        }

        this.fullStop();
        level = targetLevel;
    }

    public void liftToLevel(int newLevel)
    {
        setTargetLevel(newLevel);
        driveForward();
    }
}