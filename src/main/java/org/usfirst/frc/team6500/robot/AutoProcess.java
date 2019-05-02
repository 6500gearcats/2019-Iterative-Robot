package org.usfirst.frc.team6500.robot;


import org.usfirst.frc.team6500.trc.auto.TRCDirectionalSystemAction;
import org.usfirst.frc.team6500.trc.auto.TRCDrivePID;
import org.usfirst.frc.team6500.trc.auto.TRCPneumaticSystemAction;
import org.usfirst.frc.team6500.trc.util.TRCTypes.DirectionalSystemActionType;
import org.usfirst.frc.team6500.trc.util.TRCTypes.DriveActionType;
import org.usfirst.frc.team6500.trc.util.TRCTypes.PneumaticSystemActionType;
import org.usfirst.frc.team6500.trc.wrappers.systems.drives.TRCMecanumDrive;


public class AutoProcess
{
    public static TRCMecanumDrive drive;
    public static Lift lift;
    public static Grabber grabber;
    public static Arm arm;

    public static void setupSystems(TRCMecanumDrive nDrive, Lift nLift, Grabber nGrabber, Arm nArm)
    {
        drive = nDrive;
        lift = nLift;
        grabber = nGrabber;
        arm = nArm;
    }


//     public static void grabPanel()
//     {
//         TRCDrivePID.run(DriveActionType.Forward, 2.5);
//         TRCDrivePID.run(DriveActionType.Forward, -1.0);
//     }

//     public static void ejectPanel()
//     {
//         TRCPneumaticSystemAction ejectHatch = new TRCPneumaticSystemAction("Pokie", PneumaticSystemActionType.Open, 0.25, false);
//         TRCPneumaticSystemAction resetPokies = new TRCPneumaticSystemAction("Pokie", PneumaticSystemActionType.Close, 0.1, false);

//         ejectHatch.start();
//         resetPokies.start();

//         TRCDrivePID.run(DriveActionType.Forward, -1.0);
//     }

//     public static void intakeCargo()
//     {
//         grabber.driveReverse();
//     }

    public static void ejectCargo()
    {
        TRCDirectionalSystemAction ejectCargo = new TRCDirectionalSystemAction("Grabber", DirectionalSystemActionType.Forward, 2.0, false);

        ejectCargo.start();
    }

//     // public static void armUp()
//     // {
//     //     arm.armToUp();
//     // }

//     // public static void armDown()
//     // {
//     //     arm.armToDown();
//     // }

    public static void autoCargo()
    {
        arm.armToHatch();
        //lift.liftToLevel(1);
        arm.armToHatch();
        TRCDrivePID.run(DriveActionType.Forward, 10);
        ejectCargo();
        TRCDrivePID.run(DriveActionType.Forward, -10);
        //lift.returnToPrevious();
    }


//     public static void obtainPanel()
//     {
//         AutoAlign.align();
//         grabPanel();
//     }

//     public static void obtainCargo()
//     {
//         AutoAlign.align();
//         grabPanel();
//     }

//     public static void levelOnePanel()
//     {
//         AutoAlign.align();
//         lift.liftToLevel(1);
//         ejectPanel();
//     }

//     public static void levelTwoPanel()
//     {
//         AutoAlign.align();
//         lift.liftToLevel(2);
//         ejectPanel();
//     }

//     public static void levelThreePanel()
//     {
//         AutoAlign.align();
//         lift.liftToLevel(3);
//         ejectPanel();
//     }

//     public static void levelOneCargo()
//     {
//         AutoAlign.align();
//         lift.liftToLevel(2);
//         ejectCargo();
//     }

//     public static void levelTwoCargo()
//     {
//         AutoAlign.align();
//         lift.liftToLevel(3);
//         ejectCargo();
//     }

//     public static void levelThreeCargo()
//     {
//         AutoAlign.align();
//         lift.liftToLevel(4);
//         ejectCargo();
//     }
}