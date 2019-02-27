package org.usfirst.frc.team6500.robot;

import org.usfirst.frc.team6500.trc.util.TRCNetworkData;
import org.usfirst.frc.team6500.trc.wrappers.systems.drives.TRCMecanumDrive;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AnalogInput;

public class AutoAlign
{
    private static TRCMecanumDrive drive;
    private static AnalogInput leftProx;
    private static AnalogInput rightProx;
    private static NetworkTable visionTable = null;
    private static boolean isReady = false;

    public static void setupAlignment(TRCMecanumDrive robotDrive, AnalogInput leftSensor, AnalogInput rightSensor)
    {
        drive = robotDrive;
        leftProx = leftSensor;
        rightProx = rightSensor;
        visionTable = TRCNetworkData.getVisionTable();

        isReady = true;
    }


    public static void richard(double distance, double angle)
    {
        return;
    }

    public static void alignWithVisionTargets()
    {
        if (!isReady) { return; }

        boolean running = true;

        while (running)
        {
            double distance = visionTable.getEntry("distance").getDouble(0.0);
            double angle = visionTable.getEntry("angle").getDouble(0.0);

            richard(distance, angle);
        }
    }

    public static double calculateUltrasonicDistance(double voltage) // See https://www.maxbotix.com/ultrasonic-sensor-hrlv%E2%80%91maxsonar%E2%80%91ez-guide-158
    {
        double Vcc = 5.0;          // Input Voltage
        double Vm = voltage;       // Measured Voltage

        double Vi = Vcc / 1024;    // Volts per 5mm
        double Ri = 5 * (Vm / Vi); // Range in mm

        return Ri;
    }

    public static void alignWithFloorTape()
    {
        if (!isReady) { return; }
        visionTable.getEntry("mode").setString("Near");

        boolean running = true;

        while (running)
        {
            String instruction = visionTable.getEntry("instruction").getString("");

            if (instruction.equals("MF"))      // Move Forward
            {
                drive.driveCartesian(Constants.SPEED_AUTO_LINE, 0.0, 0.0);
            }
            
            else if (instruction.equals("ML")) // Move Left
            {
                drive.driveCartesian(0.0, Constants.SPEED_AUTO_LINE_STRAFE, 0.025);
            }
            else if (instruction.equals("MR")) // Move Right
            {
                drive.driveCartesian(0.0, -Constants.SPEED_AUTO_LINE_STRAFE, -0.025);
            }

            else if (instruction.equals("TL")) // Turn Left
            {
                drive.driveCartesian(0.0, 0.0, -Constants.SPEED_AUTO_LINE_TURN);
            }
            else if (instruction.equals("TR")) // Turn Right
            {
                drive.driveCartesian(0.0, 0.0, Constants.SPEED_AUTO_LINE_TURN);
            }

            TRCNetworkData.updateDataPoint("Left Proximity", AutoAlign.calculateUltrasonicDistance(leftProx.getVoltage()));
            TRCNetworkData.updateDataPoint("Right Proximity", AutoAlign.calculateUltrasonicDistance(rightProx.getVoltage()));


            if (calculateUltrasonicDistance(leftProx.getVoltage()) <= Constants.PROXIMITY_THRESHOLD_MM && calculateUltrasonicDistance(rightProx.getVoltage()) <= Constants.PROXIMITY_THRESHOLD_MM)
            {
                running = false; // If we're close enough on both sides, we're good
            }
        }

        visionTable.getEntry("mode").setString("None");
    }

    public static void align()
    {
        alignWithVisionTargets();
        alignWithFloorTape();
    }
}