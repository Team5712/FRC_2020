package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Vision
 */
public class Vision {

    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-two");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    public double[] getTurnValues() {
        double x = tx.getDouble(0.0);
        float Kp = -0.03f; // Proportional control constant
        double min_command = 0.2;

        double steering_adjust = 0;

        if (x > .5) {
            steering_adjust = Kp * x - min_command;
        } else if (x < -.5) {
            steering_adjust = Kp * x + min_command;
        }

        return new double[] { -steering_adjust, -steering_adjust };
    }

    public static double getDistanceToTarget() {
        double height = getNumber("ty");

        if (height == 0) {
            System.out.println("TARGET NOT FOUND");
            return 0.0;
        }

        // TODO: redo this
        // return (height * -3.98374) + 111.198;
        return (0.16976 * Math.pow(height, 2)) + (-6.85384 * height) + 117.413;
    }

    public static double getDesiredHoodAngleToTarget(double distanceToTarget) {
        
        // TODO: noew
        return 0;
    }


    public void setPipeLine(int pipeline) {
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(<value>);
    }

    public static double getWidth() {
        return table.getEntry("thor").getDouble(0.0);
    }
    
    public static double getNumber(String number) {
        return table.getEntry(number).getDouble(0.0);
    }

    public static double getHeight() {
        return table.getEntry("tvert").getDouble(0.0);
    }

    public static double getXOffset() {
        return table.getEntry("tx").getDouble(0.0);
    }
}