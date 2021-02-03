package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.Const;

/**
 * Vision
 */
public class Vision {

    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-two");
    
    public double[] getTurnValues() {
        enableLEDS();

        double x = table.getEntry("tx").getDouble(0.0);
        float Kp = -0.035f; // Proportional control constant
        double min_command = 0.2;

        double steering_adjust = 0;

        if (x > .5) {
            steering_adjust = Kp * x - min_command;
        } else if (x < -.5) {
            steering_adjust = Kp * x + min_command;
        }

        return new double[] { -steering_adjust, -steering_adjust };
    }

    /**
     * 
     * @return the distance to the found target in inches
     */
    public static double getDistanceToTarget() {
        enableLEDS();

        double height = getNumber("ty");

        if (height == 0) {
            System.out.println("TARGET NOT FOUND");
            return 0.0;
        }

        // TODO: redo this
        // return (height * -3.98374) + 111.198;
        return (0.16976 * Math.pow(height, 2)) + (-6.85384 * height) + 117.413;
    }

    public static double getDesiredHoodAngleToTarget() {
        // TODO: check adjustment
        return (getNumber("ty") * -0.66303) - 1.8563;
    }

    public static boolean isWithinLateralRange() {

        // distance in inches
        double distance = getDistanceToTarget();
        double acceptableAngularOffset = Math.toDegrees(Math.tanh((Const.FIELD_OUTER_PORT_WIDTH / (2 * distance))));   
        
        // System.out.println("distance " + distance + " acceptable offset " + acceptableAngularOffset);

        if(Math.abs(getNumber("tx")) < acceptableAngularOffset) {
            return true;
        } else {
            return false;
        }
    }

    public static void setNumber(String entry, int number) {
        table.getEntry(entry).setNumber(number);
    }

    public static void disableLEDS() {
        setNumber("ledMode", 1);
    }

    public static void enableLEDS() {
        setNumber("ledMode", 3);
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