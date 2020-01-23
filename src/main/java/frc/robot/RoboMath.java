package frc.robot;

import frc.Const;

/**
 * RoboMath
 */
public class RoboMath {

    // public static double toInches(double ticks) {
    //     return ticks * Const.TICKS_TO_INCHES_RATIO;
    // }

    public static double ticksToMeters(double ticks) {
        return ticks * Const.TICKS_TO_METERS_RATIO;
    }
}