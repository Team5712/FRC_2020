package frc.robot;

import frc.Const;

/**
 * RoboMath
 */
public class RoboMath {

    public static double ticksToMeters(double ticks) {
        return ticks * Const.TICKS_TO_METERS_RATIO;
    }
}