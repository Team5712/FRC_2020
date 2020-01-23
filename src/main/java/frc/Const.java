package frc;

/**
 * Const
 */
public class Const {

    // public static final double TRACK_WIDTH_INCHES = 24.5;

    public static final double WHEEL_RADIUS = .0508;

    // public static final double TICKS_TO_INCHES_RATIO = (2 * Math.PI * 2) / 512;
    public static final double TICKS_TO_METERS_RATIO = (2 * Math.PI * WHEEL_RADIUS) / 512;

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // SHOOTING CONSTANTS
    // ................................................................................................

    public static final double SHOOTING_TARGET_RPM = 3200;
    public static final double SHOOTING_MAX_RPM = 4000;

    
    public static final double SHOOTING_Kp = 0.001;
    public static final double SHOOTING_Kd = 0.0000008;


    // TODO: prefix these

    public static final double Kp = 6.5; 
    public static final double Kd = 0.0;

    public static final double Ks = 0.772;
    public static final double Kv = 3.88;
    public static final double Ka = 0.321;

    public static final double TRACK_WIDTH = .77259721916115;

    public static final boolean kGyroReversed = true;
}