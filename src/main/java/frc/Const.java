package frc;

/**
 * Const
 */
public class Const {

    // public static final double TRACK_WIDTH_INCHES = 24.5;

    public static final double WHEEL_RADIUS = .0762;

    // public static final double TICKS_TO_INCHES_RATIO = (2 * Math.PI * 2) / 512;
    public static final double TICKS_TO_METERS_RATIO = (2 * Math.PI * WHEEL_RADIUS) / 7;

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // SHOOTING CONSTANTS
    // ................................................................................................

    //public static final double NEO_SHOOTING_TARGET_RPM = 3200;
    // public static final double SHOOTING_MAX_RPM = 4000;
    

    public static final double HOOD_BACK = 0.53;
    public static final double HOOD_FRONT = 0.51;
    public static final double HOOD_Kp = 0.00001;
    public static final double HOOD_MAXOUTPUT = 0.1;
    public static final double HOOD_MINOUTPUT = -0.1;
    

    
    // public static final double NEO_SHOOTING_Kp = 0.001;
    // public static final double NEO_SHOOTING_Kd = 0.0000008;
    public static final double SHOOTING_Kp = 0.4;
    public static final double SHOOTING_Kd = 70.5; //4.5
    public static final double SHOOTING_Kf = 0.0639;
    public static final double SHOOTING_Ki = .001; //.001
    public static final double SHOOTING_Kizone = 80; //50
    public static final double SHOOTING_TARGET_RPM = 3000;
    public static final double SHOOTING_UNITS_PER_REV = 820;

    // TODO: prefix these


    //Programming chassis
    // public static final double Kp = 6.5; 
    // public static final double Kd = 0.0;
    // public static final double Ks = 0.772;
    // public static final double Kv = 3.88;
    // public static final double Ka = 0.321;
    //public static final double TRACK_WIDTH = .77259721916115;
    public static final double Kp = 3.6; 
    public static final double Kd = 0.0;
    public static final double Ks = 0.276;
    public static final double Kv = 1.79;
    public static final double Ka = 0.322;
    public static final double TRACK_WIDTH = .61;

    public static final boolean kGyroReversed = true;
}