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
    // TURRET CONSTANTS
    // ................................................................................................
    public static final double HOOD_MAX_POSITION = 23.5;
    public static final double HOOD_MIN_POSITION = -10;
    // old old 3.928568124771118 with 10,000 rpm
    // 2.500006675720215 old
    public static final double HOOD_CONTROL_PANEL_POSITION = 2.90;
    public static final double HOOD_LINE_POSITION = 0;
    public static final double HOOD_TRENCH_POSITION = 2.2857134491205215;
    public static final double HOOD_Kp = 0.15;
    public static final double HOOD_MAXOUTPUT = 0.1;
    public static final double HOOD_MINOUTPUT = -0.1;

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // TURRET CONSTANTS
    // ................................................................................................

    public static final double TURRET_YAW_RATE = 0.1;

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // CLIMBING CONSTANTS
    // ................................................................................................

    public static final double CLIMB_SPEED = 0.75;

    public static final double SHOOTING_MAX = 0.8;
    public static final double SHOOTING_MIN = -0.1;
    public static final double SHOOTING_Kp = 1.0;
    public static final double SHOOTING_Ki = 0; // .001
    public static final double SHOOTING_Kd = 0; // 70.5
    public static final double SHOOTING_Kf = 0.0639; //
    public static final double SHOOTING_Kizone = 0; // 50
    public static final double SHOOTING_TARGET_RPM = 8000;
    public static final double SHOOTING_UNITS_PER_REV = 820;

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // INTAKE CONSTANTS
    // ................................................................................................

    public static final double INTAKE_SPEED = 1.0;
    public static final double INTAKE_BACK_CONVEYOR_THRESHOLD = 200;
    public static final double FRONT_CONVEYOR_SPEED = 1.0;
    public static final double BACK_CONVEYOR_SPEED = 1;

    // 10.8
    public static final double Kp = 3;
    public static final double Kd = 0;
    public static final double Ks = 0.413;
    public static final double Kv = 1.87;
    public static final double Ka = 0.494;
    public static final double TRACK_WIDTH = .61;

    public static final boolean kGyroReversed = true;

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // COLOR CONSTANTS
    // ................................................................................................

    public final static double COLOR_RED_THRESHOLD = 7000;
    public final static double COLOR_GREEN_THRESHOLD = 13000;
    public final static double COLOR_ALPHA_THRESHOLD = 29192;
}