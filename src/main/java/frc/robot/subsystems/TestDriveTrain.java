package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Const;
import frc.robot.RoboMath;

/**
 * TestDriveTrain
 */
public class TestDriveTrain extends SubsystemBase {

    private WPI_TalonSRX leftMaster = new WPI_TalonSRX(1);
    private WPI_TalonSRX leftSlave = new WPI_TalonSRX(2);

    private WPI_TalonSRX rightMaster = new WPI_TalonSRX(5);
    private WPI_TalonSRX rightSlave = new WPI_TalonSRX(6);

    private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

    private AHRS gyro = new AHRS(Port.kMXP);

    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Const.TRACK_WIDTH);
    private Pose2d position = new Pose2d(0.0, 0.0, getHeading());

    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), position);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Const.Ks, Const.Kv, Const.Ka);

    private PIDController leftPidController = new PIDController(Const.Kp, Const.Kd, 0); // .835
    private PIDController rightPidController = new PIDController(Const.Kp, Const.Kd, 0); // .835

    public TestDriveTrain() {
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftMaster.setInverted(false);
        rightMaster.setInverted(true);

        leftMaster.setSensorPhase(true);
        rightMaster.setSensorPhase(true);

        resetEncoders();
        zeroHeading();
    }

    @Override
    public void periodic() {

        // leftDistance = RoboMath.toInches(leftMaster.getSelectedSensorPosition(0) -
        // leftDistance);
        // rightDistance = RoboMath.toInches(rightMaster.getSelectedSensorPosition(0) -
        // rightDistance);
        // System.out.println("leftDistance = " + leftDistance + " rightDistane = " +
        // rightDistance);
        // position = odometry.update(getHeading(), Units.inchesToMeters(leftDistance),
        // Units.inchesToMeters(rightDistance));

        // System.out.println("left " + (leftMaster.getSelectedSensorPosition(0) -
        // previousTicksLeft) + " right "
        // + (rightMaster.getSelectedSensorPosition(0) - previousTicksRight));
        // System.out.println("left "+ deltaMetersLeft + " right " + deltaMetersRight);

        position = odometry.update(getHeading(), RoboMath.ticksToMeters(leftMaster.getSelectedSensorPosition(0)), RoboMath.ticksToMeters(rightMaster.getSelectedSensorPosition(0)));
    }

    // /**
    // * Resets the odometry to the specified pose.
    // *
    // * @param pose The pose to which to set the odometry.
    // */
    // public void resetOdometry(Pose2d pose) {
    // resetEncoders();
    // odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    // }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // return new DifferentialDriveWheelSpeeds(
        // leftMaster.getSelectedSensorVelocity(0) / Const.GEAR_RATIO * 2 * Math.PI
        // * Units.inchesToMeters(Const.WHEEL_RADIUS) / 60,
        // rightMaster.getSelectedSensorVelocity(0) / Const.GEAR_RATIO * 2 * Math.PI
        // * Units.inchesToMeters(Const.WHEEL_RADIUS) / 60);

        return new DifferentialDriveWheelSpeeds(RoboMath.ticksToMeters(leftMaster.getSelectedSensorVelocity(0) * 10),
                RoboMath.ticksToMeters(rightMaster.getSelectedSensorVelocity(0) * 10));
    }

    public void setVolts(double leftVolts, double rightVolts) {
        // System.out.println("position " + position.getTranslation().getX() + " y " + position.getTranslation().getY()
        //         + " Angle " + getHeading());

        // System.out.println("left " + leftMaster.getSelectedSensorPosition(0) + " right " + rightMaster.getSelectedSensorPosition(0));

        // System.out.println("left speed: " + RoboMath.ticksToMeters(leftMaster.getSelectedSensorVelocity(0) * 10) + "right speed: " + RoboMath.ticksToMeters(rightMaster.getSelectedSensorVelocity(0) * 10));
        drive.feed();
        leftMaster.set(leftVolts / 12);
        rightMaster.set(rightVolts / 12);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
    }

    /**
     * sets the robots xy position to 0
     */
    public void resetPosition() {
        // TODO: sometime later
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        gyro.reset();
        gyro.zeroYaw();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getAngle(), 360) * (Const.kGyroReversed ? -1.0 : 1.0));
    }

    public SimpleMotorFeedforward getFeedFoward() {
        return feedforward;
    }

    public PIDController getLeftPIDController() {
        return leftPidController;
    }

    public PIDController getRightPIDController() {
        return rightPidController;
    }

    public DifferentialDriveKinematics getDifferentialDriveKinematics() {
        return kinematics;
    }

    public Pose2d getPosition() {
        return position;
    }
}