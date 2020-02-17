package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
 * DriveTrain
 */
public class DriveTrainNeo extends SubsystemBase {

    // private WPI_TalonSRX leftMaster = new WPI_TalonSRX(1);
    // private WPI_TalonSRX leftSlave = new WPI_TalonSRX(2);

    // private WPI_TalonSRX rightMaster = new WPI_TalonSRX(5);
    // private WPI_TalonSRX rightSlave = new WPI_TalonSRX(6);

     private CANSparkMax leftMaster = new CANSparkMax(1, MotorType.kBrushless);
     private CANSparkMax leftSlave = new CANSparkMax(2, MotorType.kBrushless);

    private CANSparkMax rightMaster = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rightSlave = new CANSparkMax(4, MotorType.kBrushless);
    
    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;

    private DifferentialDrive drive;

    private AHRS gyro = new AHRS(Port.kMXP);

    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Const.TRACK_WIDTH);
    // private Pose2d position = new Pose2d(0.0, 0.0, getHeading());
    private Pose2d position = new Pose2d(3.157, -2.361, getHeading());

    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), position);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Const.Ks, Const.Kv, Const.Ka);

    private PIDController leftPidController = new PIDController(Const.Kp, Const.Kd, 0); // .835
    private PIDController rightPidController = new PIDController(Const.Kp, Const.Kd, 0); // .835

    public DriveTrainNeo() {

         leftSlave.follow(leftMaster);
         rightSlave.follow(rightMaster);
        
         leftMaster.setInverted(true);
        rightMaster.setInverted(false);

        drive = new DifferentialDrive(leftMaster, rightMaster);

        leftEncoder = leftMaster.getEncoder();
        rightEncoder = rightMaster.getEncoder();

        resetEncoders();
        zeroHeading();
    }

    @Override
    public void periodic() {

        // System.out.println("Velocity left: " + leftMaster.getEncoder().getVelocity() + "Velocity right: " + rightMaster.getEncoder().getVelocity()  );
        // position = odometry.update(getHeading(), RoboMath.ticksToMeters(leftMaster.getSelectedSensorPosition(0)), RoboMath.ticksToMeters(rightMaster.getSelectedSensorPosition(0)));
        position = odometry.update(getHeading(), RoboMath.ticksToMeters(leftEncoder.getPosition()), RoboMath.ticksToMeters(rightEncoder.getPosition()));
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
        return new DifferentialDriveWheelSpeeds(
            leftEncoder.getVelocity() / 7 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60,
            rightEncoder.getVelocity() / 7 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60
            );
    }

    public void setVolts(double leftVolts, double rightVolts) {
        leftMaster.set(leftVolts / 12);
        rightMaster.set(rightVolts / 12);

        //System.out.println("x " + position.getTranslation().getX() + " y " + position.getTranslation().getY());
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
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