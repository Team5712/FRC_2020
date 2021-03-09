package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
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
public class DriveTrain extends SubsystemBase {


    private CANSparkMax leftMaster = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax leftSlave = new CANSparkMax(2, MotorType.kBrushless);

    private CANSparkMax rightMaster = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rightSlave = new CANSparkMax(4, MotorType.kBrushless);
    
    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;

    // TODO: set id
    private Solenoid shifter = new Solenoid(1);

    private DifferentialDrive drive;

    public AHRS gyro = new AHRS(Port.kMXP);

    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Const.TRACK_WIDTH);

    private Pose2d position = new Pose2d(0.85, -2.286, new Rotation2d(0.00));

    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), position);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Const.Ks, Const.Kv, Const.Ka);

    private PIDController leftPidController = new PIDController(Const.Kp, Const.Kd, 0);
    private PIDController rightPidController = new PIDController(Const.Kp, Const.Kd, 0);

    public DriveTrain() {
        
        leftMaster.restoreFactoryDefaults();
        leftSlave.restoreFactoryDefaults();
        rightMaster.restoreFactoryDefaults();
        rightSlave.restoreFactoryDefaults();

        leftMaster.setIdleMode(IdleMode.kCoast);
        leftSlave.setIdleMode(IdleMode.kCoast);
        rightMaster.setIdleMode(IdleMode.kCoast);
        rightSlave.setIdleMode(IdleMode.kCoast);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        
        leftMaster.setInverted(true);
        rightMaster.setInverted(false);

        drive = new DifferentialDrive(leftMaster, rightMaster);

        leftEncoder = leftMaster.getEncoder();
        rightEncoder = rightMaster.getEncoder();

        resetEncoders();
        zeroHeading();

        leftMaster.burnFlash();
        leftSlave.burnFlash();
        rightMaster.burnFlash();
        rightSlave.burnFlash();
    }

    public void setTeleopConfig() {
        leftMaster.setIdleMode(IdleMode.kCoast);
        leftSlave.setIdleMode(IdleMode.kCoast);
        rightMaster.setIdleMode(IdleMode.kCoast);
        rightSlave.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void periodic() {
        position = odometry.update(getHeading(), RoboMath.ticksToMeters(leftEncoder.getPosition()), RoboMath.ticksToMeters(rightEncoder.getPosition()));
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            leftEncoder.getVelocity() / 7 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60,
            rightEncoder.getVelocity() / 7 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60
            );
    }

    public void setVolts(double leftVolts, double rightVolts) {

        leftMaster.set((leftVolts / 12));
        rightMaster.set((rightVolts / 12));
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
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

    public void setShifterPosition(boolean position) {
        shifter.set(position);
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

    public void printYaw() {
        System.out.println("gyro yaw: " + gyro.getYaw());
    }

    public void TankDrive(double left, double right) {
        drive.tankDrive(left, right);
    }

    public void setPosition(Pose2d position) {
        this.position  = position;
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(-gyro.getAngle()));
    }
}