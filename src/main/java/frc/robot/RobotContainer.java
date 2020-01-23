package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.Const;
import frc.robot.subsystems.DriveTrain;

/**
 * RobotContainer
 */
public class RobotContainer {

    private DriveTrain drive = new DriveTrain();

    public Command getAutonomousCommand() {
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Const.Ks, Const.Kv, Const.Ka), drive.getDifferentialDriveKinematics(), 10);

        TrajectoryConfig config = new TrajectoryConfig(2, 2);

        config.addConstraint(autoVoltageConstraint);
        config.setKinematics(drive.getDifferentialDriveKinematics());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,
        0, new Rotation2d(0)),
        List.of(new Translation2d(2.1, 1.8), new Translation2d(4.8, 1.8)),new Pose2d(2, 1.8,
        new Rotation2d(Math.PI)),
        config);

        // Trajectory trajectory = null;

        // try {
        //     trajectory = TrajectoryUtil.toPathweaverJson(Paths.get("/home/lvuser/deploy/testauto"));
        //     Pathfinder.readFromCSV("");
        // } catch (IOException e) {
        //     e.printStackTrace();
        // }

        // // new Pose2d(1, 0, new Rotation2d(90))),
        RamseteCommand command = new RamseteCommand(trajectory, drive::getPosition, new RamseteController(2.0, .7),
                drive.getFeedFoward(), drive.getDifferentialDriveKinematics(), drive::getWheelSpeeds,
                drive.getLeftPIDController(), drive.getRightPIDController(), drive::setVolts, drive);

         return command.andThen(() -> drive.setVolts(0, 0));
    }


    public void TankDrive(double left, double right) {
        drive.setVolts(left * 12, right * 12);
    }

    public void resetSensors() {
        drive.resetEncoders();
        drive.zeroHeading();
        drive.resetPosition();
    }

}