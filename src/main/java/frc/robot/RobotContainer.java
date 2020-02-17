package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.Const;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrainNeo;

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

        String trajectoryJSON = "paths/collect.wpilib.json";

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

            RamseteCommand command = new RamseteCommand(trajectory, drive::getPosition, new RamseteController(2.0, .7),
                    drive.getFeedFoward(), drive.getDifferentialDriveKinematics(), drive::getWheelSpeeds,
                    drive.getLeftPIDController(), drive.getRightPIDController(), drive::setVolts, drive);

            return command.andThen(() -> drive.setVolts(0, 0));
        } catch (IOException ex) {
            System.out.println("Unable to open trajectory: " + trajectoryJSON);
        }

        // TODO: return empty command to set motors to 0, 0
        return null;
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