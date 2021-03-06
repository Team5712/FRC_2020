package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.Const;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

/**
 * RobotContainer
 */
public class RobotContainer {

    public DriveTrain drive = new DriveTrain();
    public Turret turret = new Turret();
    public Intake intake = new Intake();

    private ArrayList<Command> commands = new ArrayList<Command>();

    // TODO: create a dict of paths for a given autonomous mode

    public void loadConfigs(ArrayList<String> trajectoryPaths) {

        int index = 0;

        for (String path : trajectoryPaths) {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            try {

                Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                
                // if(index == 0) {
                //     drive.setPosition(trajectory.getInitialPose());
                //     System.out.println("INITIAL POSE " + drive.getPosition());
                // }

                RamseteCommand command = new RamseteCommand(trajectory, drive::getPosition,
                new RamseteController(2.0, .7), drive.getFeedFoward(), drive.getDifferentialDriveKinematics(),
                drive::getWheelSpeeds, drive.getLeftPIDController(), drive.getRightPIDController(),
                drive::setVolts, drive);
                commands.add(new WaitCommand(1).andThen(() -> turret.shoot()).andThen(new WaitCommand(5)).andThen(()->turret.stop()).andThen(()->intake.setIntakePower(Const.INTAKE_SPEED)).andThen(()->intake.setSolenoid(true)));

                commands.add(command);

                index++;

            } catch (IOException e) {
                // TODO: handle this just in case maybe
                System.out.println("Unable to open trajectory: " + path);
            }
        }

        commands.add(new WaitCommand(10).andThen(()->turret.shoot()).andThen(new WaitCommand(5)));

        System.out.println(trajectoryPaths.size() + " successfully loaded");
    }

    public Command getAutonomousCommand() {
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Const.Ks, Const.Kv, Const.Ka), drive.getDifferentialDriveKinematics(), 10);

        // TODO: update these
        TrajectoryConfig config = new TrajectoryConfig(1, 1);

        config.addConstraint(autoVoltageConstraint);
        config.setKinematics(drive.getDifferentialDriveKinematics());

        String trajectoryJSON = "paths/first.wpilib.json";

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


    public Command getNextAutonomousCommand() {

        System.out.println("retieving command, new size " + (commands.size()-1));

        try {
            return commands.remove(0);
        } catch (IndexOutOfBoundsException e) {
            System.out.println("out of bounds exception size: " + commands.size());
            return null;
        }
    }

    public void TankDrive(double left, double right) {
        // drive.setVolts(left * 12, right * 12);
        drive.TankDrive(left, right);
    }

    public void shift(boolean position) {
        drive.setShifterPosition(position);
    }

    public void resetSensors() {
        drive.resetEncoders();
        drive.zeroHeading();
    }

    public void printGyroYaw() {
        drive.printYaw();
    }

    public ArrayList<Command> getCommands() {
        return commands;
    }

}