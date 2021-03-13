package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

/**
 * RobotContainer
 */
public class RobotContainer {

    public DriveTrain drive = new DriveTrain();

    SendableChooser<ArrayList<String>> chooser = new SendableChooser<ArrayList<String>>();
    HashMap<String, Trajectory> pathweaverTrajectories = new HashMap<String, Trajectory>();
    HashMap<String, Command> pathweaverCommands = new HashMap<String, Command>();

    public RobotContainer() {

        /**
         *  Generate trajectories for all files in deploy path. It is recommended to 
         *  generate trajectories on startup.
         * */ 
        File folder = Filesystem.getDeployDirectory().toPath().resolve("paths/output").toFile();
        File[] listOfFiles = folder.listFiles();

        for (int i = 0; i < listOfFiles.length; i++) {
            if (listOfFiles[i].isFile()) {
                try {
                    Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(listOfFiles[i].toPath());
        
                    RamseteCommand command = new RamseteCommand(trajectory, drive::getPosition,
                    new RamseteController(2.0, .7), drive.getFeedFoward(), drive.getDifferentialDriveKinematics(),
                    drive::getWheelSpeeds, drive.getLeftPIDController(), drive.getRightPIDController(),
                    drive::setVolts, drive);

                    System.out.println(listOfFiles[i].getName());

                    pathweaverTrajectories.put(listOfFiles[i].getName(), trajectory);
                    pathweaverCommands.put(listOfFiles[i].getName(), command);
        
                    
                } catch (IOException e) {
                    System.out.println("Unable to open trajectory: " + listOfFiles[i].getName());
                }
            } 
        }

        /**
         *  Define a list of selections to choose an autonomous path. 
         * */ 
        ArrayList<String> straightPath = new ArrayList<String>();
        straightPath.add("straight.wpilib.json");
        chooser.setDefaultOption("Straight Path", straightPath);

        ArrayList<String> slalomPath = new ArrayList<String>();
        slalomPath.add("slalom.wpilib.json");
        chooser.addOption("Slalom Path", slalomPath);

        ArrayList<String> barrelRacingPath = new ArrayList<String>();
        barrelRacingPath.add("barrelracing.wpilib.json");
        chooser.addOption("Barrel Racing Path", barrelRacingPath);

        ArrayList<String> bouncePath = new ArrayList<String>();
        bouncePath.add("Bounce.wpilib.json");
        bouncePath.add("Bounce_0.wpilib.json");
        bouncePath.add("Bounce_2.wpilib.json");
        bouncePath.add("Bounce_1.wpilib.json");
        chooser.addOption("Bounce Path", bouncePath);

        SmartDashboard.putData("Auto Mode", chooser);

    }

    public Command getAutonomousCommand() {

        System.out.println(chooser.getSelected());
        
        drive.resetOdometry(pathweaverTrajectories.get(chooser.getSelected().get(0)).getInitialPose());
        
        SequentialCommandGroup autonPaths = new SequentialCommandGroup();
		for(String paths: chooser.getSelected()) {
            autonPaths.addCommands(pathweaverCommands.get(paths));
        }
        return autonPaths;
    }


    // private ArrayList<Command> getConfigs(ArrayList<String> trajectoryPaths) {

    //     ArrayList<Command> commands = new ArrayList<Command>();

    //     for(int i = 0; i < trajectoryPaths.size(); i++) {

    //         String path = trajectoryPaths.get(i);
    //         Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);

    //         try {
    //             Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    //             RamseteCommand command = new RamseteCommand(trajectory, drive::getPosition,
    //             new RamseteController(2.0, .7), drive.getFeedFoward(), drive.getDifferentialDriveKinematics(),
    //             drive::getWheelSpeeds, drive.getLeftPIDController(), drive.getRightPIDController(),
    //             drive::setVolts, drive);

    //             // Reset odometry to the starting pose of the trajectory.
    //             // if (i == 0) {
    //             //     drive.resetOdometry(trajectory.getInitialPose());
    //             // }
               
    //             commands.add(command);
                
    //         } catch (IOException e) {
    //             System.out.println("Unable to open trajectory: " + path);
    //         }
    //     }

    //     System.out.println(trajectoryPaths.size() + " successfully loaded");

    //     return commands;
    // }

    // public Command getAutonomousCommand(String trajectoryJSON) {

    //     try {

    //         Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //         Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    //         RamseteCommand command = new RamseteCommand(trajectory, drive::getPosition,
    //         new RamseteController(2.0, .7), drive.getFeedFoward(), drive.getDifferentialDriveKinematics(),
    //         drive::getWheelSpeeds, drive.getLeftPIDController(), drive.getRightPIDController(),
    //         drive::setVolts, drive);
    

    //         // Reset odometry to the starting pose of the trajectory.
    //         drive.resetOdometry(trajectory.getInitialPose());

    //         // Run path following command, then stop at the end.
    //         return command;

    //     } catch (IOException e) {
    //         System.out.println("Unable to open trajectory: " + trajectoryJSON);
    //         return null;
    //     }

    // }


    // public Command getNextAutonomousCommand() {

    //     System.out.println("retieving command, new size " + (commands.size()-1));

    //     try {
    //         return commands.remove(0);
    //     } catch (IndexOutOfBoundsException e) {
    //         System.out.println("out of bounds exception size: " + commands.size());
    //         return null;
    //     }
    // }

    public void TankDrive(double left, double right) {
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
}