/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */

    public Joystick leftJoystick = new Joystick(0);
    public Joystick rightJoystick = new Joystick(1);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-two");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    double left_command = 0;
    double right_command = 0;

    private Command autonomousCommand;

    private RobotContainer container;

    Shooter shooter = new Shooter();

    @Override
    public void robotInit() {
        container = new RobotContainer();

    }

    @Override
    public void autonomousInit() {
        container.resetSensors();

        autonomousCommand = container.getAutonomousCommand();
        
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        container.resetSensors();
        shooter.resetShooterTicks();
    }

    @Override
    public void teleopPeriodic() {
        // double x = tx.getDouble(0.0);
        // double y = ty.getDouble(0.0);
        // double area = ta.getDouble(0.0);
        // float Kp = -0.03f; // Proportional control constant
        // double min_command = 0.09;

        // if (leftJoystick.getRawButton(3)) {
        //     double steering_adjust = 0;
        //     left_command = 0;
        //     right_command = 0;

        //     if (x > .5) {
        //         steering_adjust = Kp * x - min_command;
        //     } else if (x < -.5) {
        //         steering_adjust = Kp * x + min_command;
        //     }

        //     left_command -= steering_adjust;
        //     right_command += steering_adjust;
        //     //System.out.println("Left_command: " + left_command);
        //     //System.out.println("Right_command: " + right_command);
        //     container.TankDrive(left_command - leftJoystick.getRawAxis(1), right_command - rightJoystick.getRawAxis(1));
        // } else {
        //     container.TankDrive(-leftJoystick.getRawAxis(1), -rightJoystick.getRawAxis(1));
        // }

        // if (leftJoystick.getRawButton(1)) {
        //     shooter.shoot();
        //     //shooter.printShooterTicks();
        //     //shooter.printHoodPosition();
        // } else {
        //     shooter.stop();
        // } 
        // if (rightJoystick.getRawButton(2)) {
        //     shooter.setHoodPower(.1);
        //     shooter.printHoodPosition();
        //     //shooter.setHoodBackPosition();
        // }
        //  else if(rightJoystick.getRawButton(3)) {
        //     shooter.setHoodPower(-.1);
        //     //shooter.setHoodFrontPosition();
        //     shooter.printHoodPosition();
        // } 
        // else{
        //     shooter.setHoodPower(0);
        // }


        container.TankDrive(-leftJoystick.getRawAxis(1), -rightJoystick.getRawAxis(1));
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

}
