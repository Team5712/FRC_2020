/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.Const;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

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
    public Joystick auxJoystick = new Joystick(2);
    public String autonMode = "";

    private Command currentCommand;
    private int commandNumber = 0;

    private Vision vision = new Vision();
    private Climber climber = new Climber();
    private Intake intake = new Intake();
    private Hood hood = new Hood();

    private RobotContainer container;

    Turret turret = new Turret();
    SendableChooser<String> chooser = new SendableChooser<String>();

    @Override
    public void robotInit() {
        container = new RobotContainer();
        chooser.addOption("startmidclose3balltrench", "startmidclose3balltrench");
        // chooser.addOption("startright3balltrench", "startright3balltrench");
        chooser.addOption("startmidfar3balltrench", "startmidfar3balltrench");
        // chooser.addOption("startmidfar5balltrench", "startmidfar5balltrench");
        // chooser.addOption("startmidclose5balltrench", "startmidclose5balltrench");
        // chooser.addOption("startmidright5balltrench", "startmidright5balltrench");
        // chooser.addOption("startmidclose2ballren", "startmidclose2ballren");
        // chooser.addOption("startmidclose3ballren", "startmidclose3ballren");
        // chooser.addOption("startleft2balltrench", "startleft2balltrench");
        // chooser.addOption("Cit_Circuits", "Cit_Circuits");
        SmartDashboard.putData("Auto Mode", chooser);

        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Look for every path inside of our path folder prefixed with the appropriate
        // group and load them
        // ....................................................................................................
        
        // String autoSelection = "";
        ArrayList<String> trajectoryPaths = new ArrayList<String>();

        // if (chooser.getSelected() == null || chooser.getSelected().isEmpty()) {
        //     System.out.println("dashboard is null!");
        //     autoSelection = "startmidfar3balltrench";
        // } else {
        //     autoSelection = chooser.getSelected();
        // }

        // System.out.println("Auto Selected: " + autoSelection);

        // File pathFolder = new File(Filesystem.getDeployDirectory().toPath().resolve("paths").toString());


        // File[] paths = pathFolder.listFiles();

        // for (File path : paths) {
        //     String pathName = path.getName();

        //     if (pathName.startsWith(autoSelection)) {
        //         System.out.println("adding path/" + pathName);
        //         trajectoryPaths.add("paths/" + pathName);
        //     }
        // }

        trajectoryPaths.add("paths/startmidfar3balltrench.wpilib.json");
        container.loadConfigs(trajectoryPaths);
    }

    @Override
    public void autonomousInit() {
        container.resetSensors();
        System.out.println("Autonomous has started");

        currentCommand = container.getNextAutonomousCommand();

        if (currentCommand != null) {
            currentCommand.schedule();
        }

        Consumer<Command> increment = a -> commandNumber++;
        CommandScheduler.getInstance().onCommandFinish(increment);

        // autoTimer.reset();
        // autoTimer.start();
    }

    @Override
    public void autonomousPeriodic() {

        if (commandNumber == 0) {
            CommandScheduler.getInstance().run();
            System.out.println("RUNNING FIRST");
        } else if (commandNumber == 1) {

            System.out.println("ONTO SECOND COMMAND");
        } else {
            System.out.println(" no work :(");
        }
    }

    @Override
    public void teleopInit() {
        container.resetSensors();
        turret.resetShooterTicks();
    }

    @Override
    public void teleopPeriodic() {
        handlePrimaryDriverInput();
        handleSecondaryDriverInput();

        SmartDashboard.putNumber("Joystick X value", leftJoystick.getX());
        // System.out.println(chooser.getSelected());

    }

    /**
     * Handle input for the primary driver ((left/right)Joystick)
     */
    private void handlePrimaryDriverInput() {

        // Shifting
        if (leftJoystick.getRawButton(1)) {
            container.shift(true);
        } else {
            container.shift(false);
        }

        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Intake
        // .............................................................................

        // run ground intake
        if (rightJoystick.getRawButton(4)) {
            intake.setIntakePower(-Const.INTAKE_SPEED);
            intake.setFrontConveyorPower(-Const.FRONT_CONVEYOR_SPEED);
            // Run conveyors to pick directly from station
        } else if (rightJoystick.getRawButton(2)) {
            intake.setIntakePower(Const.INTAKE_SPEED);
            intake.setFrontConveyorPower(Const.FRONT_CONVEYOR_SPEED);
        } else {
            intake.setIntakePower(0);
            intake.setFrontConveyorPower(0);
        }

        // run conveyor
        if (leftJoystick.getRawButton(4)) {
            intake.setBackConveyorPower(-Const.BACK_CONVEYOR_SPEED);
        } else if (leftJoystick.getRawButton(3)) {
            intake.setBackConveyorPower(Const.BACK_CONVEYOR_SPEED);
        } else {
            intake.setBackConveyorPower(0);
        }

        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Drive Handling
        // .............................................................................

        // Auto center on target
        if (rightJoystick.getRawButton(1)) {
            // TODO: Test turn

            double[] turnValues = vision.getTurnValues();
            // System.out.println("left adjust " + turnValues[0] + "right adjust " +
            // turnValues[1]);
            container.TankDrive(turnValues[0] - leftJoystick.getRawAxis(1),
                    turnValues[1] - rightJoystick.getRawAxis(1));

            // default drive
        } else {
            container.TankDrive(-leftJoystick.getRawAxis(1), rightJoystick.getRawAxis(1));
        }

        // reverse intake
        if (rightJoystick.getRawButton(3)) {
            // intake.setSolenoid(true);
            intake.setIntakePower(Const.INTAKE_SPEED);
            intake.setFrontConveyorPower(Const.FRONT_CONVEYOR_SPEED);
            intake.setBackConveyorPower(Const.BACK_CONVEYOR_SPEED);
        }
    }

    /**
     * Handle input for the secondary driver (auxJoystick)
     */
    private void handleSecondaryDriverInput() {
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Climbing
        // .............................................................................
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Turret
        // .............................................................................

        // Left
        if (auxJoystick.getRawButton(5)) {
            // TODO: check direction
            turret.setTurretYawPower(Const.TURRET_YAW_RATE);
            // Right
        } else if (auxJoystick.getRawButton(6)) {
            turret.setTurretYawPower(-Const.TURRET_YAW_RATE);
        } else {
            turret.setTurretYawPower(0);
        }

        if (auxJoystick.getRawButton(1)) {
            hood.setPower(.1);
        } else if (auxJoystick.getRawButton(4)) {
            hood.setPower(-.1);
        } else if (auxJoystick.getRawButton(3)) {
            hood.setPosition(Const.HOOD_CONTROL_PANEL_POSITION);
        } else {
            // hood.setPosition(Const.HOOD_MAX_POSITION);
            hood.setPower(0);
        }

        // if(auxJoystick.getRawButton(1)) {
        // hood.setHoodPosition(1);
        // }

        if (auxJoystick.getRawAxis(3) > .3) {
            turret.shoot();
            // turret.setPower(0.6);
        } else {
            turret.stop();
        }
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Climbing
        // .............................................................................

        // right joystick handles climbing
        if (auxJoystick.getRawAxis(5) > 0.3) {
            climber.setPower(-Const.CLIMB_SPEED);
        } else if (auxJoystick.getRawAxis(5) < -0.3) {
            climber.setPower(Const.CLIMB_SPEED);
        } else {
            climber.setPower(0);
        }

        // B Button for Popup actuation
        if (auxJoystick.getRawButton(2)) {
            turret.setPopUpPosition(true);
        } else {
            turret.setPopUpPosition(false);
        }
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

}
