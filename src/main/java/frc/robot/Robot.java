/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    private Timer autoTimer = new Timer();
    private Timer IRSensorTimer = new Timer();
    private Timer conveyorReverseTimer = new Timer();


    boolean isIntaking = false;
    boolean isColorSensorActive = true;
    
    private boolean isConveyorReversed = true;
    private boolean isIRConveyorRunning = false;
    private DigitalInput IRSensor = new DigitalInput(0);


    // TODO: remove these
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);



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
        // System.out.println("dashboard is null!");
        // autoSelection = "startmidfar3balltrench";
        // } else {
        // autoSelection = chooser.getSelected();
        // }

        // System.out.println("Auto Selected: " + autoSelection);

        // File pathFolder = new
        // File(Filesystem.getDeployDirectory().toPath().resolve("paths").toString());

        // File[] paths = pathFolder.listFiles();

        // for (File path : paths) {
        // String pathName = path.getName();

        // if (pathName.startsWith(autoSelection)) {
        // System.out.println("adding path/" + pathName);
        // trajectoryPaths.add("paths/" + pathName);
        // }
        // }

        // trajectoryPaths.add("paths/startmidfar3balltrench_trench.wpilib.json");
        // container.loadConfigs(trajectoryPaths);

    }

    @Override
    public void autonomousInit() {
        container.resetSensors();
        System.out.println("Autonomous has started");

        autoTimer.start();

        // currentCommand = container.getNextAutonomousCommand();

        // if (currentCommand != null) {
        // currentCommand.schedule();
        // }

        // Consumer<Command> increment = a -> commandNumber++;
        // CommandScheduler.getInstance().onCommandFinish(increment);
    }

    @Override
    public void autonomousPeriodic() {

        // CommandScheduler.getInstance().run();

        switch (commandNumber) {
        case 0:

            if (autoTimer.get() > 10) {
                turret.stop();
                commandNumber++;
            } else {

                if (autoTimer.get() > 3) {
                    intake.setFrontConveyorPower(-Const.FRONT_CONVEYOR_SPEED);
                    intake.setBackConveyorPower(-Const.BACK_CONVEYOR_SPEED);
                    intake.setIntakePower(-Const.INTAKE_SPEED);
                }

                turret.shoot();
            }

            break;

        case 1:

            if (autoTimer.get() > 12) {
                commandNumber++;
            } else {
                turret.stop();
                container.drive.TankDrive(-0.55, 0.55);
            }

            break;

        case 2:
            intake.setIntakePower(0);
            intake.setFrontConveyorPower(0);
            intake.setBackConveyorPower(0);

            container.TankDrive(0, 0);
            turret.stop();

            break;

        default:
            System.out.println("invalid command number: " + commandNumber);
        }
    }

    @Override
    public void teleopInit() {
        isIntaking = false;
        container.resetSensors();
        turret.resetShooterTicks();
    }

    @Override
    public void teleopPeriodic() {
        handlePrimaryDriverInput();
        handleSecondaryDriverInput();

        System.out.println("pos " + hood.getRelativePosition());
        SmartDashboard.putNumber("RPM Error", turret.getShooterError());
        // System.out.println("y thing " + Vision.getNumber("ty") + " hood " + hood.getRelativePosition());
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

        // run intake in and out
        if (rightJoystick.getRawButtonPressed(2)) {
            isIntaking = !isIntaking;
        }

        if (isIntaking) {

            intake.setIntakePower(-Const.INTAKE_SPEED);
            intake.setSolenoid(true);

            // if they aren't reversing
        } else {
            intake.setIntakePower(0);
            intake.setSolenoid(false);
        }

        // left joystick left button disable color sensor
        if (leftJoystick.getRawButtonPressed(2)) {
            isColorSensorActive = !isColorSensorActive;
        }

        double red = colorSensor.getRed();
        double green = colorSensor.getGreen();
        double blue = colorSensor.getBlue();

        double alpha = red + green + blue / (1);

        // left joystick middle button reverse WHILE intaking
        // left joystick
        if (leftJoystick.getRawButton(4)) {
            intake.setFrontConveyorPower(Const.FRONT_CONVEYOR_SPEED);
            intake.setIntakePower(Const.INTAKE_SPEED);
            intake.setBackConveyorPower(Const.BACK_CONVEYOR_SPEED);
        } else if (leftJoystick.getRawButton(3)) {
            intake.setFrontConveyorPower(-Const.FRONT_CONVEYOR_SPEED);
            // (red > Const.COLOR_RED_THRESHOLD && green > Const.COLOR_GREEN_THRESHOLD)
        } else if (IRSensor.get() && isColorSensorActive && isIntaking) {
            IRSensorTimer.start();
            isIRConveyorRunning = true;
        } else {
            intake.setFrontConveyorPower(0);
            // System.out.println("red " + colorSensor.getRed() + " green " +
            // colorSensor.getGreen() + " alpha " + alpha);
        }

        if (isIRConveyorRunning && IRSensorTimer.get() < 0.5) {
            // System.out.println("running timer " + IRSensorTimer.get());
            intake.setFrontConveyorPower(-0.5);
        } else if (!leftJoystick.getRawButton(3) && !leftJoystick.getRawButton(4)) {
            isIRConveyorRunning = false;
            intake.setFrontConveyorPower(0);
        }

        // System.out.println("distance " + Vision.getDistanceToTarget());

        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Drive Handling
        // .............................................................................

        // Auto center on target
        if (rightJoystick.getRawButton(1)) {

            double[] turnValues = vision.getTurnValues();
            // System.out.println("left adjust " + turnValues[0] + "right adjust " +
            // turnValues[1]);
            container.TankDrive(turnValues[0] - leftJoystick.getRawAxis(1),
                    turnValues[1] - rightJoystick.getRawAxis(1));

            // default drive
        } else {
            // TODO: bring this back
            Vision.disableLEDS();
            container.TankDrive(-leftJoystick.getRawAxis(1), rightJoystick.getRawAxis(1));
        }
    }

    /**
     * Handle input for the secondary driver (auxJoystick)
     */
    private void handleSecondaryDriverInput() {
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Turret
        // .............................................................................

        // A button hood back
        if (auxJoystick.getRawButton(1)) {
            hood.setPower(.1);
            // Y button hood forward
        } else if (auxJoystick.getRawButton(4)) {
            hood.setPower(-.1);
            // X button hood set to control panel shooting position
        } else if (auxJoystick.getRawButton(3)) {
            hood.setPosition(Const.HOOD_LINE_POSITION);
            Vision.setNumber("pipeline", 0);
            Vision.enableLEDS();
            // Hold B button for shooting on the line
        } else if (auxJoystick.getRawButton(2)) {
            Vision.setNumber("pipeline", 0);
            Vision.enableLEDS();
            hood.setPosition(Vision.getDesiredHoodAngleToTarget());
        } else if(auxJoystick.getRawButton(7)) {
            Vision.enableLEDS();
            hood.setPosition(Const.HOOD_CONTROL_PANEL_POSITION);
        } else {
            hood.setPower(0);
        }
        
        if (auxJoystick.getRawAxis(3) > .3) {
            turret.shoot();
            if (turret.getShooterError() < Math.abs(Const.INTAKE_BACK_CONVEYOR_THRESHOLD)) {
                // TODO: debug this
                // intake.setBackConveyorPower(-Const.BACK_CONVEYOR_SPEED);
                // intake.setFrontConveyorPower(-Const.FRONT_CONVEYOR_SPEED);
            }
        } else {
            turret.stop();
            intake.setBackConveyorPower(0);
        }

        if (auxJoystick.getRawButtonPressed(5)) {
            isConveyorReversed = true;
            conveyorReverseTimer.start();
        }
        
        if (isConveyorReversed && conveyorReverseTimer.get() < 0.1 && auxJoystick.getRawButton(5)) {
            // System.out.println("running timer " + IRSensorTimer.get());
            intake.setBackConveyorPower(Const.BACK_CONVEYOR_SPEED);
            intake.setFrontConveyorPower(Const.FRONT_CONVEYOR_SPEED);
        } else if (!isConveyorReversed && auxJoystick.getRawButton(5)) {
            intake.setFrontConveyorPower(-Const.FRONT_CONVEYOR_SPEED);
            intake.setBackConveyorPower(-Const.BACK_CONVEYOR_SPEED);
        } else if(auxJoystick.getRawButton(6)) {
            intake.setFrontConveyorPower(Const.FRONT_CONVEYOR_SPEED);
        } else {
            isConveyorReversed = false;
            conveyorReverseTimer.reset();
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

    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

}
