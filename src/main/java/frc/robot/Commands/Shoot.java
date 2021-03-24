package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Turret;

/**
 * This completes a full cycle of shooting towards the target
 * 
 * This includes: turning aligning ramping up wheel speed moving intake wheels
 * shooting
 * 
 * This class extends the ParallelDeadlineGroup, which takes a list of commands and completes 
 * when a specified command is met
 */
public class Shoot extends ParallelDeadlineGroup {

    public Shoot(edu.wpi.first.wpilibj2.command.Command deadline, edu.wpi.first.wpilibj2.command.Command[] commands) {
        super(deadline, commands);
    }

    private Turret turretSubsystem;


    // @Override
    // public void initialize() {
        
    // }

    // @Override
    // public void execute() {

    // }

    // @Override
    // public boolean isFinished() {
    //     return false;
    // }

    
}