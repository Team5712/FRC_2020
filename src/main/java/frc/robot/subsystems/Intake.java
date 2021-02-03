/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class Intake {
    private CANSparkMax intake = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax frontConveyor = new CANSparkMax(6, MotorType.kBrushless);
    private CANSparkMax backConveyor = new CANSparkMax(7, MotorType.kBrushless);
    
    private Solenoid intakeSolenoid = new Solenoid(2);
   
    public void setSolenoid(boolean position) {
       intakeSolenoid.set(position);
    }

    public void setIntakePower(double power) {
        intake.set(power);
    }

    public void setFrontConveyorPower(double power) {
        frontConveyor.set(power);
    }

    public void setBackConveyorPower(double power) {
        backConveyor.set(-power);
    }
}
