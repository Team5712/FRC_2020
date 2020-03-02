package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Controls everything about the climber
 */
public class Climber {

    CANSparkMax climber = new CANSparkMax(12, MotorType.kBrushless);

    public void setPower(double power) {
        climber.set(power);
    }
}