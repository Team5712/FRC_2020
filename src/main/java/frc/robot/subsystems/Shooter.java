package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import frc.Const;

/**
 * Shooter
 */
public class Shooter {

    CANSparkMax shooterMaster = new CANSparkMax(7, CANSparkMax.MotorType.kBrushless);
    CANSparkMax shooterSlave = new CANSparkMax(8, CANSparkMax.MotorType.kBrushless);

    CANPIDController pidController;

    CANSparkMax hoodAdjuster = new CANSparkMax(9, CANSparkMax.MotorType.kBrushless);

    CANEncoder shooterEncoder = shooterMaster.getEncoder();
    CANEncoder hoodAdjusterEncoder = hoodAdjuster.getEncoder();
    

    public Shooter() {
        shooterSlave.follow(shooterMaster, true);

        pidController = shooterMaster.getPIDController();

        pidController.setP(Const.SHOOTING_Kp);
        pidController.setI(Const.SHOOTING_Kd);
        pidController.setD(0);
        pidController.setIZone(0);
        pidController.setFF(0);
        pidController.setOutputRange(0, Const.SHOOTING_TARGET_RPM);
    }

    public void shoot() {

        pidController.setReference(Const.SHOOTING_TARGET_RPM, ControlType.kVelocity);

        System.out.println("speed " + shooterEncoder.getVelocity());
    }
    
    private void setPower(double power) {
        shooterMaster.set(power);
    }

    public void stop() {
        shooterMaster.set(0);
    }
}