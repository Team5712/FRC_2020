package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.Const;

import com.revrobotics.CANEncoder;

/**
 * Shooter
 */
public class Shooter {

    // CANSparkMax shooterMaster = new CANSparkMax(7,
    // CANSparkMax.MotorType.kBrushless);
    // CANSparkMax shooterSlave = new CANSparkMax(8,
    // CANSparkMax.MotorType.kBrushless);

    // CANPIDController pidController;

    WPI_TalonFX shooterMaster = new WPI_TalonFX(7);
    WPI_TalonFX shooterSlave = new WPI_TalonFX(8);

    CANSparkMax hoodAdjuster = new CANSparkMax(9, CANSparkMax.MotorType.kBrushless);
    private CANSparkMax turret_spin  = new CANSparkMax(10, CANSparkMax.MotorType.kBrushless);
    AnalogPotentiometer hoodAdjusterPot;

    

    public Shooter() {
        // shooterSlave.follow(shooterMaster, true);

        // pidController = shooterMaster.getPIDController();

        // pidController.setP(Const.SHOOTING_Kp);
        // pidController.setI(Const.SHOOTING_Kd);
        // pidController.setD(0);
        // pidController.setIZone(0);
        // pidController.setFF(0);
        // pidController.setOutputRange(0, Const.SHOOTING_TARGET_RPM);

        hoodAdjusterPot = new AnalogPotentiometer(2, 10);

        shooterMaster.config_kP(0, Const.SHOOTING_Kp, 30);
        shooterMaster.config_kI(0, Const.SHOOTING_Ki, 30);
        shooterMaster.config_kD(0, Const.SHOOTING_Kd, 30);
        shooterMaster.config_kF(0, Const.SHOOTING_Kf, 30);
        shooterMaster.config_IntegralZone(0, 50);
        shooterSlave.config_kP(0, Const.SHOOTING_Kp, 30);
        shooterSlave.config_kI(0, Const.SHOOTING_Ki, 30);
        shooterSlave.config_kD(0, Const.SHOOTING_Kd, 30);
        shooterSlave.config_kF(0, Const.SHOOTING_Kf, 30);
        shooterSlave.config_IntegralZone(0, 50);
    }

    public void shoot() {
        setPowerRPM();
        System.out.println("Error:" + shooterMaster.getClosedLoopError());
    
        //setPower(.2);
        
    }

    public void setPower(double power) {
        shooterSlave.follow(shooterMaster);
        //shooterSlave.setInverted(InvertType.OpposeMaster);
        shooterMaster.set(ControlMode.PercentOutput, power);
        //shooterSlave.set(ControlMode.PercentOutput, power);

    }

    public void setPowerRPM() {
        shooterSlave.follow(shooterMaster, FollowerType.AuxOutput1);
        shooterMaster.set(ControlMode.Velocity, Const.SHOOTING_UNITS_PER_REV * Const.SHOOTING_TARGET_RPM / 600);
        //shooterSlave.set(ControlMode.Velocity, Const.SHOOTING_UNITS_PER_REV * Const.SHOOTING_TARGET_RPM / 600);
        System.out.println("Shooter Error (ticks): " + shooterMaster.getClosedLoopError(0));

    }

    public void stop() {
        shooterMaster.set(0);
        shooterSlave.set(0);
    }

    public void printShooterTicks() {
        System.out.println("Master: " + shooterMaster.getSelectedSensorPosition());
        System.out.println("Slave: " + shooterSlave.getSelectedSensorPosition());
    }

    public void printHoodPosition() {
        System.out.println(new DecimalFormat("#.##").format(hoodAdjusterPot.get()));
    }

    public void setHoodBackPosition() {
        double error = Const.HOOD_BACK - hoodAdjusterPot.get();
        double output = Const.HOOD_Kp * error;
        double final_output = Math.min(Math.max(output, Const.HOOD_MINOUTPUT), Const.HOOD_MAXOUTPUT);
        System.out.println("final_output: " + final_output);
        this.setHoodPower(final_output);
    }

    public void setHoodFrontPosition() {
        double error = Const.HOOD_FRONT - hoodAdjusterPot.get();
        double output = Const.HOOD_Kp * error;
        double final_output = Math.min(Math.max(output, Const.HOOD_MINOUTPUT), Const.HOOD_MAXOUTPUT);
        this.setHoodPower(final_output);
        System.out.println("final_output: " + final_output);
    }

    public void setHoodMiddlePosition() {
        double error = Const.HOOD_MIDDLE - hoodAdjusterPot.get();
        double output = Const.HOOD_Kp * error;
        double final_output = Math.min(Math.max(output, Const.HOOD_MINOUTPUT), Const.HOOD_MAXOUTPUT);
        this.setHoodPower(final_output);
        System.out.println("final_output: " + final_output);
    }

    public void setHoodPower(double power) {
        hoodAdjuster.set(power);
    }

    public void resetShooterTicks(){
        shooterMaster.setSelectedSensorPosition(0, 0, 10);
        shooterSlave.setSelectedSensorPosition(0, 0, 10);
    }
}