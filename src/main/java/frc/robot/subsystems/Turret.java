package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Solenoid;
import frc.Const;

/**
 * Turret
 */
public class Turret {

    WPI_TalonFX shooterMaster = new WPI_TalonFX(10);
    WPI_TalonFX shooterSlave = new WPI_TalonFX(11);

    private CANSparkMax turretYawAdjuster = new CANSparkMax(8, CANSparkMax.MotorType.kBrushless);

    Solenoid popUp = new Solenoid(0);

    public Turret() {
        shooterSlave.setInverted(true);
        shooterSlave.follow(shooterMaster, FollowerType.AuxOutput1);

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
        shooterMaster.set(ControlMode.Velocity, Const.SHOOTING_UNITS_PER_REV * Const.SHOOTING_TARGET_RPM / 600);
        System.out.println("Turret Error (ticks): " + shooterMaster.getClosedLoopError(0));
    }

    public void setPower(double power) {
        // shooterSlave.follow(shooterMaster);
        shooterMaster.set(ControlMode.PercentOutput, power);
    }

    public void stop() {
        shooterMaster.set(0);
        shooterSlave.set(0);
    }

    public void setPopUpPosition(boolean position) {
        popUp.set(position);
    }

    public void printShooterTicks() {
        System.out.println("Master: " + shooterMaster.getSelectedSensorPosition());
        System.out.println("Slave: " + shooterSlave.getSelectedSensorPosition());
    }

    public void setTurretYawPosition(double position) {
        // TODO: create ratio between degrees and potentiometer values
    }

    public void setTurretYawPower(double power) {
        turretYawAdjuster.set(power);
    }

    // public void setHoodBackPosition() {
    // double error = Const.HOOD_BACK - yawPotentiometer.get();
    // double output = Const.HOOD_Kp * error;
    // double final_output = Math.min(Math.max(output, Const.HOOD_MINOUTPUT),
    // Const.HOOD_MAXOUTPUT);
    // System.out.println("final_output: " + final_output);
    // this.setHoodPower(final_output);
    // }

    // public void setHoodFrontPosition() {
    // double error = Const.HOOD_FRONT - yawPotentiometer.get();
    // double output = Const.HOOD_Kp * error;
    // double final_output = Math.min(Math.max(output, Const.HOOD_MINOUTPUT),
    // Const.HOOD_MAXOUTPUT);
    // this.setHoodPower(final_output);
    // System.out.println("final_output: " + final_output);
    // }

    // public void setHoodMiddlePosition() {
    // double error = Const.HOOD_MIDDLE - yawPotentiometer.get();
    // double output = Const.HOOD_Kp * error;
    // double final_output = Math.min(Math.max(output, Const.HOOD_MINOUTPUT),
    // Const.HOOD_MAXOUTPUT);
    // this.setHoodPower(final_output);
    // System.out.println("final_output: " + final_output);
    // }

    public void resetShooterTicks() {
        shooterMaster.setSelectedSensorPosition(0, 0, 10);
        shooterSlave.setSelectedSensorPosition(0, 0, 10);
    }
}