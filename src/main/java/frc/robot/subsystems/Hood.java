package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import frc.Const;

/**
 * Hood
 */
public class Hood {

    private CANSparkMax hoodMotor = new CANSparkMax(9, CANSparkMax.MotorType.kBrushless);
    CANEncoder encoder = hoodMotor.getEncoder();

    private double zeroValue;

    public Hood() {
        this.zeroValue = encoder.getPosition();
    }


    public void setPower(double power) {
        // if(encoder.getPosition() >= Const.HOOD_MAX_POSITION && power > 0) {
        //     System.out.println("too far back");
        //     power = 0;
        // } else if(encoder.getPosition() <= Const.HOOD_MIN_POSITION && power < 0) {
        //     power = 0;
        //     System.out.println("too far foward");
        //}
        // TODO: check these again :(

        power = Math.max(Math.min(power, Const.HOOD_MAXOUTPUT), Const.HOOD_MINOUTPUT);

        hoodMotor.set(power);
    }

    /**
     * sets the hood position using pid and also prohibits it from exceeding max ranges
     * positive power makes the hood shrink
     */
    public void setPosition(double position) {

        double error = this.getRelativePosition() - position;
        double power = Const.HOOD_Kp * error;
        // System.out.println("err " + error);
        setPower(power);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getRelativePosition() {
        return zeroValue - encoder.getPosition();
    }
}