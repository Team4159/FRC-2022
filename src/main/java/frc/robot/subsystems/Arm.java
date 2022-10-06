package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;

// intake "arm"
public class Arm extends SubsystemBase{
    private CANSparkMax armSpark;
    private RelativeEncoder encoder;
    private PIDController pid;
    
    public static enum ArmState {
        LOW,
        HIGH
    }
    
    private ArmState armState;

    private double lowSetPoint, highSetPoint;

    public Arm() {
        armSpark = new CANSparkMax(Constants.CanIds.armSpark, MotorType.kBrushless);
        armSpark.setInverted(false);

        encoder = armSpark.getEncoder();

        lowSetPoint = Constants.IntakeAndArmConstants.pidLowSetPoint;
        highSetPoint = Constants.IntakeAndArmConstants.pidHighSetPoint;

        pid = new PIDController(
            Constants.IntakeAndArmConstants.kP, 
            Constants.IntakeAndArmConstants.kI, 
            Constants.IntakeAndArmConstants.kD
        );

        armState = ArmState.HIGH;
    }

    public void setArmSpeed(double speed) {
        if (speed >= 0.6) {
            speed = 0.6;
        } else if (speed <= -0.6) {
            speed = -0.6;
        }

        armSpark.set(speed);
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }

    public double calculatePID(double encoderRaw, double setPoint) {
        if (atSetpoint(setPoint, Constants.IntakeAndArmConstants.tolerance)) {
            return 0;
        } else {
            return pid.calculate(getEncoderPosition(), setPoint);
        }
    }
    @Override
    public void periodic() {
        switch (armState) {
            case HIGH:
                setArmSpeed(calculatePID(getEncoderPosition(), highSetPoint));
                break;
            case LOW:
                setArmSpeed(calculatePID(getEncoderPosition(), lowSetPoint));
                break;
        }
    }

    public void runArm(ArmState armState) {
        this.armState = armState;
    }

    public void resetPID() {
        pid.reset();
    }

    public void zeroArm() {
        resetPID();
        armState = ArmState.HIGH;
        runArm(armState);
    }

    public boolean atSetpoint(Double setpoint, Double tolerance) {
        return armSpark.getEncoder().getPosition() <= setpoint + tolerance && armSpark.getEncoder().getPosition() >= setpoint - tolerance;
    }

    public void close() {
        armSpark.close();
        pid.close();
    }

}