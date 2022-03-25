package frc.robot.subsystems;

import java.util.TooManyListenersException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.controller.PIDController;

public class Arm extends SubsystemBase{
    private CANSparkMax armSpark1;
    // private CANSparkMax armSpark2;
    private MotorControllerGroup armSparks;
    private Encoder encoder1, encoder2;
    private PIDController pid;
    
    public static enum ArmState {
        LOW,
        HIGH
    }
    
    private ArmState armState;

    private double lowSetPoint, highSetPoint;

    public Arm() {
        armSpark1 = new CANSparkMax(Constants.CanIds.armSpark1, MotorType.kBrushless);
        // armSpark2 = new CANSparkMax(Constants.CanIds.armSpark2, MotorType.kBrushless);

        //TODO: CHECK WHICH ONES ARE INVERTED
        armSpark1.setInverted(false);
        // armSpark2.setInverted(true);

        armSparks = new MotorControllerGroup(armSpark1);

        encoder2 = new Encoder(
            2,
            3
        );

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
        //System.out.println(speed);
        armSparks.set(speed);
    }

    public double getEncoderRaw() {
        //System.out.println("Encoder Average: " + (encoder1.getRaw()+encoder2.getRaw())/2);
        return encoder2.getRaw();
    }

    public Encoder getEncoder() {
        return encoder2;
    }

    public double calculatePID(double encoderRaw, double setPoint) {
        if (atSetpoint(setPoint, Constants.IntakeAndArmConstants.tolerance)) {
            return 0;
        } else {
            return pid.calculate(encoderRaw, setPoint);
        }
    }
    @Override
    public void periodic() {
        //System.out.println("Encoder: " + getEncoderRaw());
        switch (armState) {
            case HIGH:
                setArmSpeed(calculatePID(getEncoderRaw(), highSetPoint));
                break;
            case LOW:
                setArmSpeed(calculatePID(getEncoderRaw(), lowSetPoint));
                break;
        }
    }

    public void runArm(ArmState armState) {
        this.armState = armState;
    }

    public void resetPID() {
        pid.reset();
    }

    public MotorControllerGroup getArmSpark() {
        return armSparks;
    }

    public void zeroArm() {
        resetPID();
        encoder2.reset();
        armState = ArmState.HIGH;
        runArm(armState);
    }

    public boolean atSetpoint(Double setpoint, Double tolerance) {
        return getEncoderRaw() <= setpoint + tolerance && getEncoderRaw() >= setpoint - tolerance;
    }

    public void close() {
        armSpark1.close();
        // armSpark2.close();
        pid.close();
    }

}