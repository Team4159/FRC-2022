package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.PIDController;

public class Arm extends SubsystemBase{
    private CANSparkMax armSpark;
    //private CANSparkMax armSpark2;
    //private MotorControllerGroup armSparks;
    private Encoder encoder;
    private PIDController pid;
    
    public static enum ArmState {
        LOW,
        HIGH
    }
    
    private ArmState armState;

    private double lowSetPoint, highSetPoint;

    public Arm() {
        armSpark = new CANSparkMax(Constants.CanIds.armSpark, MotorType.kBrushless);
        //armSpark2 = new CANSparkMax(Constants.CanIds.armSpark2, MotorType.kBrushless);

        //TODO: CHECK WHICH ONES ARE INVERTED
        armSpark.setInverted(false);
        // armSpark2.setInverted(true);

        //armSparks = new MotorControllerGroup(armSpark1);

        encoder = new Encoder(
            0,
            1
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

        armSpark.set(speed);
    }

    public double getEncoderRaw() {
        return encoder.getRaw();
    }

    public Encoder getEncoder() {
        return encoder;
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

    // public MotorControllerGroup getArmSpark() {
    //     return armSparks;
    // }

    public void zeroArm() {
        resetPID();
        encoder.reset();
        armState = ArmState.HIGH;
        runArm(armState);
    }

    public boolean atSetpoint(Double setpoint, Double tolerance) {
        return getEncoderRaw() <= setpoint + tolerance && getEncoderRaw() >= setpoint - tolerance;
    }

    public void close() {
        armSpark.close();
        pid.close();
    }

}