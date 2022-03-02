package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;

public class Arm extends SubsystemBase{
    private CANSparkMax armSpark;
    private Encoder encoder;
    private PIDController pid;
    
    public static enum ArmState {
        LOW,
        HIGH
    }
    
    private ArmState armState;

    private int lowSetPoint, highSetPoint;

    public Arm() {
        armSpark = new CANSparkMax(Constants.CanIds.armSpark, MotorType.kBrushless);
        encoder = new Encoder(
            Constants.IntakeAndArmConstants.encoderChannelA,
            Constants.IntakeAndArmConstants.encoderChannelB,
            Constants.IntakeAndArmConstants.encoderReverse,
            Constants.IntakeAndArmConstants.encodingType
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
        armSpark.set(speed);
    }

    public int getEncoderRaw() {
        return encoder.getRaw();
    }

    public double calculatePID(double encoderRaw, int setPoint) {
        return pid.calculate(encoderRaw, setPoint);
    }

    public void runArm(ArmState armState) {
        this.armState = armState;
        switch (this.armState) {
            case HIGH:
                setArmSpeed(calculatePID(getEncoderRaw(), highSetPoint));
                break;
            case LOW:
                setArmSpeed(calculatePID(getEncoderRaw(), lowSetPoint));
                break;
        }
    }

    public void resetPID() {
        pid.reset();
    }

    public CANSparkMax getArmSpark() {
        return armSpark;
    }

    public void zeroArm() {
        pid.reset();
        armState = ArmState.HIGH;
        runArm(armState);
    }

    public void close() {
        armSpark.close();
        encoder.close();
        pid.close();
    }

}

