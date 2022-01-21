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

    public Arm() {
        armSpark = new CANSparkMax(Constants.CanIds.armSpark, MotorType.kBrushless);
        encoder = new Encoder(
            Constants.IntakeAndArmConstants.encoderChannelA,
            Constants.IntakeAndArmConstants.encoderChannelB,
            Constants.IntakeAndArmConstants.encoderReverse,
            Constants.IntakeAndArmConstants.encodingType
        );

        pid = new PIDController(
            Constants.IntakeAndArmConstants.kP, 
            Constants.IntakeAndArmConstants.kI, 
            Constants.IntakeAndArmConstants.kD
        );

    }

    public void setArmSpeed(double speed) {
        armSpark.set(speed);
    }

    public void stopArm() {
        armSpark.set(0);
    }

    public CANSparkMax getArmSpark() {
        return armSpark;
    }

    public int getEncoderRaw() {
        return encoder.getRaw();
    }

    public double calculatePID(double encoderRaw, int setPoint) {
        return pid.calculate(encoderRaw, setPoint);
    }

    public void resetPID() {
        pid.reset();
    }

    public void close() {
        armSpark.close();
        encoder.close();
        pid.close();
    }

}
