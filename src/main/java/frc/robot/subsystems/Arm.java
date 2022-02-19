package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    public static enum ArmState {
        LOW, HIGH
    }

    private CANSparkMax[] armSparkMotors = new CANSparkMax[2];
    private MotorControllerGroup motor;
    private ArmState state;

    private PIDController pid;
    private Encoder encoder;
    private int lowSetPoint, highSetPoint;

    public Arm() {
        this.armSparkMotors[0] = new CANSparkMax(Constants.CanIds.armSpark1, MotorType.kBrushless);
        this.armSparkMotors[1] = new CANSparkMax(Constants.CanIds.armSpark2, MotorType.kBrushless);
        this.armSparkMotors[0].setInverted(true); // TODO: CHECK WHICH ONES ARE INVERTED
        this.motor = new MotorControllerGroup(this.armSparkMotors[0], this.armSparkMotors[1]);

        encoder = new Encoder(
                Constants.ArmConstants.encoderChannelA,
                Constants.ArmConstants.encoderChannelB,
                Constants.ArmConstants.encoderReverse,
                Constants.ArmConstants.encodingType);
        pid = new PIDController(
                Constants.ArmConstants.kP,
                Constants.ArmConstants.kI,
                Constants.ArmConstants.kD);
    }

    public void runArm(ArmState armState) {
        this.state = armState;
        switch (this.state) {
            case HIGH:
                motor.set(calculatePID(getEncoderRaw(), Constants.ArmConstants.pidHighSetPoint));
                break;
            case LOW:
                motor.set(calculatePID(getEncoderRaw(), Constants.ArmConstants.pidLowSetPoint));
                break;
        }
    }

    public void setDirect(double speed) {
        motor.set(speed);
    }

    public void zeroArm() {
        pid.reset();
        state = ArmState.HIGH;
        runArm(state);
    }

    public double calculatePID(double encoderRaw, int setPoint) {
        return pid.calculate(encoderRaw, setPoint);
    }

    public void resetPID() {
        pid.reset();
    }

    public int getEncoderRaw() {
        return this.encoder.getRaw();
    }

    public void stop() {
        this.motor.set(0.0f);
    }

    public MotorControllerGroup getMotor() {
        return this.motor;
    }

    public void close() {
        this.motor.close();
        this.armSparkMotors[0].close();
        this.armSparkMotors[1].close();
        encoder.close();
        pid.close();
    }
}