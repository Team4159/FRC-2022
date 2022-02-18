package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Direction;

public class Climber extends SubsystemBase {
    private CANSparkMax[] climberSparkMotors = new CANSparkMax[2];
    private MotorControllerGroup motor;
    private Direction direction;

    private PIDController pid;
    private Encoder encoder;
    private int lowSetPoint, highSetPoint;

    public Climber() {
        this.climberSparkMotors[0] = new CANSparkMax(Constants.CanIds.climberSpark1, MotorType.kBrushless);
        this.climberSparkMotors[1] = new CANSparkMax(Constants.CanIds.climberSpark2, MotorType.kBrushless);
        this.motor = new MotorControllerGroup(this.climberSparkMotors[0], this.climberSparkMotors[1]);

        this.encoder = new Encoder(
                Constants.ClimberConstants.encoderChannelA,
                Constants.ClimberConstants.encoderChannelB,
                Constants.ClimberConstants.encoderReverse,
                Constants.ClimberConstants.encodingType);
        this.pid = new PIDController(
                Constants.ClimberConstants.kP,
                Constants.ClimberConstants.kI,
                Constants.ClimberConstants.kD);
    }

    public void set(Direction direction) {
        this.direction = direction;
        switch (direction) {
            case FORWARDS:
                motor.set(calculatePID(getEncoderRaw(), Constants.ClimberConstants.pidHighSetPoint));
                break;
            case BACKWARDS:
                motor.set(calculatePID(getEncoderRaw(), Constants.ClimberConstants.pidLowSetPoint));
                break;
        }
    }

    public void setDirect(double speed) {
        motor.set(speed);
    }

    public void zeroClimber() {
        this.resetPID();
        direction = Direction.BACKWARDS;
        set(Direction.BACKWARDS);
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
        motor.close();
        this.climberSparkMotors[0].close();
        this.climberSparkMotors[1].close();
        encoder.close();
        pid.close();
    }
}
