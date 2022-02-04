package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    private CANSparkMax climberSparkMotorOne;
    private CANSparkMax climberSparkMotorTwo;

    private DigitalInput topLimitSwitch;
    private DigitalInput bottomLimitSwitch;

    private PIDController pid;
    private Encoder encoder;

    private int lowSetPoint, highSetPoint;

    public static enum ClimberState {
        RAISE,
        LOWER
    }

    private MotorControllerGroup climberSparks;
    private ClimberState climberState;

    public Climber() {
        climberSparkMotorOne = new CANSparkMax(Constants.CanIds.climberSpark1, MotorType.kBrushless);
        climberSparkMotorTwo = new CANSparkMax(Constants.CanIds.climberSpark2, MotorType.kBrushless);

        topLimitSwitch = new DigitalInput(Constants.ClimberConstants.topLimitSwitchChannel);
        bottomLimitSwitch = new DigitalInput(Constants.ClimberConstants.bottomLimitSwitchChannel);

        climberSparks = new MotorControllerGroup(climberSparkMotorOne, climberSparkMotorTwo);

        encoder = new Encoder(
            Constants.ClimberConstants.encoderChannelA,
            Constants.ClimberConstants.encoderChannelB,
            Constants.ClimberConstants.encoderReverse,
            Constants.ClimberConstants.encodingType
        );

        pid = new PIDController(
            Constants.ClimberConstants.kP,
            Constants.ClimberConstants.kI,
            Constants.ClimberConstants.kD
        );

        lowSetPoint = Constants.ClimberConstants.pidLowSetPoint;
        lowSetPoint = Constants.ClimberConstants.pidHighSetPoint;
        
        climberState = ClimberState.LOWER;
    }

    public void setClimberSpeed(double speed) {
        if (speed > 0) {
            if (topLimitSwitch.get()) {
                climberSparks.set(0);
            }
            else {
                climberSparks.set(speed);
            }
        } else {
            if (bottomLimitSwitch.get()) {
                climberSparks.set(0.1);
            }
            else {
                climberSparks.set(speed);
            }
        }
    }

    public double calculatePID(double encoderRaw, int setPoint) {
        return pid.calculate(encoderRaw, setPoint);
    }

    public void resetPID() {
        pid.reset();
    }

    public MotorControllerGroup getClimberGroup() {
        return climberSparks;
    }

    public int getEncoderRaw() {
        return encoder.getRaw();
    }

    public void runClimber(ClimberState climberState) {
        this.climberState = climberState;
        switch (this.climberState) {
            case RAISE:
                setClimberSpeed(calculatePID(getEncoderRaw(), highSetPoint));
                break;
            case LOWER:
                setClimberSpeed(calculatePID(getEncoderRaw(), lowSetPoint));
                break;
        }
    }

    public void close() {
        climberSparks.close();
        climberSparkMotorOne.close();
        climberSparkMotorTwo.close();
        encoder.close();
        pid.close();
        topLimitSwitch.close();
        bottomLimitSwitch.close();
    }

}
