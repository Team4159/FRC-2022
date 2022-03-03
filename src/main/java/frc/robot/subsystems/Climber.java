package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase{
    private CANSparkMax climberSparkMotorOne;
    private CANSparkMax climberSparkMotorTwo;

    private PIDController pid;
    private RelativeEncoder encoder;

    private int lowSetPoint, highSetPoint;

    private MotorControllerGroup climberSparks;

    public static enum ClimberState {
        RAISE,
        LOWER
    }
    private ClimberState climberState;

    public Climber() {
        climberSparkMotorOne = new CANSparkMax(Constants.CanIds.climberSpark1, MotorType.kBrushless);
        climberSparkMotorTwo = new CANSparkMax(Constants.CanIds.climberSpark2, MotorType.kBrushless);

        climberSparks = new MotorControllerGroup(climberSparkMotorOne, climberSparkMotorTwo);

        encoder = climberSparkMotorOne.getEncoder();

        pid = new PIDController(
            Constants.ClimberConstants.kP,
            Constants.ClimberConstants.kI,
            Constants.ClimberConstants.kD
        );

        lowSetPoint = Constants.ClimberConstants.pidLowSetPoint;
        highSetPoint = Constants.ClimberConstants.pidHighSetPoint;
        
        climberState = ClimberState.LOWER;
    }

    public void setClimberSpeed(double speed) {
        climberSparks.set(speed);
    }

    public double calculatePID(double encoderRaw, int setPoint) {
        return pid.calculate(encoderRaw, setPoint);
    }

    public double getEncoderRaw() {
        return encoder.getPosition();
    }
    public int getLowSetPoint() {
        return this.lowSetPoint;
    }

    public int getHighSetPoint() {
        return this.highSetPoint;
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

    public void resetPID() {
        pid.reset();
    }

    public MotorControllerGroup getClimberGroup() {
        return climberSparks;
    }

    public void zeroClimber() {
        pid.reset();
        climberState = ClimberState.LOWER;
        runClimber(ClimberState.LOWER);
    }

    public void close() {
        climberSparks.close();
        climberSparkMotorOne.close();
        climberSparkMotorTwo.close();
        pid.close();
    }

}
