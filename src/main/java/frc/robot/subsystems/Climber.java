package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase{
    private CANSparkMax climberSparkMotorOne;
    private CANSparkMax climberSparkMotorTwo;

    private WPI_TalonFX climberTalonOne;
    private WPI_TalonFX climberTalonTwo;

    private PIDController climberArmPid;
    private Encoder climberArmEncoder;

    private int armLowSetPoint, armHighSetPoint;
    private int elevatorLowSetPoint, elevatorHighSetPoint;

    private MotorControllerGroup climberSparks;

    public static enum ClimberState {
        RAISE,
        LOWER
    }
    private ClimberState armClimberState;
    private ClimberState elevatorClimberState;

    public Climber() {
        climberSparkMotorOne = new CANSparkMax(Constants.CanIds.climberSpark1, MotorType.kBrushless);
        climberSparkMotorTwo = new CANSparkMax(Constants.CanIds.climberSpark2, MotorType.kBrushless);

        climberSparks = new MotorControllerGroup(climberSparkMotorOne, climberSparkMotorTwo);

        climberArmEncoder = new Encoder(
            Constants.ClimberConstants.encoderChannelA,
            Constants.ClimberConstants.encoderChannelB,
            Constants.ClimberConstants.encoderReverse,
            Constants.ClimberConstants.encodingType
        ); climberSparkMotorOne.getEncoder();

        climberArmPid = new PIDController(
            Constants.ClimberConstants.armKP,
            Constants.ClimberConstants.armKI,
            Constants.ClimberConstants.armKD
        );
        
        climberTalonOne = new WPI_TalonFX(Constants.CanIds.climberTalon1);
        climberTalonTwo = new WPI_TalonFX(Constants.CanIds.climberTalon2);

        climberTalonTwo.follow(climberTalonOne);

        climberTalonOne.config_kP(Constants.ClimberConstants.kPIDLoopIdx, Constants.ClimberConstants.elevatorKP);
        climberTalonOne.config_kI(Constants.ClimberConstants.kPIDLoopIdx, Constants.ClimberConstants.elevatorKI);
        climberTalonOne.config_kD(Constants.ClimberConstants.kPIDLoopIdx, Constants.ClimberConstants.elevatorKD);
        climberTalonOne.config_kF(Constants.ClimberConstants.kPIDLoopIdx, Constants.ClimberConstants.elevatorKF);

        armLowSetPoint = Constants.ClimberConstants.armLowSetPoint;
        armHighSetPoint = Constants.ClimberConstants.armHighSetPoint;

        elevatorLowSetPoint = Constants.ClimberConstants.elevatorLowSetPoint;
        elevatorHighSetPoint = Constants.ClimberConstants.elevatorHighSetPoint;
        
        armClimberState = ClimberState.LOWER;
        elevatorClimberState = ClimberState.LOWER;
    }

    public void setClimberSpeed(double speed) {
        climberSparks.set(speed);
    }

    public double calculatePID(PIDController pid, double encoderRaw, int setPoint) {
        return pid.calculate(encoderRaw, setPoint);
    }

    public double getArmEncoderRaw() {
        return climberArmEncoder.getRaw();
    }

    public double getElevatorEncoders() {
        return climberTalonTwo.getSelectedSensorPosition();
    }

    public void runClimberArm(ClimberState climberState) {
        this.armClimberState = climberState;
        switch (this.armClimberState) {
            case RAISE:
                setClimberSpeed(calculatePID(climberArmPid, getArmEncoderRaw(), armHighSetPoint));
                break;
            case LOWER:
                setClimberSpeed(calculatePID(climberArmPid, getArmEncoderRaw(), armLowSetPoint));
                break;
        }
    }

    public void runClimberElevator(ClimberState climberState) {
        this.elevatorClimberState = climberState;
        switch (this.elevatorClimberState) {
            case RAISE:
                climberTalonOne.set(ControlMode.Position, elevatorHighSetPoint);
                break;
            case LOWER:
            climberTalonOne.set(ControlMode.Position, elevatorLowSetPoint);
                break;
        }
    }


    public void resetPID() {
        climberArmPid.reset();
    }

    public MotorControllerGroup getClimberGroup() {
        return climberSparks;
    }

    public void zeroClimber() {
        resetPID();
        armClimberState = ClimberState.LOWER;
        elevatorClimberState = ClimberState.LOWER;
        runClimberArm(ClimberState.LOWER);
        runClimberElevator(ClimberState.LOWER);
    }

    public void close() {
        climberSparks.close();
        climberSparkMotorOne.close();
        climberSparkMotorTwo.close();
        climberArmPid.close();
    }

}
