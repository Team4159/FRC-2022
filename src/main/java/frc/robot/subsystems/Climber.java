package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase{
    private CANSparkMax climberSparkMotorOne;
    private CANSparkMax climberSparkMotorTwo;

    public WPI_TalonFX climberTalonOne;
    public WPI_TalonFX climberTalonTwo;

    private PIDController climberArmPid;
    private PIDController climberElevatorPID;
    private Encoder climberArmEncoder;

    private int armLowSetPoint, armHighSetPoint;
    private int elevatorLowSetPoint, elevatorHighSetPoint;

    private MotorControllerGroup climberSparks;
    private MotorControllerGroup climberTalons;

    private Encoder encoder;

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
        ); ;

        climberArmPid = new PIDController(
            Constants.ClimberConstants.armKP,
            Constants.ClimberConstants.armKI,
            Constants.ClimberConstants.armKD
        );

        climberElevatorPID = new PIDController(
            Constants.ClimberConstants.elevatorKP,
            Constants.ClimberConstants.elevatorKI,
            Constants.ClimberConstants.elevatorKD
        );
        
        climberTalonOne = new WPI_TalonFX(Constants.CanIds.climberTalon1);
        climberTalonTwo = new WPI_TalonFX(Constants.CanIds.climberTalon2);

        climberTalons = new MotorControllerGroup(climberTalonOne, climberTalonTwo);


        climberTalonOne.setInverted(false);
        climberTalonTwo.setInverted(true);
        //climberTalonTwo.follow(climberTalonOne);

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

        climberTalonOne.setSelectedSensorPosition(0);
    }

    public void setClimberSpeed(double speed) {
        climberSparks.set(speed);
    }

    public double calculatePID(PIDController pid, double encoderRaw, int setPoint) {

        if (atSetpoint(setPoint, 0)) {
            return 0;
        } 
        else {
            return pid.calculate(encoderRaw, setPoint);
        }
    }



    public double getArmEncoderRaw() {
        return climberArmEncoder.getRaw();
    }

    public double getElevatorEncoders() {
        return climberTalonOne.getSelectedSensorPosition();
    }

    
    public void teleopInit() {
        climberElevatorPID.reset();
    }

    
    public void autonomousInit() {
        climberElevatorPID.reset();
    }


    @Override
    public void periodic() {
        switch (armClimberState) {
            case RAISE:
                setClimberSpeed(calculatePID(climberArmPid, getArmEncoderRaw(), armHighSetPoint));
                break;
            case LOWER:
                setClimberSpeed(calculatePID(climberArmPid, getArmEncoderRaw(), armLowSetPoint));
                break;
        }
        switch (elevatorClimberState) {
            case RAISE:
                //climberTalonOne.set(ControlMode.Position, elevatorHighSetPoint);
                System.out.println(calculatePID(climberElevatorPID, getElevatorEncoders(), elevatorHighSetPoint));
                climberTalons.set(calculatePID(climberElevatorPID, getElevatorEncoders(), elevatorHighSetPoint));
                break;
            case LOWER:
                //climberTalonOne.set(ControlMode.Position, elevatorLowSetPoint);
                climberTalons.set(calculatePID(climberElevatorPID, getElevatorEncoders(), elevatorLowSetPoint));
                break;
        }
    }

    public void runClimberArm(ClimberState climberState) {
        this.armClimberState = climberState;
    }

    public void runClimberElevator(ClimberState climberState) {
        this.elevatorClimberState = climberState;
    }

    public boolean atSetpoint(int setpoint, int tolerance) {
        return getElevatorEncoders() <= setpoint + tolerance && getElevatorEncoders() >= setpoint - tolerance;
    }


    public void resetPID() {
        climberArmPid.reset();
    }

    public MotorControllerGroup getClimberGroup() {
        return climberSparks;
    }

    public Encoder getEncoder() {
        return encoder;
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
