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
import frc.robot.Constants.Direction;


public class Climber extends SubsystemBase{
    private CANSparkMax climberSparkMotorOne;
    private CANSparkMax climberSparkMotorTwo;

    public WPI_TalonFX climberTalonOne;
    public WPI_TalonFX climberTalonTwo;
       
    private PIDController climberElevatorPID;

    private int elevatorLowSetPoint, elevatorHighSetPoint;

    private MotorControllerGroup climberSparks;
    private MotorControllerGroup climberTalons;

    private Encoder encoder;

    public static enum ClimberState {
        RAISE,
        LOWER
    }

    private ClimberState elevatorClimberState;

    public Climber() {
        climberSparkMotorOne = new CANSparkMax(Constants.CanIds.climberSpark1, MotorType.kBrushless);
        climberSparkMotorTwo = new CANSparkMax(Constants.CanIds.climberSpark2, MotorType.kBrushless);

        climberSparks = new MotorControllerGroup(climberSparkMotorOne, climberSparkMotorTwo);

        climberElevatorPID = new PIDController(
            Constants.ClimberConstants.elevatorKP,
            Constants.ClimberConstants.elevatorKI,
            Constants.ClimberConstants.elevatorKD
        );
        
        climberTalonOne = new WPI_TalonFX(Constants.CanIds.climberTalon1);
        climberTalonTwo = new WPI_TalonFX(Constants.CanIds.climberTalon2);

        climberTalons = new MotorControllerGroup(climberTalonOne, climberTalonTwo);

        climberSparkMotorOne.setInverted(false);
        climberSparkMotorTwo.setInverted(true);
        climberTalonOne.setInverted(false);
        climberTalonTwo.setInverted(true);
        //climberTalonTwo.follow(climberTalonOne);

        climberTalonOne.config_kP(Constants.ClimberConstants.kPIDLoopIdx, Constants.ClimberConstants.elevatorKP);
        climberTalonOne.config_kI(Constants.ClimberConstants.kPIDLoopIdx, Constants.ClimberConstants.elevatorKI);
        climberTalonOne.config_kD(Constants.ClimberConstants.kPIDLoopIdx, Constants.ClimberConstants.elevatorKD);
        climberTalonOne.config_kF(Constants.ClimberConstants.kPIDLoopIdx, Constants.ClimberConstants.elevatorKF);

        elevatorLowSetPoint = Constants.ClimberConstants.elevatorLowSetPoint;
        elevatorHighSetPoint = Constants.ClimberConstants.elevatorHighSetPoint;
        
        elevatorClimberState = ClimberState.LOWER;

        climberTalonOne.setSelectedSensorPosition(0);
    }

    public void setArmSpeed(double speed) {
        climberSparks.set(speed);
        System.out.println("Climber arm speed:" + speed);
    }


    public double calculatePID(PIDController pid, double encoderRaw, int setPoint) {

        if (atSetpoint(setPoint, 0)) {
            return 0;
        } 
        else {
            return pid.calculate(encoderRaw, setPoint);
        }
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

    public void runClimberElevator(ClimberState climberState) {
        this.elevatorClimberState = climberState;
    }

    public boolean atSetpoint(int setpoint, int tolerance) {
        return getElevatorEncoders() <= setpoint + tolerance && getElevatorEncoders() >= setpoint - tolerance;
    }

    public MotorControllerGroup getClimberGroup() {
        return climberSparks;
    }

    public Encoder getEncoder() {
        return encoder;
    }

    public void zeroClimber() {
        elevatorClimberState = ClimberState.LOWER;
        runClimberElevator(ClimberState.LOWER);
    }

    public void close() {
        climberSparks.close();
        climberSparkMotorOne.close();
        climberSparkMotorTwo.close();
    }

}
