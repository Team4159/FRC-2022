package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
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
       
    private PIDController climberElevatorPID1;
    private PIDController climberElevatorPID2;

    private ElevatorFeedforward climberFeedforward1;
    private ElevatorFeedforward climberFeedforward2;


    private int elevatorLowSetPoint, elevatorHighSetPoint;

    private MotorControllerGroup climberSparks;
    private MotorControllerGroup climberTalons;

    private Encoder encoder;

    public static enum ClimberState {
        RAISE,
        LOWER
    }
    public void brakeMode() {
        // BRAKING MODE ATTEMPT :D //
        // BREAK CAUSES DECELERATION, COAST MEANS MOTOT SPINS IN LAST DIRECTION DRIVEN 
        climberTalonOne.setNeutralMode(NeutralMode.Brake);
        climberTalonTwo.setNeutralMode(NeutralMode.Brake);
    }

    public void coastMode() {
        climberTalonOne.setNeutralMode(NeutralMode.Coast);
        climberTalonTwo.setNeutralMode(NeutralMode.Coast);

    }

    private ClimberState elevatorClimberState;

    public Climber() {
        climberSparkMotorOne = new CANSparkMax(Constants.CanIds.climberSpark1, MotorType.kBrushless);
        climberSparkMotorTwo = new CANSparkMax(Constants.CanIds.climberSpark2, MotorType.kBrushless);

        climberSparks = new MotorControllerGroup(climberSparkMotorOne, climberSparkMotorTwo);

        climberElevatorPID1 = new PIDController(
            Constants.ClimberConstants.elevatorKP,
            Constants.ClimberConstants.elevatorKI,
            Constants.ClimberConstants.elevatorKD
        );

        climberElevatorPID2 = new PIDController(
            Constants.ClimberConstants.elevatorKP,
            Constants.ClimberConstants.elevatorKI,
            Constants.ClimberConstants.elevatorKD
        );
        
        climberFeedforward1 = new ElevatorFeedforward(
            Constants.ClimberConstants.kS, 
            Constants.ClimberConstants.kG, 
            Constants.ClimberConstants.kV, 
            Constants.ClimberConstants.kA
        );

        climberFeedforward2 = new ElevatorFeedforward(
            Constants.ClimberConstants.kS, 
            Constants.ClimberConstants.kG, 
            Constants.ClimberConstants.kV, 
            Constants.ClimberConstants.kA
        );       

        climberTalonOne = new WPI_TalonFX(Constants.CanIds.climberTalon1);
        climberTalonTwo = new WPI_TalonFX(Constants.CanIds.climberTalon2);

        climberTalons = new MotorControllerGroup(climberTalonOne, climberTalonTwo);

        climberSparkMotorOne.setInverted(false);
        climberSparkMotorTwo.setInverted(true);
        climberTalonOne.setInverted(true);
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
        //System.out.println("Climber arm speed:" + speed);
    }


    public double calculatePID(PIDController pid, double encoderRaw, int setPoint) {

        if (atSetpoint(setPoint, 0)) {
            return 0;
        } 
        else {
            return pid.calculate(encoderRaw, setPoint);
        }
    }

    public double calculateFeedforward(ElevatorFeedforward feedForward, int setPoint) {
        return feedForward.calculate(setPoint);
    }

    public double getElevatorEncoders() {
        return climberTalonOne.getSelectedSensorPosition();
    }

    
    public void teleopInit() {
        climberElevatorPID1.reset();
        climberElevatorPID2.reset();
    }

    
    public void autonomousInit() {
        climberElevatorPID1.reset();
        climberElevatorPID2.reset();
    }


    @Override
    public void periodic() {
        switch (elevatorClimberState) {
            case RAISE:
                climberTalonOne.set(ControlMode.Position, elevatorHighSetPoint);
                // System.out.println(calculatePID(climberElevatorPID, getElevatorEncoders(), elevatorHighSetPoint));
                climberTalons.setVoltage(calculatePID(climberElevatorPID1, getElevatorEncoders(), elevatorHighSetPoint) + calculateFeedforward(climberFeedforward1, elevatorHighSetPoint));
                // fix the setPoint; random int 5 :(
                // cole can't code feed forward
                // add PID2 later??
                break;
            case LOWER:
                //climberTalonOne.set(ControlMode.Position, elevatorLowSetPoint);
                climberTalons.setVoltage(calculatePID(climberElevatorPID1, getElevatorEncoders(), elevatorLowSetPoint));
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
