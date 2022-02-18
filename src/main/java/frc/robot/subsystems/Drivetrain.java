package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain extends SubsystemBase {
    private WPI_TalonFX rightFrontTalon;
    private WPI_TalonFX rightRearTalon;
    private WPI_TalonFX leftFrontTalon;
    private WPI_TalonFX leftRearTalon;
    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;

    private Pigeon2 pigeon;

    private Encoder rightEncoder;
    private Encoder leftEncoder;

    private DifferentialDrive drive;
    //private final DifferentialDriveOdometry odometry;

    public Drivetrain() {
        rightFrontTalon = new WPI_TalonFX(Constants.CanIds.rightFrontTalon);
        rightRearTalon = new WPI_TalonFX(Constants.CanIds.rightRearTalon);
        leftFrontTalon = new WPI_TalonFX(Constants.CanIds.leftFrontTalon);
        leftRearTalon = new WPI_TalonFX(Constants.CanIds.leftRearTalon);

        leftMotors = new MotorControllerGroup(leftFrontTalon, leftRearTalon);
        rightMotors = new MotorControllerGroup(rightFrontTalon, rightRearTalon);

        //drive = new DifferentialDrive(leftMotors, rightMotors);

        pigeon = new Pigeon2(Constants.CanIds.pigeonId);

        



        // TODO: Need to See Which Ones Are Inverted

        leftMotors.setInverted(true);
        rightMotors.setInverted(false);
        
    }

    public void drive(double leftSpeed, double rightSpeed) {
        leftMotors.set(leftSpeed);
        rightMotors.set(rightSpeed);
    }

    public void stop(){
        leftMotors.set(0);
        rightMotors.set(0);
    }

    @Override
    public void periodic() {
        
    }
    

    public void zeroSensors() {
        zeroEncoders();
    }

    public void zeroEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public MotorControllerGroup getLeftMotors() {
        return leftMotors;
    }

    public MotorControllerGroup getRightMotors() {
        return rightMotors;
    }

    //Return Position from Integrated Encoders from TalonFXs 

    //TODO: Check How Accurate Encoder Values Are

    public double getLeftPosition() {
        return leftEncoder.getDistance() * Constants.DriveTrainConstants.metersPerRev;
    }

    public double getRightPosition() {
        return rightEncoder.getDistance() * Constants.DriveTrainConstants.metersPerRev;
    }

}
