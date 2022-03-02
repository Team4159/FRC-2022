package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain extends SubsystemBase {
    private WPI_TalonFX rightFrontTalon;
    private WPI_TalonFX rightRearTalon;
    private WPI_TalonFX leftFrontTalon;
    private WPI_TalonFX leftRearTalon;
    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;

    private PigeonIMU pigeon;

    private final DifferentialDrive drive;
    //private final DifferentialDriveOdometry odometry;
    private DifferentialDriveKinematics kinematics;
    private DifferentialDriveOdometry odometry;
    private SimpleMotorFeedforward feedforward;
    private Pose2d pose;

    private PIDController leftPID, rightPID;

    public Drivetrain() {
        rightFrontTalon = new WPI_TalonFX(Constants.CanIds.rightFrontTalon);
        rightRearTalon = new WPI_TalonFX(Constants.CanIds.rightRearTalon);
        leftFrontTalon = new WPI_TalonFX(Constants.CanIds.leftFrontTalon);
        leftRearTalon = new WPI_TalonFX(Constants.CanIds.leftRearTalon);

        leftMotors = new MotorControllerGroup(leftFrontTalon, leftRearTalon);
        rightMotors = new MotorControllerGroup(rightFrontTalon, rightRearTalon);
        

        //TODO: Need to See Which Ones Are Inverted

        drive = new DifferentialDrive(leftMotors, rightMotors);

        pigeon = new PigeonIMU(Constants.CanIds.pigeonId);

        // TODO: Need to See Which Ones Are Inverted
        leftMotors.setInverted(true);
        rightMotors.setInverted(false);


        //Kinematics parameter is the distance between the wheels aka track width.
        kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(23d));
        feedforward = new SimpleMotorFeedforward(DriveTrainConstants.kS, DriveTrainConstants.kV, DriveTrainConstants.kA);
        odometry = new DifferentialDriveOdometry(getHeading());
        leftPID = new PIDController(DriveTrainConstants.kP, DriveTrainConstants.kI, DriveTrainConstants.kD);
        rightPID = new PIDController(DriveTrainConstants.kP, DriveTrainConstants.kI, DriveTrainConstants.kD);
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
        pose = odometry.update(getHeading(), getLeftPosition(), getRightPosition());
    }
    

    public void zeroSensors() {
        zeroEncoders();
    }

    public void zeroEncoders() {

    }

    public MotorControllerGroup getLeftMotors() {
        return leftMotors;
    }

    public MotorControllerGroup getRightMotors() {
        return rightMotors;
    }

    //Return Position from Integrated Encoders from TalonFXs 

    /*
        Need to Check How Accurate This Is
    */
    public double getLeftPosition() {
        return leftFrontTalon.getSelectedSensorPosition() * Constants.DriveTrainConstants.metersPerRev;
    }

    public double getRightPosition() {
        return rightFrontTalon.getSelectedSensorPosition() * Constants.DriveTrainConstants.metersPerRev;
    }

    public double getRightVelocity() {
        return rightFrontTalon.getSelectedSensorVelocity();
    }

    public double getLeftVelocity() {
        return leftFrontTalon.getSelectedSensorVelocity();
    }

    
    //Use Pigeon to get angle
    public Rotation2d getHeading(){
        //Need to double check the reading from this later.
        return Rotation2d.fromDegrees(pigeon.getCompassHeading() * -1);
    }

    public DifferentialDriveKinematics getDifferentialDriveKinematics(){
        return kinematics;
    }
    
    public Pose2d getPose(){
        return pose;
    }

    public SimpleMotorFeedforward getMotorFeedForward(){
        return feedforward;
    }
    
    public DifferentialDriveWheelSpeeds getVelocities(){
        //I suspect we might have to do RPM motor to RPM wheel before converting to m/time
        //So we might need gear ratios? => sensorVelocity / GEAR_RATIO *2PI * m/min / 60s
        return new DifferentialDriveWheelSpeeds(
            leftFrontTalon.getSelectedSensorVelocity(),
            rightFrontTalon.getSelectedSensorVelocity()
        );
    }
    public PIDController getLeftPIDController(){return leftPID;}
    public PIDController getRightPIDController(){return rightPID;}
}

