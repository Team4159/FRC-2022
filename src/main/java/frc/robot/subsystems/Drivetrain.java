package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

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

  public static enum PowerOutput {
    FULL_POWER,
    HALF_POWER
  }

  public static enum Orientation {
    FORWARD,
    BACKWARD
  }

  public WPI_TalonFX rightFrontTalon;
  private WPI_TalonFX rightRearTalon;
  private WPI_TalonFX leftFrontTalon;
  private WPI_TalonFX leftRearTalon;
  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;

  private WPI_PigeonIMU pigeon;
  private PowerOutput powerOutput;
  public Orientation orientation;
  private boolean climbState;

  private DifferentialDriveKinematics kinematics;
  private Pose2d pose;
  private DifferentialDriveOdometry odometry;
  private SimpleMotorFeedforward feedforward;

  private PIDController linearDriveTrainPID = new PIDController(Constants.DriveTrainConstants.lP, Constants.DriveTrainConstants.lI, Constants.DriveTrainConstants.lD);
  private PIDController angularDriveTrainPID = new PIDController(Constants.DriveTrainConstants.aP, Constants.DriveTrainConstants.aI, Constants.DriveTrainConstants.aD);

  public Drivetrain() {
    rightFrontTalon = new WPI_TalonFX(Constants.CanIds.rightFrontTalon);
    rightRearTalon = new WPI_TalonFX(Constants.CanIds.rightRearTalon);
    leftFrontTalon = new WPI_TalonFX(Constants.CanIds.leftFrontTalon);
    leftRearTalon = new WPI_TalonFX(Constants.CanIds.leftRearTalon);

    leftMotors = new MotorControllerGroup(leftRearTalon, leftFrontTalon);
    rightMotors = new MotorControllerGroup(rightFrontTalon, rightRearTalon);

    //leftMotors.setInverted(true);
    
    pigeon = new WPI_PigeonIMU(Constants.CanIds.pigeonId);

    powerOutput = powerOutput.FULL_POWER;
    orientation = orientation.FORWARD;
    climbState = false;

    kinematics = new DifferentialDriveKinematics(Constants.DriveTrainConstants.trackWidth);
    feedforward = new SimpleMotorFeedforward(Constants.DriveTrainConstants.ksVolts, Constants.DriveTrainConstants.kvVoltSecondsPerMeter, Constants.DriveTrainConstants.kaVoltSecondsSquaredPerMeter);
    odometry = new DifferentialDriveOdometry(getHeading());
  }

  public void teleopInit() {
  }

  public void autoClimb() {
    climbState = true;
  }
  public void stopClimb() {
    climbState = false;
  }

  public void drive(double leftSpeed, double rightSpeed) {
    leftMotors.setInverted(true);

    if (climbState) {
      leftMotors.set(0.125);
      rightMotors.set(0.125);
    } else {
      if (orientation == Orientation.BACKWARD) {
        double temp = leftSpeed * -1;
        leftSpeed = rightSpeed * -1;
        rightSpeed = temp;
      }
      if (powerOutput == PowerOutput.FULL_POWER) {
        leftMotors.set(leftSpeed);
        rightMotors.set(rightSpeed);
      } else {
        leftMotors.set(leftSpeed * 0.5);
        rightMotors.set(rightSpeed * 0.5);
      }
    }
  }

  public void halfPower() {
    powerOutput = PowerOutput.HALF_POWER;
  }

  public void fullPower() {
    powerOutput = PowerOutput.FULL_POWER;
  }

  public void flipDrivetrain() {
    if (orientation == Orientation.FORWARD) {
      orientation = Orientation.BACKWARD;
    } else {
      orientation = Orientation.FORWARD;
    }
  }

  public void stop(){
    leftMotors.set(0);
    rightMotors.set(0);
  }

  public void zeroSensors() {
    leftFrontTalon.setSelectedSensorPosition(0);
    rightFrontTalon.setSelectedSensorPosition(0);
    pigeon.setFusedHeading(0);
  }

  public void moveDistance(double distance) {
    double output = linearDriveTrainPID.calculate(getRobotPosition(), distance); //Negative
    if(output > 0.2 || output < -0.2) {
      output = 0.2;
    }
    leftMotors.set(output);
    rightMotors.set(-output); //Positive
  }

  public boolean atDistanceSetpoint(double distance, double tolerance) {
    return getRobotPosition() <= distance + tolerance && getRobotPosition() >= distance - tolerance;
  }
  public WPI_TalonFX getLeftTalon() {
    return leftFrontTalon;
  }
  public WPI_TalonFX getRightTalon() {
    return rightFrontTalon;
  }

  public void turnDegrees(double angle) { 
    
    double output = angularDriveTrainPID.calculate(getAngle(), angle); // - 
    if(output > 0.2 || output < -0.2) {
      output = 0.2;
    }

    leftMotors.set(output);
    rightMotors.set(output);
  }

  public boolean atAngleSetpoint(double angle, double tolerance) {
    return getAngle() <= angle + tolerance && getAngle() >= angle - tolerance;
  }

  public double getAngle() {
    return pigeon.getAngle();
  }

  public Rotation2d getHeading() {
      return Rotation2d.fromDegrees(-pigeon.getAngle());
  }
  
  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getLeftVelocity(),
        getRightVelocity()
    );
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public void setOutputVolts(double leftVolts, double rightVolts) {
    leftMotors.set(leftVolts / 12);
    rightMotors.set(rightVolts / 12);
  }

  public void reset() {
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  @Override
  public void periodic() {
    pose = odometry.update(getHeading(), getLeftPosition(), getRightPosition());
  }

  public double getRobotPosition() {
    return (getLeftPosition() + getRightPosition())/2;
  }

  public double getLeftPosition() {
      return leftFrontTalon.getSelectedSensorPosition() * Constants.DriveTrainConstants.wheelCircumference / (Constants.DriveTrainConstants.gearRatio * Constants.DriveTrainConstants.encoderEdgesPerRev);
  }

  public double getRightPosition() {
      return -rightFrontTalon.getSelectedSensorPosition() * Constants.DriveTrainConstants.wheelCircumference / (Constants.DriveTrainConstants.gearRatio * Constants.DriveTrainConstants.encoderEdgesPerRev);
  }

  public double getRightVelocity() {
      return rightFrontTalon.getSelectedSensorVelocity() *   Constants.DriveTrainConstants.wheelCircumference / (Constants.DriveTrainConstants.gearRatio * Constants.DriveTrainConstants.encoderEdgesPerRev);
  }

  public double getLeftVelocity() {
      return leftFrontTalon.getSelectedSensorVelocity() * Constants.DriveTrainConstants.wheelCircumference / (Constants.DriveTrainConstants.gearRatio * Constants.DriveTrainConstants.encoderEdgesPerRev);
  }

}

