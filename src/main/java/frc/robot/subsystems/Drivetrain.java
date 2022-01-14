package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    private WPI_TalonFX rightFrontTalon;
    private WPI_TalonFX rightRearTalon;
    private WPI_TalonFX leftFrontTalon;
    private WPI_TalonFX leftRearTalon;
    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;

    private Pigeon2 pigeon;


    public Drivetrain() {
        rightFrontTalon = new WPI_TalonFX(Constants.CanIds.rightFrontTalon);
        rightRearTalon = new WPI_TalonFX(Constants.CanIds.rightRearTalon);
        leftFrontTalon = new WPI_TalonFX(Constants.CanIds.leftFrontTalon);
        leftRearTalon = new WPI_TalonFX(Constants.CanIds.leftRearTalon);

        leftMotors = new MotorControllerGroup(leftFrontTalon, leftRearTalon);
        rightMotors = new MotorControllerGroup(rightFrontTalon, rightRearTalon);

        pigeon = new Pigeon2(Constants.CanIds.pigeonId);

        // TODO: Need to See Which Ones Are Inverted

        leftMotors.setInverted(true);
        rightMotors.setInverted(false);

        
    }

    public void drive(double leftSpeed, double rightSpeed) {
        leftMotors.set(leftSpeed);
        rightMotors.set(rightSpeed);
    }




    @Override
    public void periodic() {
        
    }
    

    public void zeroSensors() {
        zeroEncoders();
    }

    public void zeroEncoders() {
        leftFrontTalon.setSelectedSensorPosition(0);
        rightFrontTalon.setSelectedSensorPosition(0);
        leftRearTalon.setSelectedSensorPosition(0);
        rightRearTalon.setSelectedSensorPosition(0);
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
}
