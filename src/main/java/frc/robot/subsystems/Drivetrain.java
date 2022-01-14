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

    private double error;

    public void setSetpoint(double setpoint) { // In Meters

    }

    public double getLeftPosition() {
        return leftFrontTalon.getSelectedSensorPosition();
    }

    public double getRightPosition() {
        return rightFrontTalon.getSelectedSensorPosition();
    }

    public double getRightVelocity() {
        return rightFrontTalon.getSelectedSensorVelocity();
    }

    public double getLeftVelocity() {
        return leftFrontTalon.getSelectedSensorVelocity();
    }
}
