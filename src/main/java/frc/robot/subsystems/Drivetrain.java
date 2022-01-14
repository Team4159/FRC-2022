package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CanIds;

public class Drivetrain extends SubsystemBase{
    private WPI_TalonFX rightFrontTalon;
    private WPI_TalonFX rightRearTalon;
    private WPI_TalonFX leftFrontTalon;
    private WPI_TalonFX leftRearTalon;
    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;

    public Drivetrain() {
        rightFrontTalon = new WPI_TalonFX(CanIds.rightFrontTalon);
        rightRearTalon = new WPI_TalonFX(CanIds.rightRearTalon);
        leftFrontTalon = new WPI_TalonFX(CanIds.leftFrontTalon);
        leftRearTalon = new WPI_TalonFX(CanIds.leftRearTalon);

        leftMotors = new MotorControllerGroup(leftFrontTalon, leftRearTalon);
        rightMotors = new MotorControllerGroup(rightFrontTalon, rightRearTalon);

        leftMotors.setInverted(true);
        rightMotors.setInverted(false);
    }

    public void drive(double leftSpeed, double rightSpeed) {
        leftMotors.set(leftSpeed);
        rightMotors.set(rightSpeed);
    }

    public double getSpeed() {
        return ((leftMotors.get() + rightMotors.get()) / 2);
    }

    public void close() throws Exception {
        leftMotors.close();
        rightMotors.close();
        leftFrontTalon.close();
        leftRearTalon.close();
        rightFrontTalon.close();
        rightRearTalon.close();
    }
}
