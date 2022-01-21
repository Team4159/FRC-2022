package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;

public class Intake extends SubsystemBase {
    private CANSparkMax intakeSpark;

    public Intake() {
        intakeSpark = new CANSparkMax(Constants.CanIds.intakeSpark, MotorType.kBrushless);
    }

    public void runIntake() {
        intakeSpark.set(Constants.IntakeAndArmConstants.intakeSpeed);
    }
    public void runIntakeBackwards() {
        intakeSpark.set(Constants.IntakeAndArmConstants.backwardsIntakeSpeed);
    }
    public void stopIntake() {
        intakeSpark.set(0);
    }

    public CANSparkMax getIntakeSpark() {
        return intakeSpark;
    }

    public void close() {
        intakeSpark.close();
    }

}
