package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Direction;

public class Intake extends SubsystemBase {
    private CANSparkMax motor;

    public Intake() {
        this.motor = new CANSparkMax(Constants.CanIds.intakeSpark, MotorType.kBrushless);
    }

    public void set(Direction direction) {
        switch (direction) {
            case FORWARDS:
                this.motor.set(Constants.IntakeConstants.intakeSpeed);
                break;
            case BACKWARDS:
                this.motor.set(-Constants.IntakeConstants.intakeSpeed);
                break;
        }
    }

    public void stop() {
        this.motor.set(0.0);
    }

    public CANSparkMax getMotor() {
        return this.motor;
    }

    public void close() {
        this.motor.close();
    }
}