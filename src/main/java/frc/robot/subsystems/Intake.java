package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Direction;

public class Intake extends SubsystemBase {
    private CANSparkMax motor;
    private Direction direction;

    public Intake() {
        this.motor = new CANSparkMax(Constants.CanIds.intakeSpark, MotorType.kBrushless);
    }

    public void set(Direction direction) {
        this.direction = direction;
        switch (direction) {
            case FORWARDS:
                motor.set(Constants.IntakeAndArmConstants.intakeSpeed);
                break;
            case BACKWARDS:
                motor.set(-Constants.IntakeAndArmConstants.intakeSpeed);
                break;
        }
    }

    public void stop() {
        this.motor.set(0);
    }

    public CANSparkMax getMotor() {
        return this.motor;
    }

    public void close() {
        this.motor.close();
    }
}
