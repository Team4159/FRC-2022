package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private CANSparkMax intakeSpark;
    private Direction direction;

    public static enum Direction {
        FORWARD,
        BACKWARD
    }

    public Intake() {
        intakeSpark = new CANSparkMax(Constants.CanIds.intakeSpark, MotorType.kBrushless);
    }

    public void runIntake(Direction direction) {
        this.direction = direction;
        switch(direction) {
            case FORWARD:
                intakeSpark.set(Constants.IntakeAndArmConstants.intakeSpeed);
                break;
            case BACKWARD:
                intakeSpark.set(Constants.IntakeAndArmConstants.backwardsIntakeSpeed);
                break;
        }
    }


    public void stopIntake() {
        intakeSpark.set(0);
    }

    public CANSparkMax getIntakeSpark() { 
        return this.intakeSpark;
    }

    public void close() {
        this.intakeSpark.close();
    }

}
