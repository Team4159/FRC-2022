package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Direction;

public class Feeder extends SubsystemBase {
    private CANSparkMax motor;
    private Direction direction;

    public Feeder() {
        this.motor = new CANSparkMax(Constants.CanIds.feederSpark, MotorType.kBrushless);
    }

    public void set(Direction direction) {
        this.direction = direction;
        switch (direction) {
            case FORWARDS:
                this.motor.set(Constants.FeederConstants.feederSpeed);
                break;
            case BACKWARDS:
                this.motor.set(-Constants.FeederConstants.feederSpeed);
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