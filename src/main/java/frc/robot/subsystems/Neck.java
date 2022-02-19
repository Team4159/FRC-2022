package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Direction;

public class Neck extends SubsystemBase{
    private CANSparkMax motor;

    public Neck() {
        this.motor = new CANSparkMax(Constants.CanIds.neckSpark, MotorType.kBrushless);
    }

    public void set(Direction direction) {
        switch (direction) {
            case FORWARDS:
                this.motor.set(Constants.NeckConstants.neckSpeed);
                break;
            case BACKWARDS:
                this.motor.set(-Constants.NeckConstants.neckSpeed);
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