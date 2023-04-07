package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Direction;

public class SingleMotorSubsystem extends SubsystemBase {
    protected MotorController motor;
    private double speed;

    public SingleMotorSubsystem(int canID, double speed) {
        this.motor = new CANSparkMax(canID, MotorType.kBrushless);
        this.speed = speed;
    }

    protected SingleMotorSubsystem(MotorController motor, boolean inverted, double speed) {
        this.motor = motor;
        this.motor.setInverted(inverted);
        this.speed = speed;
    }

    public void run(Direction direction) {
        motor.set(direction.equals(Direction.FORWARDS) ? speed : -speed);
    }

    public void stop() {
        motor.stopMotor();
    }
}
