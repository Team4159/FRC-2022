package frc.robot.subsystems;

import java.util.concurrent.CancellationException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Direction;

public class Neck extends SubsystemBase{
    private CANSparkMax neckSpark;

    public Neck() {
        neckSpark = new CANSparkMax(Constants.CanIds.neckSpark, MotorType.kBrushless);
    }

    public void runNeck(Direction direction) {
        switch(direction) {
            case FORWARDS:
                neckSpark.set(Constants.NeckConstants.neckSpeed);
                break;
            case BACKWARDS:
                neckSpark.set(Constants.NeckConstants.backwardsNeckSpeed);
                break;
        }
    }

    public void stop() {
        neckSpark.set(0);
    }

    public CANSparkMax getNeckSpark() {
        return neckSpark;
    }

    // public void periodic() {
    //     neckSpark.set(0.2);
    // }
    
}
