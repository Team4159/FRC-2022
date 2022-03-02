package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
    private CANSparkMax feederSpark;

    public Feeder() {
        feederSpark = new CANSparkMax(Constants.CanIds.feederSpark1, MotorType.kBrushless);
    }

    public void set(boolean dir) {
        feederSpark.set((dir ? 1.0 : 0.0 )*Constants.FeederConstants.feederSpeed);
    }
    public void stop() {
        feederSpark.set(0.0);
    }

    public CANSparkMax getFeederSpark() {
        return this.feederSpark;
    }

    public void close(){
        feederSpark.close();    
    }
}
