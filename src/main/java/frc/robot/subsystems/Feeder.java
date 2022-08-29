package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Direction;

public class Feeder extends SubsystemBase {
    private WPI_TalonSRX feederTalon;

    public Feeder() {
        feederTalon = new WPI_TalonSRX(Constants.CanIds.feederTalon);

        feederTalon.setInverted(true);
    }

    public void runFeeder(Direction direction) {
        switch(direction) {
            case FORWARDS:
                feederTalon.set(ControlMode.PercentOutput, Constants.FeederConstants.feederSpeed);
                break;
            case BACKWARDS:
                feederTalon.set(ControlMode.PercentOutput, Constants.FeederConstants.backwardsFeederSpeed);
                break;
        }
    }
    public void stop() {
        feederTalon.set(0.0);
    }

    public WPI_TalonSRX getFeederSpark() {
        return this.feederTalon;
    }

    public void close(){
        feederTalon.close();    
}
}
