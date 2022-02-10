package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private WPI_TalonFX shooterTalonLeft;
    private WPI_TalonFX shooterTalonRight;

    private CANSparkMax shooterSparkNeck;

    public Shooter() {
        shooterTalonLeft = new WPI_TalonFX(Constants.CanIds.shooterTalonLeft);
        shooterTalonRight = new WPI_TalonFX(Constants.CanIds.shooterTalonRight);

        shooterSparkNeck = new CANSparkMax(Constants.CanIds.shooterSparkNeck, MotorType.kBrushless);

        shooterTalonLeft.setInverted(true);
    }

    public void Shoot() {
        shooterTalonLeft.set(Constants.ShooterConstants.shootSpeed);
        shooterTalonRight.set(Constants.ShooterConstants.shootSpeed);
    }

    public void RunNeck() {
        shooterSparkNeck.set(Constants.ShooterConstants.neckSpeed);
    }
}
