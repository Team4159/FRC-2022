package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private WPI_TalonFX[] shooterTalons = new WPI_TalonFX[2];
    private CANSparkMax neckSpark;

    public Shooter() {
        this.shooterTalons[0] = new WPI_TalonFX(Constants.CanIds.shooterTalonLeft); // left
        this.shooterTalons[1] = new WPI_TalonFX(Constants.CanIds.shooterTalonRight);// right
        this.neckSpark = new CANSparkMax(Constants.CanIds.neckSpark, MotorType.kBrushless);

        this.shooterTalons[0].setInverted(true);
    }

    public void setShooter() {
        this.shooterTalons[0].set(Constants.ShooterConstants.shootSpeed);
        this.shooterTalons[1].set(Constants.ShooterConstants.shootSpeed);
    }

    public void setNeck() {
        this.neckSpark.set(Constants.ShooterConstants.neckSpeed);
    }

    public void stop() {
        this.shooterTalons[0].set(0.0);
        this.shooterTalons[1].set(0.0);
        this.neckSpark.set(0);
    }

    public CANSparkMax getNeckMotor() {
        return this.neckSpark;
    }

    public void close() {
        this.shooterTalons[0].close();
        this.shooterTalons[1].close();
        this.neckSpark.close();
    }
}
