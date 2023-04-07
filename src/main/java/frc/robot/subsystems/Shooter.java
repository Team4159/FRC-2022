package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private WPI_TalonFX shooterTalonRight;

    public Shooter() {
        var shooterTalonLeft = new WPI_TalonFX(Constants.CanIds.shooterTalonLeft);
        shooterTalonRight = new WPI_TalonFX(Constants.CanIds.shooterTalonRight);

        shooterTalonLeft.follow(shooterTalonRight);

        shooterTalonRight.config_kP(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kP);
        shooterTalonRight.config_kI(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kI);
        shooterTalonRight.config_kD(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kD);
        shooterTalonRight.config_kF(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kF);
        shooterTalonLeft.close();
    }

    public void shoot(double targetVelocity) {
        double targetVelocityInTicks =  targetVelocity * 2048.0 / 600.0;
        shooterTalonRight.set(ControlMode.Velocity, targetVelocityInTicks);
    }
    public void shootIn() {
        shooterTalonRight.set(ControlMode.Velocity, -1000);
    }

    public boolean atSetpoint(Double setpoint, Double tolerance) {
        return getVelocity() <= setpoint + tolerance && getVelocity() >= setpoint - tolerance;
    }

    public void stop() {
        shooterTalonRight.set(0);
    }

    public double getVelocity() {
        return shooterTalonRight.getSelectedSensorVelocity();
    }
}
