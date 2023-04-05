package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private WPI_TalonFX shooterTalonLeft;
    private WPI_TalonFX shooterTalonRight;
    private MotorControllerGroup shooterTalons;

    public Shooter() {
        shooterTalonLeft = new WPI_TalonFX(Constants.CanIds.shooterTalonLeft);
        shooterTalonRight = new WPI_TalonFX(Constants.CanIds.shooterTalonRight);
        shooterTalons = new MotorControllerGroup(shooterTalonLeft, shooterTalonRight);

        shooterTalonLeft.follow(shooterTalonRight);

        shooterTalonRight.config_kP(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kP);
        shooterTalonRight.config_kI(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kI);
        shooterTalonRight.config_kD(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kD);
        shooterTalonRight.config_kF(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kF);

    }

    public void shoot(double targetVelocity) {
        double targetVelocityInTicks =  targetVelocity * 2048.0 / 600.0;
        //System.out.println(getVelocity());
        //System.out.println(shooterTalonLeft.getMotorOutputPercent());
        shooterTalonRight.set(ControlMode.Velocity, targetVelocityInTicks); 
        //shooterTalonLeft.set(0.8);
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

    public double getMotorOuput() {
        return shooterTalonRight.getMotorOutputPercent();
    }

    public WPI_TalonFX getLeftShooterTalon() {
        return shooterTalonLeft;
    }

    public WPI_TalonFX getRightShooterTalon() {
        return shooterTalonRight;
    }

}
