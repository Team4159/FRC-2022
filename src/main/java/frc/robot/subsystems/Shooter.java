package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private WPI_TalonFX shooterTalonLeft;
    private WPI_TalonFX shooterTalonRight;

    public Shooter() {
        shooterTalonLeft = new WPI_TalonFX(Constants.CanIds.shooterTalonLeft);
        shooterTalonRight = new WPI_TalonFX(Constants.CanIds.shooterTalonRight);

        shooterTalonLeft.follow(shooterTalonRight);

        shooterTalonRight.config_kP(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kP);
        shooterTalonRight.config_kI(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kI);
        shooterTalonRight.config_kD(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kD);
        shooterTalonRight.config_kF(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kF);

    }

    public void shoot(double targetVelocity) {
        //Target Velocity In Revolution Per Seconds
        shooterTalonRight.set(ControlMode.Velocity, targetVelocity); 
    }

    public void stop() {
        shooterTalonRight.set(0);
    }

    public double getVelocity() {
        return shooterTalonRight.getSelectedSensorVelocity();
    }

}
