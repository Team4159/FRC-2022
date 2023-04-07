package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// intake "arm"
public class Arm extends SubsystemBase{
    private CANSparkMax armSpark;
    private RelativeEncoder encoder;
    private PIDController pid;
    
    public static enum ArmState {
        LOW,
        HIGH
    }
    
    private ArmState armState;

    public Arm() {
        armSpark = new CANSparkMax(Constants.CanIds.armSpark, MotorType.kBrushless);
        armSpark.setInverted(false);

        encoder = armSpark.getEncoder();

        pid = new PIDController(
            Constants.IntakeAndArmConstants.kP, 
            Constants.IntakeAndArmConstants.kI, 
            Constants.IntakeAndArmConstants.kD
        );

        armState = ArmState.HIGH;
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }

    public double calculatePID(double setPoint) {
        if (atSetpoint(setPoint, Constants.IntakeAndArmConstants.tolerance)) return 0;
        return pid.calculate(getEncoderPosition(), setPoint);
    }
    @Override
    public void periodic() {
        armSpark.set(MathUtil.clamp(calculatePID(armState.equals(ArmState.HIGH) ? Constants.IntakeAndArmConstants.pidHighSetPoint : Constants.IntakeAndArmConstants.pidLowSetPoint), -0.6, 0.6));
    }

    public void runArm(ArmState armState) {
        this.armState = armState;
    }

    public void resetPID() {
        pid.reset();
    }

    public void zeroArm() {
        resetPID();
        armState = ArmState.HIGH;
        runArm(armState);
    }

    public boolean atSetpoint(Double setpoint, Double tolerance) {
        return armSpark.getEncoder().getPosition() <= setpoint + tolerance && armSpark.getEncoder().getPosition() >= setpoint - tolerance;
    }

    public void close() {
        armSpark.close();
        pid.close();
    }
}