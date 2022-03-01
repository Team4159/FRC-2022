package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveWithoutControls extends CommandBase {
    private Drivetrain drivetrain;
    private double leftSpeed;
    private double rightSpeed;

    public DriveWithoutControls(Drivetrain drivetrain, double leftSpeed, double rightSpeed) {
        this.drivetrain = drivetrain;
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(leftSpeed, rightSpeed);
    }

    @Override
    public void end(boolean i) {
        drivetrain.stop();
    }
}
