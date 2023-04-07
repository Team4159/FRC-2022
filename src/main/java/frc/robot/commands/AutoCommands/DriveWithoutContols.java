package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveWithoutContols extends CommandBase{
    private Drivetrain drivetrain;
    private Double speed;

    public DriveWithoutContols(Drivetrain drivetrain, double speed) {
        this.drivetrain = drivetrain;
        this.speed = speed;
        addRequirements(drivetrain);
    }

    public void execute() {
        drivetrain.drive(speed, speed);
    }
}
