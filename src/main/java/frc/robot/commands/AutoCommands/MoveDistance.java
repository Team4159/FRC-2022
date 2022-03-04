package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;

public class MoveDistance extends CommandBase{
    private Drivetrain drivetrain;
    private double distance;

    public MoveDistance(Drivetrain drivetrain, double distance) {
        this.drivetrain = drivetrain;
        this.distance = distance;

        addRequirements(drivetrain);
    }

    public void execute() {
        drivetrain.moveDistance(distance);
    }

    public void end(boolean i) {
        drivetrain.stop();
    }
}
