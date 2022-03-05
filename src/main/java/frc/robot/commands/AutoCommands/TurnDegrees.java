package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;

public class TurnDegrees extends CommandBase{
    private Drivetrain drivetrain;
    private double angle; //In Degrees

    public TurnDegrees(Drivetrain drivetrain, double angle) {
        this.drivetrain = drivetrain;
        this.angle = angle;

        addRequirements(drivetrain);
    }

    public void execute() {
        drivetrain.turnDegrees(angle);
    }

    public boolean isFinished() {
        return drivetrain.atAngleSetpoint(angle, Constants.DriveTrainConstants.aTolerance);
    }

    public void end(boolean i) {
        drivetrain.stop();
        drivetrain.zeroSensors();
    }
}
