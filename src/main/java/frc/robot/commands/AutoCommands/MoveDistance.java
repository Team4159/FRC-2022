package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
        // double error = distance - drivetrain.getRightPosition();

        // double outputSpeed = 0.5 * error + kI * errorSum + kD * errorRate;

        drivetrain.moveDistance(distance);
        isFinished();
    }

    public boolean isFinished() {
        
        return drivetrain.atDistanceSetpoint(distance, Constants.DriveTrainConstants.lTolerance);
    }

    public void end(boolean i) {
        drivetrain.stop();
        drivetrain.zeroSensors();
    }
}
