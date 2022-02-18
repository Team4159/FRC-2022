package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Direction;
import frc.robot.subsystems.Climber;

public class RunClimber extends CommandBase {
    private Climber climber;
    private Direction direction;

    public RunClimber(Climber climber, Direction dir) {
        this.climber = climber;
        this.direction = dir;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.set(this.direction);
    }

    @Override
    public void end(boolean i) {
        climber.stop();
    }
}