package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Direction;
import frc.robot.subsystems.Feeder;

public class RunFeeder extends CommandBase{
    private Feeder feeder;
    private Direction direction;

    public RunFeeder(Feeder feeder, Direction direction) {
        this.feeder = feeder;
        this.direction = direction;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        feeder.run(direction);
    }

    @Override
    public void end(boolean i) {
        feeder.stop();
    }
}
