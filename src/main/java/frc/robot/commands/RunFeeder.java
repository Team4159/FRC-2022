package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Direction;
import frc.robot.subsystems.Feeder;

public class RunFeeder extends CommandBase{
    private Feeder feeder;
    private Direction direction;

    public RunFeeder(Feeder feeder, Direction dir) {
        this.feeder = feeder;
        this.direction = dir;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        feeder.set(this.direction);
    }

    @Override
    public void end(boolean i) {
        feeder.stop();
    }
}