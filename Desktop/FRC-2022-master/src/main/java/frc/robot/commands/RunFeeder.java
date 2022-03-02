package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class RunFeeder extends CommandBase{
    private Feeder feeder;
    private boolean direction;

    public RunFeeder(Feeder feeder, boolean direction) {
        this.feeder = feeder;
        this.direction = direction;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        feeder.set(true);
    }

    @Override
    public void end(boolean i) {
        feeder.stop();
    }
}
