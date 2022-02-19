package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Direction;
import frc.robot.subsystems.Neck;

public class RunNeck extends CommandBase {
    Neck neck;
    private Direction direction;

    public RunNeck(Neck neck, Direction dir) {
        this.neck = neck;
        this.direction = dir;

        addRequirements(neck);
    }

    @Override
    public void execute() {
        neck.set(this.direction);
    }

    @Override
    public void end(boolean i) {
        neck.stop();
    }
}
