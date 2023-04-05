package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Direction;
import frc.robot.subsystems.Neck;

public class RunNeck extends CommandBase {
    private Neck neck;
    private Direction direction;

    public RunNeck(Neck neck, Direction direction) {
        this.neck = neck;
        this.direction = direction;
        addRequirements(neck);

    }

    @Override
    public void execute() {
        neck.runNeck(direction);
    }

    @Override 
    public void end(boolean i) {
        neck.stop();
    }
}
