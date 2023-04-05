package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Feeder;
import frc.robot.Constants.Direction;
import frc.robot.commands.RunNeck;
import frc.robot.commands.RunFeeder;

public class MoveNeckAndFeederBackwards extends ParallelCommandGroup {
    private Feeder feeder;
    private Neck neck;

    public MoveNeckAndFeederBackwards(Feeder feeder, Neck neck) {
        this.feeder = feeder;
        this.neck = neck;

        addCommands(
            new RunFeeder(feeder, Direction.BACKWARDS),
            new RunNeck(neck, Direction.BACKWARDS)
        );
    }
}