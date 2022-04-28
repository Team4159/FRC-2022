package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunNeck;
import frc.robot.commands.Shoot;
import frc.robot.Constants;
import frc.robot.Constants.Direction;

public class NeckAndFeeder extends ParallelCommandGroup {
    private Neck neck;
    private Feeder feeder;



    public NeckAndFeeder (Feeder feeder, Neck neck) {
        this.feeder = feeder;
        this.neck = neck;


        addCommands(
            new RunNeck(neck, Direction.BACKWARDS),
            new RunFeeder(feeder, Direction.FORWARDS)
        );
    }
}