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

public class NeckAndShoot extends SequentialCommandGroup {
    private Neck neck;
    private Shooter shooter;
    private Feeder feeder;



    public NeckAndShoot (Feeder feeder, Neck neck, Shooter shooter) {
        this.feeder = feeder;
        this.neck = neck;
        this.shooter = shooter;


        addCommands(
            new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new RunNeck(neck, Direction.BACKWARDS),
                        new RunFeeder(feeder, Direction.BACKWARDS)
                    ).withTimeout(0.1),
                    new WaitCommand(1),
                    new ParallelCommandGroup(
                        new RunNeck(neck, Direction.FORWARDS),
                        new RunFeeder(feeder, Direction.FORWARDS)
                    ).withTimeout(0.2),
                    new RunFeeder(feeder, Direction.FORWARDS).withTimeout(0.6),
                    new ParallelCommandGroup(
                        new RunNeck(neck, Direction.FORWARDS),
                        new RunFeeder(feeder, Direction.FORWARDS)
                    ).withTimeout(0.2)
                )
        );
    }
}
