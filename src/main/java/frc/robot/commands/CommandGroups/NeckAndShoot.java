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
            /*new ParallelCommandGroup(
                new Shoot(shooter,Constants.ShooterConstants.targetVelocity).withTimeout(0.5),
                new RunNeck(neck, Direction.BACKWARDS).withTimeout(0.5),
                new RunFeeder(feeder, Direction.BACKWARDS).withTimeout(0.05)
            ),
            
            //new ParallelCommandGroup(
                //new Shoot(shooter, Constants.ShooterConstants.targetVelocity).withTimeout(0.3),
                //new RunNeck(neck, Direction.FORWARDS).withTimeout(0.3)
            //),

            new ParallelCommandGroup(
                new Shoot(shooter, Constants.ShooterConstants.targetVelocity).withTimeout(0.3),
                new RunNeck(neck, Direction.FORWARDS).withTimeout(0.3),
                new RunFeeder(feeder, Direction.FORWARDS).withTimeout(0.3)
        */  new ParallelCommandGroup(
                new Shoot(shooter, Constants.ShooterConstants.targetVelocity),
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new RunNeck(neck, Direction.BACKWARDS),
                        new RunFeeder(feeder, Direction.BACKWARDS)
                    ).withTimeout(0.1),
                
                    //new RunFeeder(feeder, Direction.FORWARDS).withTimeout(0.2),
                    new ParallelCommandGroup(
                        new RunNeck(neck, Direction.FORWARDS),
                        new RunFeeder(feeder, Direction.FORWARDS)
                    ).withTimeout(0.2),
                    new RunFeeder(feeder, Direction.FORWARDS).withTimeout(0.7),
                    new ParallelCommandGroup(
                        new RunNeck(neck, Direction.FORWARDS),
                        new RunFeeder(feeder, Direction.FORWARDS)
                    )
                )
            )
        );
    }
}
