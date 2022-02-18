package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunShooter;
import frc.robot.Constants.Direction;

public class FeederAndShoot extends ParallelCommandGroup {
    private Feeder feeder;
    private Shooter shooter;

    public FeederAndShoot (Feeder feeder, Shooter shooter) {
        this.feeder = feeder;
        this.shooter = shooter;

        addCommands(
            new RunFeeder(feeder, Direction.FORWARDS),
            new RunShooter(shooter)
        );
    }
}
