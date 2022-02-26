package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunNeck;
import frc.robot.commands.Shoot;
import frc.robot.Constants;
import frc.robot.Constants.Direction;

public class NeckAndShoot extends ParallelCommandGroup {
    private Neck neck;
    private Shooter shooter;


    public NeckAndShoot (Neck neck, Shooter shooter) {
        this.shooter = shooter;
        this.neck = neck;

        addCommands(
            new Shoot(shooter, Constants.ShooterConstants.targetVelocity),
            new SequentialCommandGroup(
                new WaitUntilCommand(shooter.atSetpoint()),
                new RunNeck(neck, Direction.FORWARDS)
            )
        );
    }
}
