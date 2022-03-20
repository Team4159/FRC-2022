package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.commands.CommandGroups.*;
import frc.robot.commands.AutoCommands.*;

public class Blue3Ball extends SequentialCommandGroup{
    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private Feeder feeder;
    private Neck neck;
    private Shooter shooter;

    public Blue3Ball(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Neck neck, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.feeder = feeder;
        this.neck = neck;
        this.shooter = shooter;

        addCommands(
            new MoveArm(arm, ArmState.HIGH),
            new ParallelCommandGroup(
                    new NeckAndShoot(feeder, neck, shooter),
                    new Shoot(shooter, Constants.ShooterConstants.targetVelocity)
            ).withTimeout(3),
            new MoveDistance(drivetrain, -1),
            new TurnDegrees(drivetrain, 150),
            new ParallelDeadlineGroup(
                new ArmIntakeAndFeeder(arm, intake, feeder,neck),
                new SequentialCommandGroup(
                    new MoveDistance(drivetrain, 2),
                    new TurnDegrees(drivetrain, 120),
                    new MoveDistance(drivetrain, 2)
                )
            ).withTimeout(5),
            new MoveArm(arm, ArmState.HIGH),
            new TurnDegrees(drivetrain, 120),
            new MoveDistance(drivetrain, 3),
            new TurnDegrees(drivetrain, -60),
            new ParallelCommandGroup(
                new NeckAndShoot(feeder, neck, shooter),
                new Shoot(shooter, Constants.ShooterConstants.targetVelocity)
            ).withTimeout(5),
            new TurnDegrees(drivetrain, 180),
            new MoveDistance(drivetrain, 1)
        );
    }

}
