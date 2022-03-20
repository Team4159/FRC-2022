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

public class Blue2Ball extends SequentialCommandGroup{
    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private Feeder feeder;
    private Neck neck;
    private Shooter shooter;

    public Blue2Ball(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Neck neck, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.feeder = feeder;
        this.neck = neck;
        this.shooter = shooter;

        addCommands(
            new ParallelDeadlineGroup(
                new MoveDistance(drivetrain, 2),
                new ArmIntakeAndFeeder(arm, intake, feeder,neck)
            ).withTimeout(3),
            new MoveArm(arm, ArmState.HIGH),
            new TurnDegrees(drivetrain, 180),
            new MoveDistance(drivetrain, 2.8),
            new ParallelCommandGroup(
                new NeckAndShoot(feeder, neck, shooter).withTimeout(5),
                new Shoot(shooter, Constants.ShooterConstants.targetVelocity).withTimeout(5)
            ),
            new TurnDegrees(drivetrain, 180),
            new MoveDistance(drivetrain, 1)
        );
    }

}
