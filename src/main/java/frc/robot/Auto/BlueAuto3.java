package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.commands.CommandGroups.*;
import frc.robot.commands.AutoCommands.*;

public class BlueAuto3 extends SequentialCommandGroup{
    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private Feeder feeder;
    private Neck neck;
    private Shooter shooter;

    public BlueAuto3(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Neck neck, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.feeder = feeder;
        this.neck = neck;
        this.shooter = shooter;

        addCommands(
            //new ArmIntakeAndFeeder(arm, intake, feeder).withTimeout(1),
            new ParallelDeadlineGroup(
                new MoveDistance(drivetrain, 1.4),
                new ArmIntakeAndFeeder(arm, intake, feeder)
            ),
            new TurnDegrees(drivetrain, 180),

            new MoveDistance(drivetrain, 2.3),
            new NeckAndShoot(feeder, neck, shooter).withTimeout(1),
            //First Part
            new TurnDegrees(drivetrain, 225),
            new ParallelDeadlineGroup(
                new MoveDistance(drivetrain, 1.4),
                new ArmIntakeAndFeeder(arm, intake, feeder)
            )
        );
    }

}
