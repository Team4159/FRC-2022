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
            new ParallelCommandGroup(
                new MoveDistance(drivetrain, 3),
                new ArmIntakeAndFeeder(arm, intake, feeder,neck).withTimeout(3)
            ).withTimeout(3),
            new TurnDegrees(drivetrain, 180),
            new MoveArm(arm, ArmState.HIGH).withTimeout(0.3),
            new MoveDistance(drivetrain, 5.8).withTimeout(3),
            new ParallelCommandGroup(
                new NeckAndShoot(feeder, neck, shooter),
                new Shoot(shooter, Constants.ShooterConstants.targetVelocity)
            ).withTimeout(5)
        );
    }

}
