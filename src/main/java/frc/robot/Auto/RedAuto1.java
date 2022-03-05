package frc.robot.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.MoveArm;
import frc.robot.commands.AutoCommands.MoveDistance;
import frc.robot.commands.AutoCommands.TurnDegrees;
import frc.robot.commands.CommandGroups.ArmIntakeAndFeeder;
import frc.robot.commands.CommandGroups.NeckAndShoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.trajectories.Trajectories;


public class RedAuto1 extends ParallelCommandGroup {
    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private Feeder feeder;
    private Neck neck;
    private Shooter shooter;


    public RedAuto1(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Shooter shooter, Neck neck) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.feeder = feeder;
        this.neck = neck;
        this.shooter = shooter;

        ParallelCommandGroup redAuto1 = new ParallelCommandGroup(
            // new MoveArm(arm, ArmState.HIGH),
            // new SequentialCommandGroup(
            //     new NeckAndShoot(feeder, neck, shooter).withTimeout(1),
            //     new MoveDistance(drivetrain, -1),
            //     new TurnDegrees(drivetrain, 180),
            //     new MoveDistance(drivetrain, 0.5),
            //     new ArmIntakeAndFeeder(arm, intake, feeder).withTimeout(1.5),
            //     new TurnDegrees(drivetrain, 90),
            //     new MoveDistance(drivetrain, 1.5),
            //     new ArmIntakeAndFeeder(arm, intake, feeder).withTimeout(1.5),
            //     new TurnDegrees(drivetrain, 90),
            //     new MoveDistance(drivetrain, 1.5),
            //     new NeckAndShoot(feeder, neck, shooter).withTimeout(2)
            // )
        );

        addCommands(redAuto1);

    }

}
