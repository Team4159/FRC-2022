package frc.robot.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Direction;
import frc.robot.commands.Shoot;
import frc.robot.commands.CommandGroups.ArmIntakeAndFeeder;
import frc.robot.subsystems.*;
import frc.robot.trajectories.Trajectories;

public class BlueAuto2 extends ParallelCommandGroup{
    Drivetrain drivetrain;
    Arm arm;
    Intake intake;
    Feeder feeder;
    Shooter shooter;
    Trajectories trajectories = new Trajectories();
    Trajectory trajectory;

    public BlueAuto2(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.feeder = feeder;
        this.shooter = shooter;
        trajectory =  trajectories.loadTrajectory("paths/BlueAuto2.wpilib.json");

        ParallelDeadlineGroup BlueAuto2 = new ParallelDeadlineGroup(
            new WaitCommand(15),
            Trajectories.followTrajectory(drivetrain, trajectory),
            new SequentialCommandGroup(
                new Shoot(shooter, Constants.ShooterConstants.targetVelocity).withTimeout(1),
                new WaitCommand(3),
                new ArmIntakeAndFeeder(arm, intake, feeder).withTimeout(2),
                new WaitCommand(2),
                new ArmIntakeAndFeeder(arm, intake, feeder).withTimeout(2),
                new WaitCommand(3),
                new Shoot(shooter, Constants.ShooterConstants.targetVelocity).withTimeout(1),
                new Shoot(shooter, Constants.ShooterConstants.targetVelocity).withTimeout(1)
            )  
        );

        addCommands(BlueAuto2);

    }

}
