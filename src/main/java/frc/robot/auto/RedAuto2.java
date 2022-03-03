package frc.robot.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.commands.CommandGroups.ArmIntakeAndFeeder;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.trajectories.Trajectories;


public class RedAuto2 extends ParallelCommandGroup{
     private Drivetrain drivetrain;
     private Arm arm;
     private Intake intake;
     private Feeder feeder;
     private Shooter shooter;
     private Trajectories trajectories = new Trajectories();
     private Trajectory trajectory;



     public RedAuto2(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.feeder = feeder;
        this.shooter = shooter;
        trajectory =  trajectories.loadTrajectory("../trajectories/output/RedAuto2.wpilib.json");

        ParallelDeadlineGroup BlueAuto2 = new ParallelDeadlineGroup(
            new WaitCommand(15d),
            Trajectories.followTrajectory(drivetrain, trajectory),
            new SequentialCommandGroup(
                new Shoot(shooter).withTimeout(1),
                new WaitCommand(3),
                new ArmIntakeAndFeeder(arm, intake, feeder).withTimeout(2),
                new WaitCommand(2),
                new ArmIntakeAndFeeder(arm, intake, feeder).withTimeout(2),
                new WaitCommand(3),
                new Shoot(shooter).withTimeout(2)
            )  
        );

        addCommands(BlueAuto2);
    }
}


