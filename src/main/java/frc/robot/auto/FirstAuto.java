package frc.robot.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;
import frc.robot.trajectories.Trajectories;


public class FirstAuto extends ParallelCommandGroup {


    // public FirstAuto(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Shooter shooter, Neck neck) {
    //     this.drivetrain = drivetrain;
    //     this.arm = arm;
    //     this.intake = intake;
    //     this.feeder = feeder;
    //     this.shooter = shooter;
    //     this.neck = neck;
    //     trajectory =  trajectories.loadTrajectory("path1");

    //     new ParallelCommandGroup(
    //       trajectories.followTrajectory(drivetrain, trajectory),
    //       new SequentialCommandGroup( 
    //           new ParallelCommandGroup( //initial shoot
    //             new WaitCommand(1),
    //             new shoot(shooter) //assuming ball in the neck
    //           ).withTimeout(1);

    //           new SequentialCommandGroup( //take ball 2
    //             new moveArm(arm, ArmState.LOW),
    //             new runFeeder(feeder, Direction.FORWARDS),
    //             new moveArm(arm, ArmState.HIGH),
    //             new runNeck(neck, Direction.FORWARDS)
    //           ).withTimeout(1);

    //           new SequentialCommandGroup( //take ball 3
    //             new moveArm(arm, ArmState.LOW),
    //             new runFeeder(feeder, Direction.FORWARDS),
    //             new moveArm(arm, ArmState.HIGH)
    //             //new runNeck(neck, Direction.FORWARDS)
    //           ).withTimeout(1);

    //           new SequentialCommandGroup( //shoot balls 2 and 3
    //             new shoot(shooter),
    //             new runNeck(neck, Direction.FORWARDS),
    //             new shoot(shooter)
    //           ).withTimeout(1);

    //           // >>>> ambitious mode == on >:) <<<<
    //           new SequentialCommandGroup( //take ball 1
    //             new moveArm(arm, ArmState.LOW),
    //             new runFeeder(feeder, Direction.FORWARDS),
    //             new moveArm(arm, ArmState.HIGH),
    //             new runNeck(neck, Direction.FORWARDS)
    //           ).withTimeout(1);

    //           new SequentialCommandGroup( //take ball 4
    //             new moveArm(arm, ArmState.LOW),
    //             new runFeeder(feeder, Direction.FORWARDS),
    //             new moveArm(arm, ArmState.HIGH)
    //           ).withTimeout(1);

    //           new SequentialCommandGroup( //shoot balls 1 and 4
    //             new shoot(shooter),
    //             new runNeck(neck, Direction.FORWARDS),
    //             new shoot(shooter)
    //           ).withTimeout(1);
    //       ).withTimeout(15);
    // }
}
