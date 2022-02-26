package frc.robot.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Direction;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;
import frc.robot.trajectories.Trajectories;


public class FirstAuto extends ParallelCommandGroup {
    Drivetrain drivetrain;
    Arm arm;
    Intake intake;
    Feeder feeder;
    Shooter shooter;
    Neck neck;
    Trajectories trajectories = new Trajectories();
    Trajectory trajectory;



    public FirstAuto(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Shooter shooter, Neck neck) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.feeder = feeder;
        this.shooter = shooter;
        this.neck = neck;
        trajectory =  trajectories.loadTrajectory("path1");
    }

} 