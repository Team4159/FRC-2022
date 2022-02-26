package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
import frc.robot.subsystems.Shooter;
import frc.robot.trajectories.Trajectories;

//Does Nothing
public class ZeroAuto extends CommandBase{
//     Drivetrain drivetrain;
//     Arm arm;
//     Intake intake;
//     Feeder feeder;
//     Shooter shooter;
//     Trajectories trajectories = new Trajectories();
//     Trajectory trajectory;



//     public FirstAuto(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Shooter shooter) {
//         this.drivetrain = drivetrain;
//         this.arm = arm;
//         this.intake = intake;
//         this.feeder = feeder;
//         this.shooter = shooter;
//         trajectory =  trajectories.loadTrajectory("Path");

//         new ParallelCommandGroup(
//           trajectories.followTrajectory(drivetrain, trajectory),
//           new SequentialCommandGroup(
//               new Shoot(shooter),
//               new WaitCommand(5),
//               new ArmAndIntake(arm, intake).withTimeout(5),
//               new RunFeeder(feeder, Direction.FORWARDS),
//               new WaitCommand(5)
//           )  
//         ).withTimeout(15);


//     }
// }
}
