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
    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private Feeder feeder;
    private Neck neck;
    private Shooter shooter;



    public BlueAuto2(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Neck neck, Shooter shooter) {
       this.drivetrain = drivetrain;
       this.arm = arm;
       this.intake = intake;
       this.feeder = feeder;
       this.neck = neck;
       this.shooter = shooter;

       addCommands(

       );
   }

}