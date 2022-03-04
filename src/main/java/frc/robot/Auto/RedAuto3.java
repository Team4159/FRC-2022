package frc.robot.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Direction;
import frc.robot.commands.MoveArm;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunNeck;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.trajectories.Trajectories;

public class RedAuto3 extends ParallelCommandGroup{
    Drivetrain drivetrain;
    Arm arm;
    Intake intake;
    Feeder feeder;
    Shooter shooter;
    Neck neck;

    public RedAuto3(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Shooter shooter, Neck neck) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.feeder = feeder;
        this.shooter = shooter;
        this.neck = neck;

    }

}

    