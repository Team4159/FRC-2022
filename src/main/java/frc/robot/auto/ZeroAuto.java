package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Direction;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveWithoutControls;
import frc.robot.commands.MoveArm;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.Shoot;
import frc.robot.commands.CommandGroups.NeckAndShoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.trajectories.Trajectories;

//Shoot, and Back Up
public class ZeroAuto extends SequentialCommandGroup{
    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private Feeder feeder;
    private Neck neck;
    private Shooter shooter;

    public ZeroAuto(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Neck neck, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.feeder = feeder;
        this.neck = neck;
        this.shooter = shooter;

        addCommands(
            new NeckAndShoot(feeder, neck, shooter).withTimeout(1),
            new DriveWithoutControls(drivetrain, 0.5, 0.5).withTimeout(0.5),
            new DriveWithoutControls(drivetrain, -0.1, -0.1).withTimeout(0.1),
            new MoveArm(arm, ArmState.HIGH).withTimeout(0.1)
        );
    }

}
