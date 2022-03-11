package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Direction;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.*;
import frc.robot.commands.CommandGroups.ArmIntakeAndFeeder;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;


public class RedAuto2 extends ParallelCommandGroup {
    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private Feeder feeder;
    private Neck neck;
    private Shooter shooter;



    public RedAuto2(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Neck neck, Shooter shooter) {
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

