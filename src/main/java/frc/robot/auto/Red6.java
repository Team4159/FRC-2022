package frc.robot.auto;

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


public class Red6 extends SequentialCommandGroup{
    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private Feeder feeder;
    private Neck neck;
    private Shooter shooter;

    //Not Tested
    public Red6(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Neck neck, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.feeder = feeder;
        this.neck = neck;
        this.shooter = shooter;

        addCommands(
            new SequentialCommandGroup(
                new NeckAndShoot(feeder, neck, shooter).withTimeout(2),
                new WaitCommand(8),
                new TurnDegrees(drivetrain, 180),
                new MoveDistance(drivetrain, 4)                 
            )  
        );
    }

}
