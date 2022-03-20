package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.MoveArm;
import frc.robot.commands.Shoot;
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


public class Red1Ball extends SequentialCommandGroup{
    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;
    private Feeder feeder;
    private Neck neck;
    private Shooter shooter;

    //Not Tested
    public Red1Ball(Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Neck neck, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
        this.feeder = feeder;
        this.neck = neck;
        this.shooter = shooter;

        addCommands(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new NeckAndShoot(feeder, neck, shooter).withTimeout(3),
                    new Shoot(shooter, Constants.ShooterConstants.targetVelocity).withTimeout(3)
                ),    
                new WaitCommand(8),
                new TurnDegrees(drivetrain, 180),
                new MoveDistance(drivetrain, 2),
                new MoveArm(arm, ArmState.HIGH)                
            )  
        );
    }

}
