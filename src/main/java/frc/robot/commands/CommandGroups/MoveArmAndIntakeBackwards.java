package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Feeder;
import frc.robot.Constants.Direction;
import frc.robot.commands.MoveArm;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunNeck;
import frc.robot.commands.RunFeeder;

public class MoveArmAndIntakeBackwards extends ParallelCommandGroup {
    private Arm arm;
    private Intake intake;
    private Feeder feeder;
    private Neck neck;

    public MoveArmAndIntakeBackwards (Arm arm, Intake intake, Feeder feeder, Neck neck) {
        this.arm = arm;
        this.intake = intake;
        this.feeder = feeder;
        this.neck = neck;

        addCommands(
            new MoveArm(arm, ArmState.LOW),
            new RunIntake(intake, Direction.BACKWARDS),
            new RunFeeder(feeder, Direction.BACKWARDS),
            new RunNeck(neck, Direction.BACKWARDS)
        );
    }
}