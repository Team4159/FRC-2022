package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.Constants.Direction;
import frc.robot.commands.RunArm;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunFeeder;

public class ArmIntakeAndFeeder extends ParallelCommandGroup {
    private Arm arm;
    private Intake intake;
    private Feeder feeder;

    public ArmIntakeAndFeeder (Arm arm, Intake intake, Feeder feeder) {
        this.arm = arm;
        this.intake = intake;
        this.feeder = feeder;

        addCommands(
            new RunArm(arm, ArmState.LOW),
            new RunIntake(intake, Direction.FORWARDS),
            new RunFeeder(feeder, Direction.FORWARDS)
        );
    }
}