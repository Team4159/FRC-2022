package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Direction;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
    private Intake intake;
    private Direction direction;

    public RunIntake(Intake intake, Direction dir) {
        this.intake = intake;
        this.direction = dir;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.set(this.direction);
    }

    @Override
    public void end(boolean i) {
        intake.stop();
    }
}