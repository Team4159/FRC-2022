package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class FlipDrivetrain extends CommandBase {
    private final Drivetrain drivetrain;

    public FlipDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.flipDrivetrain();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
