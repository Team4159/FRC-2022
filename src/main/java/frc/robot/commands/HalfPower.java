package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class HalfPower extends CommandBase {
    private final Drivetrain drivetrain;

    public HalfPower(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.halfPower();
    }

    @Override
    public void end(boolean i) {
        drivetrain.fullPower();
    }
}
