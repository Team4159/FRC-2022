package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class HalfPower extends CommandBase {
    private final Drivetrain drivetrain;

    private boolean halfPower;

    public HalfPower(Drivetrain drivetrain, boolean halfPower) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        this.halfPower = halfPower;
    }

    @Override
    public void execute() {
        if (halfPower) {
            drivetrain.halfPower();
        } else {
            drivetrain.fullPower();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
