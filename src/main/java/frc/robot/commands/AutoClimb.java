package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoClimb extends CommandBase{
    private Drivetrain drivetrain;
    private boolean climbState = false;

    public AutoClimb(Drivetrain drivetrain, boolean state) {
        this.drivetrain = drivetrain;
        this.climbState = state;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        if (climbState) {
            drivetrain.autoClimb();
        } else {
            drivetrain.stopClimb();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
