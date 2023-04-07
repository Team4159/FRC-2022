package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootIn extends CommandBase {
    private Shooter shooter;

    public ShootIn(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.shootIn();//In RPM
    }

    @Override 
    public void end(boolean i) {
        shooter.stop();
    }
}
