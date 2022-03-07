package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
    private Shooter shooter;



    public Shoot(Shooter shooter, double targetvelocity) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.getRightShooterTalon().set(1);
    }

    @Override 
    public void end(boolean i) {
        shooter.stop();
    }
}
