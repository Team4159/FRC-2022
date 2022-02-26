package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.Direction;

public class Shoot extends CommandBase {
    private Neck neck;
    private Shooter shooter;
    private Double targetVelocity;


    public Shoot(Shooter shooter, double targetVelocity) {
        this.targetVelocity = targetVelocity;
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.shoot(targetVelocity);
    }

    @Override 
    public void end(boolean i) {
        shooter.stop();
    }
}
