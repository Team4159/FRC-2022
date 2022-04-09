package frc.robot.commands;


import java.util.concurrent.locks.Condition;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
        shooter.shoot(Constants.ShooterConstants.targetVelocity);//In RPM
    }


    @Override 
    public void end(boolean i) {
        shooter.stop();
    }
}
