package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
    private Shooter shooter;



    public Shoot(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        //System.out.println(shooter.getVelocity());
        shooter.shoot(5000);//In RPM
    }

    @Override 
    public void end(boolean i) {
        shooter.stop();
    }
}
