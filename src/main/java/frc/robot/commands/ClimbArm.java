package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.Joystick;

public class ClimbArm extends CommandBase {
    private Climber climber;
    private Joystick secondaryJoystick;

    public ClimbArm(Climber climber, Joystick secondaryJoystick) {
        this.climber = climber;
        this.secondaryJoystick = secondaryJoystick;
        addRequirements(climber); 
    }

    @Override
    public void execute() {
        climber.setArmSpeed(secondaryJoystick.getY()/10);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
