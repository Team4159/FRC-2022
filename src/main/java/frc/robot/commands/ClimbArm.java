package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberState;

public class ClimbArm extends CommandBase {
    private Climber climber;
    private ClimberState climberState;

    public ClimbArm(Climber climber, ClimberState climberState) {
        this.climber = climber;
        this.climberState = climberState;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.runClimberElevator(climberState);
    }
}
