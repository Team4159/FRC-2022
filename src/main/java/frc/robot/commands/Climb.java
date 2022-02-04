package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberState;



public class Climb extends CommandBase {
    private Climber climber;
    private ClimberState climberState;

    public Climb(Climber climber, ClimberState climberState) {
        this.climber = climber;
        this.climberState = climberState;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.runClimber(climberState);
    }
}
