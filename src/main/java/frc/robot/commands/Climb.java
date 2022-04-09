package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
    public void initialize() {
        climber.runClimberElevator(climberState);
    }

    public boolean isFinished() {
        return climber.atSetpoint(Constants.ClimberConstants.elevatorHighSetPoint, 1000);
    }
}
