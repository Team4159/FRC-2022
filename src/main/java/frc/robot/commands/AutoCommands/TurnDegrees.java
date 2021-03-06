package frc.robot.commands.AutoCommands;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;

public class TurnDegrees extends CommandBase{
    private Drivetrain drivetrain;
    private double angle; //In Degrees

    public TurnDegrees(Drivetrain drivetrain, double angle) {
        this.drivetrain = drivetrain;
        this.angle = angle;

        addRequirements(drivetrain);
    }

    public void execute() {
        drivetrain.turnDegrees(angle);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.atAngleSetpoint(angle, 5);
    }

    public void end(boolean i) {
        drivetrain.stop();
        drivetrain.zeroSensors();
    }
}