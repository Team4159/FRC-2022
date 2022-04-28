/*package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.Orientation;

public class ClimbAuto extends CommandBase{
    private Drivetrain drivetrain;
    private Joystick leftJoystick;
    private Joystick rightJoystick;

    public ClimbAuto(Drivetrain drivetrain, Joystick leftJoystick, Joystick rightJoystick) {
        this.drivetrain = drivetrain;
        this.leftJoystick = leftJoystick;



        this.rightJoystick = rightJoystick;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if(leftJoystick.getY() == 0 && rightJoystick.getY() == 0) {
            drivetrain.orientation = Orientation.FORWARD;
            drivetrain.drive(0.125, 0.125);
        }
    }

    public void end() {
        drivetrain.stop();
    }
}
*/