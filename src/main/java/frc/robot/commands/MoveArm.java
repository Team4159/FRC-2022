package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase{
    
    private Arm arm;

    private int armState = 0; // 0 is stopped, 1 is bring to lower set point, 2 is bring to higher set point

    private int lowSetPoint = Constants.IntakeAndArmConstants.pidLowSetPoint, highSetPoint = Constants.IntakeAndArmConstants.pidHighSetPoint;

    public MoveArm(Arm arm, int armState) {
        this.arm = arm;
        this.armState = armState;
        addRequirements(arm);
        
    }


    @Override
    public void execute() {
        if (armState == 0) {
            arm.stopArm();
        } else if (armState == 1) {
            arm.setArmSpeed(arm.calculatePID(arm.getEncoderRaw(), lowSetPoint));
        } else if (armState == 2) {
            arm.setArmSpeed(arm.calculatePID(arm.getEncoderRaw(), highSetPoint));
        } else {
            arm.stopArm();
        }

    }

}

