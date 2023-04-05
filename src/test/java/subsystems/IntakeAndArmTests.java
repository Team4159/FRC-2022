package subsystems;
//import org.junit.Test;
//import org.junit.Before;
//import org.junit.After;
import frc.robot.subsystems.Intake;
//import frc.robot.Constants.Direction;
import frc.robot.subsystems.Arm;

//import static org.junit.Assert.assertEquals;

public class IntakeAndArmTests {
    private static final double delta = 0.01;
    Intake intake;
    Arm arm;

    //@Before
    public void setup() {
        intake = new Intake();
        arm = new Arm();
    }

    // @Test
    // public void intakeTest() {
    //     System.out.println("when runIntake, will the intakeSpark run at the speeds the Constants define");

    //     intake.runIntake(Direction.FORWARDS);
    //     assertEquals(0.3, intake.getIntakeSpark().get(), delta);

    //     intake.runIntake(Direction.FORWARDS);
    //     assertEquals(0.3, intake.getIntakeSpark().get(), delta);

    // }

    // @Test
    // public void encoderTest() {
    //     System.out.println("Encoder should output a number");
    //     System.out.println("Encoder output: " + arm.getEncoderRaw());
    //     assertEquals(arm.getEncoderRaw(), 0, delta);
    // }

    // @Test
    // public void pidTest() {
    //     System.out.println("PID should output a number");
    //     System.out.println("PID output: " + arm.calculatePID(0, 0));
    //     arm.resetPID();
    //     assertEquals(arm.calculatePID(0, 0), 0, delta);
    // }

    //@After
    public void shutdown() {
        intake.close();
        arm.close();
    }

}
