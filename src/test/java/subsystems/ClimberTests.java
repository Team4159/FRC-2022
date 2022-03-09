package subsystems;
import org.junit.Test;
import org.junit.Before;
import org.junit.After;
import frc.robot.subsystems.Climber;

import static org.junit.Assert.assertEquals;


public class ClimberTests {
    private static final double delta = 0.01;
    Climber climber;

    @Before
    public void setup() {
        climber = new Climber();
    }

    // @Test
    // public void climberEncoderTest() {
    //     System.out.println("Climber Encoder should output a number");
    //     System.out.println("Encoder output: " + climber.getEncoderRaw());
    //     assertEquals(climber.getEncoderRaw(), 0, delta);
    // }

    @Test
    public void climberPidTest() {
        System.out.println("Climber PID should output a number");
        System.out.println("PID output: " + climber.calculatePID(0, 0));
        climber.resetPID();
        assertEquals(climber.calculatePID(0, 0), 0, delta);
    }

    @After
    public void shutdown() {
        climber.close();
    }
}
