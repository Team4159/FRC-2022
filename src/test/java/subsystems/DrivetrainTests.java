package subsystems;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.Drivetrain;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class DrivetrainTests {
    public static final double delta = 1e-2; // 0.01
    Drivetrain drivetrain;

    @Before
    public void setup() {
        assert HAL.initialize(500, 0);
        drivetrain = new Drivetrain();
    }

    @After
    public void shutdown() throws Exception {
        drivetrain.close();
    }

    @Test
    public void test() {
        System.out.println("0 should equal 0");
        assertEquals(0,0);
    }

    @Test
    public void driveTest() {
        System.out.println("Drivetrain should accelerate");
        drivetrain.drive(0.5, 0.5);
        assertEquals(0.5, drivetrain.getSpeed(), delta);
    }
}
