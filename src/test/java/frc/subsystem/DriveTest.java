package frc.subsystem;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;


public class DriveTest {
    public static final double DELTA = 1.0E-2;
    Drive drive;
    RobotTracker robotTracker;
    Random random = new Random();

    private double currentMaxVelocityChange = 0;

    @BeforeEach
    void setUp() {
        drive = Drive.getInstance();
        drive.kill();
        robotTracker = RobotTracker.getInstance();
        robotTracker.kill();
    }

    @AfterEach
    void tearDown() throws Exception {

    }

    @Test
    void testAngleDiff1() {
        assertEquals(0, drive.getAngleDiff(0, 0));
    }

    @Test
    void testAngleDiff2() {
        assertEquals(-10, drive.getAngleDiff(0, 10));
    }

    @Test
    void testAngleDiff3() {
        assertEquals(-100, drive.getAngleDiff(0, 100));
    }

    @Test
    void testAngleDiff4() {
        assertEquals(170, drive.getAngleDiff(0, 190));
    }

    @Test
    void testAngleDiff5() {
        assertEquals(0, drive.getAngleDiff(0, 360));
    }

    @Test
    void testAngleDiff6() {
        assertEquals(30, drive.getAngleDiff(90, 60));
    }

    @Test
    void testAngleDif7() {
        assertEquals(-179, drive.getAngleDiff(241, 60));
    }

    @Test
    void testAngleDif8() {
        assertEquals(180, drive.getAngleDiff(240, 60));
    }

    @Test
    void testAngleDif9() {
        assertEquals(-20, drive.getAngleDiff(350, 10));
    }

    @Test
    void testAngleDif10() {
        assertEquals(20, drive.getAngleDiff(10, 350));
    }

    @Test
    void testAngleDif11() {
        assertEquals(10, drive.getAngleDiff(73, 63));
    }

    @Test
    void testAngleDif12() {
        assertEquals(116, drive.getAngleDiff(84, 328));
    }

    @Test
    void testAngleDif13() {
        assertEquals(-27, drive.getAngleDiff(96, 123));
    }

    @Test
    void testAngleDif14() {
        assertEquals(-3, drive.getAngleDiff(359, 2));
    }

    @Test
    void testAngleDif15() {
        assertEquals(133, drive.getAngleDiff(123, -10));
    }

    @Test
    void testAngleDif16() {
        assertEquals(-23, drive.getAngleDiff(240, 263));
    }

    @Test
    void testAngleDif17() {
        assertEquals(19, drive.getAngleDiff(189, 170));
    }
}
