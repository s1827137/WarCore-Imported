package org.jmhsrobotics.warcore.swerve;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class SwerveVisualizerTest {

    // Field Object Names
    private static final String ROBOT_BASE = "robotBase";
    private static final String FRONT_LEFT_WHEEL = "frontLeftWheel";
    private static final String FRONT_RIGHT_WHEEL = "frontRightWheel";
    private static final String BACK_LEFT_WHEEL = "backLeftWheel";
    private static final String BACK_RIGHT_WHEEL = "backRightWheel";

    @AfterEach
    public void cleanUp() {
        // NetworkTableInstance.getDefault().deleteAllEntries(); // Clean Networktables between tests
        //TODO: UPDATE broke cleanup
    }

    @Test
    public void testInitizationSquare() {
        SwerveVisualizer vis = new SwerveVisualizer(2, 2);
        Field2d f = vis.field;

        assertAll(() -> assertEquals(new Pose2d(0, 0, new Rotation2d()), f.getObject(ROBOT_BASE).getPose()),
                () -> assertEquals(new Pose2d(1, 1, new Rotation2d()), f.getObject(FRONT_LEFT_WHEEL).getPose()),
                () -> assertEquals(new Pose2d(1, -1, new Rotation2d()), f.getObject(FRONT_RIGHT_WHEEL).getPose()),
                () -> assertEquals(new Pose2d(-1, 1, new Rotation2d()), f.getObject(BACK_LEFT_WHEEL).getPose()),
                () -> assertEquals(new Pose2d(-1, -1, new Rotation2d()), f.getObject(BACK_RIGHT_WHEEL).getPose()));

    }

    @Test
    public void testInitizationRectangle() {
        SwerveVisualizer vis = new SwerveVisualizer(1, 6);
        Field2d f = vis.field;
        
        assertAll(() -> assertEquals(new Pose2d(0, 0, new Rotation2d()), f.getObject(ROBOT_BASE).getPose()),
                () -> assertEquals(new Pose2d(3, 0.5, new Rotation2d()), f.getObject(FRONT_LEFT_WHEEL).getPose()),
                () -> assertEquals(new Pose2d(3, -0.5, new Rotation2d()), f.getObject(FRONT_RIGHT_WHEEL).getPose()),
                () -> assertEquals(new Pose2d(-3, 0.5, new Rotation2d()), f.getObject(BACK_LEFT_WHEEL).getPose()),
                () -> assertEquals(new Pose2d(-3, -0.5, new Rotation2d()), f.getObject(BACK_RIGHT_WHEEL).getPose()));
    }

    @Test
    public void testMoveOnlyUpdate() {
        SwerveVisualizer vis = new SwerveVisualizer(2, 1);
        Field2d f = vis.field;

        vis.update(Rotation2d.fromDegrees(20), Rotation2d.fromDegrees(-20), Rotation2d.fromDegrees(30),
                Rotation2d.fromDegrees(180), new Pose2d(3, 6, new Rotation2d()));

        assertAll(() -> assertEquals(new Pose2d(3, 6, new Rotation2d()), f.getObject(ROBOT_BASE).getPose()),
                () -> assertEquals(new Pose2d(3.5, 7, Rotation2d.fromDegrees(20)),
                        f.getObject(FRONT_LEFT_WHEEL).getPose()),
                () -> assertEquals(new Pose2d(3.5, 5, Rotation2d.fromDegrees(-20)),
                        f.getObject(FRONT_RIGHT_WHEEL).getPose()),
                () -> assertEquals(new Pose2d(2.5, 7, Rotation2d.fromDegrees(30)),
                        f.getObject(BACK_LEFT_WHEEL).getPose()),
                () -> assertEquals(new Pose2d(2.5, 5, Rotation2d.fromDegrees(180)),
                        f.getObject(BACK_RIGHT_WHEEL).getPose()));
    }

    @Test
    public void testMoveAndRotateUpdate() {
        SwerveVisualizer vis = new SwerveVisualizer(2, 2);
        Field2d f = vis.field;

        vis.update(Rotation2d.fromDegrees(20), Rotation2d.fromDegrees(-20), Rotation2d.fromDegrees(30),
                Rotation2d.fromDegrees(180), new Pose2d(3, 6, Rotation2d.fromDegrees(45)));

        assertAll(() -> assertEquals(new Pose2d(3, 6, Rotation2d.fromDegrees(45)), f.getObject(ROBOT_BASE).getPose()),
                () -> assertEquals(new Pose2d(3, 6 + Math.sqrt(2), Rotation2d.fromDegrees(65)),
                        f.getObject(FRONT_LEFT_WHEEL).getPose()),
                () -> assertEquals(new Pose2d(3 + Math.sqrt(2), 6, Rotation2d.fromDegrees(25)),
                        f.getObject(FRONT_RIGHT_WHEEL).getPose()),
                () -> assertEquals(new Pose2d(3 - Math.sqrt(2), 6, Rotation2d.fromDegrees(75)),
                        f.getObject(BACK_LEFT_WHEEL).getPose()),
                () -> assertEquals(new Pose2d(3, 6 - Math.sqrt(2), Rotation2d.fromDegrees(225)),
                        f.getObject(BACK_RIGHT_WHEEL).getPose()));
    }
}
