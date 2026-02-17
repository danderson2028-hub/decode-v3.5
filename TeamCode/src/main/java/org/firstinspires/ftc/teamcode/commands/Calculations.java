package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.Pose;

public class Calculations {
    public static Pose redGoalPose = new Pose(128.0, 132.0);
    public static Pose blueGoalPose = new Pose(16.0, 132.0);

    public static Pose goalPose = redGoalPose;
    public static double getTurretAngle(Pose botPose){
        // Calculate angle from robot to target
        double fieldAngle = Math.atan2(
                goalPose.getY() - botPose.getY(),
                goalPose.getX() - botPose.getX()
        );

        // Calculate turret angle relative to robot
        double turretAngle = fieldAngle - botPose.getHeading();

        // Normalize to [-π, π]
        turretAngle = normalizeAngle(turretAngle);

        // Convert to degrees
        double angleDeg = Math.toDegrees(turretAngle);

        // Scale to motor ticks
        double turretTarget = angleDeg * -2.872;

        // Apply limits
        turretTarget = Math.max(-250, Math.min(250, turretTarget));

        return turretTarget;
    }

    // Helper method to normalize angle to [-π, π]
    private static double normalizeAngle(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public static double getShooterRPM(double distance){
        return 376.2412 * Math.pow(distance,0.284006);
    }

    public static double getHoodAngle(double distance){
        double angle = (4.28937*Math.pow(10,-7))*Math.pow(distance,3)-0.000111902*Math.pow(distance,2)+0.0129326*distance-0.32834;
        return Math.max(Math.min(angle,0.4),0.0);
    }

    public static double getTransferSpeed(double distance){
        return -0.00588235 * distance + 1.10588;
    }
}
