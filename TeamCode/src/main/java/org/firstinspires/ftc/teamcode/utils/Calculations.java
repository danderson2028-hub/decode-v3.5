package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;

import org.opencv.core.Mat;

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
        double rpm = 0.0000268987*Math.pow(distance,4)-0.00918067* Math.pow(distance,3) +1.07798*Math.pow(distance,2)-46.04816*distance+1651.96334;
        return Math.max(Math.min(rpm,1525),1000);
    }

    public static double getHoodAngle(double distance){
        double angle = -0.000050098*(Math.pow(distance,2))+0.0125441*distance-0.314086;
        return Math.max(Math.min(angle,0.47),0.0);
    }
    public static double getFlightTime(double distance){
        return 0.4;
    }
    public static double getTransferSpeed(double distance){
        return -0.00636*distance + 1.1909;
    }

    public static double getDist(Pose botPose){
        return Math.hypot(goalPose.getY()-botPose.getY(),goalPose.getX()-botPose.getX());
    }
}
