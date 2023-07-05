package org.firstinspires.ftc.teamcode.CommonPackage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class AutoUtils {
    public static Pose2d convertToNormalAxis(Pose2d old) {
        return new Pose2d(-old.getY(), old.getX());
    }

    public static int getDistance(Pose2d targetPosition, Pose2d currentPosition) {
        double x = targetPosition.getX() - currentPosition.getX();
        double y = targetPosition.getY() - currentPosition.getY();
        return (int) Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public static int getDistance(Vector2d targetPosition, Pose2d currentPosition) {
        return getDistance(new Pose2d(targetPosition.getX(), targetPosition.getY(), 0), currentPosition);
    }

    public static int getDistance(Pose2d targetPosition, Vector2d currentPosition) {
        return getDistance(targetPosition, new Pose2d(currentPosition.getX(), currentPosition.getY(), 0));
    }

    public static int getDistance(Vector2d targetPosition, Vector2d currentPosition) {
        return getDistance(new Pose2d(targetPosition.getX(), targetPosition.getY(), 0), new Pose2d(currentPosition.getX(), currentPosition.getY(), 0));
    }

    public static Pose2d pose(double x, double y, double heading) {
        return new Pose2d(x, y, heading);
    }

    public static Pose2d pose(double x, double y) {
        return new Pose2d(x, y, 0);
    }

    public static Vector2d vector(double x, double y) {
        return new Vector2d(x, y);
    }
}
