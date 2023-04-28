package org.firstinspires.ftc.teamcode.SubSystems.V2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Claw {
    public static boolean isClosed;
    public Servo clawLeft, clawRight, loonyClaw;
    public Servo pivotSecondary, pivotMain;
    public static double LOONY_OPEN = 0.0; //calibrated
    public static double LOONY_CLOSED = 0.05; //calibrated 0.58
    public static double PIVOT_SECONDARY_DOWN = 0.12; //calibrated
    public static double PIVOT_SECONDARY_UP = 0.42; //calibrated
    public static double PIVOT_SECONDARY_UP_TELEOP = 0.19; //calibrated
    public static double PIVOT_SECONDARY_INIT = 0.225; //calibrated
    public static double PIVOT_MAIN_DOWN = 0.96; //calibrated 975
    public static double PIVOT_MAIN_UP = 0.40; //calibrated
    public static double PIVOT_MAIN_INIT = 0.40; //calibrated
    public static double PIVOT_MAIN_DROP = 0.7;
    public static double PIVOT_MAIN_VERTICAL = 0.86; //calibrated 335
    public static double PIVOT_SECONDARY_VERTICAL = 0.32;
    public static double PIVOT_MAIN_AUTODROP = 0.44;
    public static double PIVOT_SECONDARY_INTERMEDIARY = 0.0;
    public static double PIVOT_SECONDARY_EXTENDED = 0.36; //calibrated
    public static double PIVOT_SECONDARY_EXTENDED_DROP = 0.335;
    public static double PIVOT_MAIN_EXTENDED = 0.87; //calibrated

    public static double PIVOT_MAIN_FLIPPED = 0.93;
    public static double PIVOT_SECONDARY_FLIPPED = 0.33;

//    public static double LEFT_OPEN = 1.0;
//    public static double LEFT_CLOSED = 0.95;
//    public static double RIGHT_OPEN = 0.0;
//    public static double RIGHT_CLOSED = 0.05;

    public Claw(LinearOpMode opMode) {
//        clawLeft = opMode.hardwareMap.get(Servo.class, "ClawLeft");
//        clawRight = opMode.hardwareMap.get(Servo.class, "ClawRight");
        pivotSecondary = opMode.hardwareMap.get(Servo.class, "PivotSecondary");
        pivotMain = opMode.hardwareMap.get(Servo.class, "PivotMain");
        loonyClaw = opMode.hardwareMap.get(Servo.class, "LoonyClaw");
    }

    public void setClawState(clawStates state) {
        if (state == clawStates.OPEN) {
            // Opens the Claw
//            clawRight.setPosition(RIGHT_OPEN);
//            clawLeft.setPosition(LEFT_OPEN);
            loonyClaw.setPosition(LOONY_OPEN);
        } else if (state == clawStates.CLOSED) {
            // Closes the Claw
//            clawRight.setPosition(RIGHT_CLOSED);
//            clawLeft.setPosition(LEFT_CLOSED);
            loonyClaw.setPosition(LOONY_CLOSED);
        }
    }

    public void setPivotPosition(pivotPositions position) {
        switch (position) {
            case INIT:
                pivotMain.setPosition(PIVOT_MAIN_INIT);
                pivotSecondary.setPosition(PIVOT_SECONDARY_INIT);
                break;
            case UP:
                pivotMain.setPosition(PIVOT_MAIN_UP);
                pivotSecondary.setPosition(PIVOT_SECONDARY_DOWN);
                break;
            case DOWN:
                pivotSecondary.setPosition(PIVOT_SECONDARY_UP);
                pivotMain.setPosition(PIVOT_MAIN_DOWN);
                break;
            case DOWN_COLLECTED:
                pivotSecondary.setPosition(0.26);
                pivotMain.setPosition(PIVOT_MAIN_DOWN);
                break;
            case RIGHT:
                pivotSecondary.setPosition(PIVOT_SECONDARY_VERTICAL);
                pivotMain.setPosition(PIVOT_MAIN_VERTICAL);
                break;
            case AUTODROP:
                pivotMain.setPosition(PIVOT_MAIN_AUTODROP);
                break;
            case INTERMEDIARY:
                pivotMain.setPosition(PIVOT_MAIN_UP);
                pivotSecondary.setPosition(PIVOT_SECONDARY_INTERMEDIARY);
                break;
            case DOWNTELEOP:
                pivotSecondary.setPosition(PIVOT_SECONDARY_UP_TELEOP);
                pivotMain.setPosition(PIVOT_MAIN_DOWN);
                break;
            case ANGLE:
                pivotSecondary.setPosition(PIVOT_SECONDARY_EXTENDED);
                pivotMain.setPosition(PIVOT_MAIN_EXTENDED);
                break;
            case ANGLE_DROP:
                pivotSecondary.setPosition(PIVOT_SECONDARY_EXTENDED_DROP);
                pivotMain.setPosition(PIVOT_MAIN_EXTENDED);
                break;
            case DOWNWARDS:
                pivotSecondary.setPosition(PIVOT_SECONDARY_FLIPPED);
                pivotMain.setPosition(PIVOT_MAIN_FLIPPED);
                break;
        }
    }

    public void setSubsystemState(subsystemStates state) {
        switch (state) {
            case READY:
                setClawState(clawStates.OPEN);
                setPivotPosition(pivotPositions.DOWN);
                break;
            case READY_COLLECTED:
                setClawState(clawStates.CLOSED);
                setPivotPosition(pivotPositions.DOWN_COLLECTED);
                break;
            case COLLECTED:
                setClawState(clawStates.CLOSED);
                setPivotPosition(pivotPositions.UP);
                break;
            case RETRACTED:
                setClawState(clawStates.CLOSED);
                setPivotPosition(pivotPositions.INIT);
                break;
            case DROP:
                pivotMain.setPosition(PIVOT_MAIN_DROP);
                setClawState(clawStates.OPEN);
                break;
            case VERTICAL:
                setPivotPosition(pivotPositions.RIGHT);
                setClawState(clawStates.CLOSED);
                break;
            case READY_TELEOP:
                setClawState(clawStates.OPEN);
                setPivotPosition(pivotPositions.DOWNTELEOP);
                break;
            case EXTENDED:
                setClawState(clawStates.CLOSED);
                setPivotPosition(pivotPositions.ANGLE);
                break;
            case EXTENDED_DROP:
                setClawState(clawStates.CLOSED);
                setPivotPosition(pivotPositions.ANGLE_DROP);
                break;
        }
    }

    public void toggle() {
        isClosed = !isClosed;
        if (!isClosed) {
            // Opens the Claw
//            clawRight.setPosition(RIGHT_OPEN);
//            clawLeft.setPosition(LEFT_OPEN);
            loonyClaw.setPosition(LOONY_OPEN);
        } else {
            // Closes the Claw
//            clawRight.setPosition(RIGHT_CLOSED);
//            clawLeft.setPosition(LEFT_CLOSED);
            loonyClaw.setPosition(LOONY_CLOSED);
        }
    }

    public enum clawStates {
        OPEN, CLOSED,
    }

    public enum pivotPositions {
        INIT, DOWN, DOWN_COLLECTED, UP, RIGHT, AUTODROP, INTERMEDIARY, DOWNTELEOP, ANGLE, ANGLE_DROP, DOWNWARDS;
    }

    public enum subsystemStates {
        READY, READY_COLLECTED, COLLECTED, RETRACTED, DROP, VERTICAL, READY_TELEOP, EXTENDED, EXTENDED_DROP, FLIPPED;
    }
}
