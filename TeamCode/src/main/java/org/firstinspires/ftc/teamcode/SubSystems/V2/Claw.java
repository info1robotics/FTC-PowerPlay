package org.firstinspires.ftc.teamcode.SubSystems.V2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Claw {
    public static boolean isClosed;
    public Servo clawLeft;
    public Servo pivotSecondary, pivotMain;

    public static double LEFT_OPEN = 1.0;
    public static double LEFT_CLOSED = 0.95;
    public static double RIGHT_OPEN = 0.0;
    public static double RIGHT_CLOSED = 0.05;
    public static double PIVOT_SECONDARY_DOWN = 0.09;
    public static double PIVOT_SECONDARY_UP = 0.16;
    public static double PIVOT_SECONDARY_INIT = 0.09;
    public static double PIVOT_MAIN_DOWN = 0.8;
    public static double PIVOT_MAIN_UP = 0.3;
    public static double PIVOT_MAIN_INIT = 0.0;
    public static double PIVOT_MAIN_DROP = 0.7;
    public static double PIVOT_MAIN_VERTICAL = 0.3;
    public static double PIVOT_SECONDARY_VERTICAL = 0.06;
    public static double PIVOT_MAIN_AUTODROP = 0.45;
    public static double PIVOT_SECONDARY_INTERMEDIARY = 0.0;
    Servo clawRight;

    public Claw(LinearOpMode opMode) {
        clawLeft = opMode.hardwareMap.get(Servo.class, "ClawLeft");
        clawRight = opMode.hardwareMap.get(Servo.class, "ClawRight");
        pivotSecondary = opMode.hardwareMap.get(Servo.class, "PivotSecondary");
        pivotMain = opMode.hardwareMap.get(Servo.class, "PivotMain");
    }

    public void setClawState(clawStates state) {
        if (state == clawStates.OPEN) {
            // Opens the Claw
            clawRight.setPosition(RIGHT_OPEN);
            clawLeft.setPosition(LEFT_OPEN);
        } else if (state == clawStates.CLOSED) {
            // Closes the Claw
            clawRight.setPosition(RIGHT_CLOSED);
            clawLeft.setPosition(LEFT_CLOSED);
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
        }
    }

    public void setSubsystemState(subsystemStates state) {
        switch (state) {
            case READY:
                setClawState(clawStates.OPEN);
                setPivotPosition(pivotPositions.DOWN);
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
            case VERTICAL:
                setPivotPosition(pivotPositions.RIGHT);
                setClawState(clawStates.CLOSED);
        }
    }

    public void toggle() {
        isClosed = !isClosed;
        if (!isClosed) {
            // Opens the Claw
            clawRight.setPosition(RIGHT_OPEN);
            clawLeft.setPosition(LEFT_OPEN);
        } else {
            // Closes the Claw
            clawRight.setPosition(RIGHT_CLOSED);
            clawLeft.setPosition(LEFT_CLOSED);
        }
    }

    public enum clawStates {
        OPEN, CLOSED,
    }

    public enum pivotPositions {
        INIT, DOWN, UP, RIGHT, AUTODROP, INTERMEDIARY
    }

    public enum subsystemStates {
        READY, COLLECTED, RETRACTED, DROP, VERTICAL
    }
}
