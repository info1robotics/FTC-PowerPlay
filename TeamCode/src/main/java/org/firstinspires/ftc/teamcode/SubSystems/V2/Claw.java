package org.firstinspires.ftc.teamcode.SubSystems.V2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public Servo clawLeft;
    Servo clawRight;
    public Servo pivotSecondary, pivotMain;

    public double LEFT_OPEN = 1.0;
    public double LEFT_CLOSED = 0.95;
    public double RIGHT_OPEN = 0.0;
    public double RIGHT_CLOSED = 0.05;
    public double PIVOT_SECONDARY_DOWN = 0.09;
    public double PIVOT_SECONDARY_UP = 0.165;
    public double PIVOT_SECONDARY_INIT = 0.075;
    public double PIVOT_MAIN_DOWN = 0.15;
    public double PIVOT_MAIN_UP = 0.05;
    public double PIVOT_MAIN_INIT = 0.0;
    public double PIVOT_MAIN_DROP = 0.075;

    public static boolean isClosed;
    public enum clawStates {
        OPEN,
        CLOSED,
    }
    public enum pivotPositions {
        INIT,
        DOWN,
        UP
    }

    public enum subsystemStates{
        READY,
        COLLECTED,
        RETRACTED,
        DROP
    }

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
        switch(position){
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
        }
    }

    public void setSubsystemState(subsystemStates state){
        switch(state){
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
}
