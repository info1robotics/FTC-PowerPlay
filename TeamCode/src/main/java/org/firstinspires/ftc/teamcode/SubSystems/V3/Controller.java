package org.firstinspires.ftc.teamcode.SubSystems.V3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Controller {
    public Claw claw;
    public ClawFlip clawFlip;
    public Extendo extendo;
    public Pivot pivot;
    public Turret turret;
    public static Controller instance;

    public static Controller getInstance() {
        return instance;
    }

    public Controller(LinearOpMode opMode) {
        this.claw = new Claw(opMode);
        this.clawFlip = new ClawFlip(opMode);
        this.extendo = new Extendo(opMode);
        this.pivot = new Pivot(opMode);
        this.turret = new Turret(opMode);
        instance = this;
    }


    public void setCollectPivotAndClawFlip() {
        pivot.setCollect();
        clawFlip.setCollect();
    }

    public void setScorePivotAndClawFlip() {
        pivot.setScore();
        clawFlip.setScore();
    }

    public void setCollectPivotAndClawFlipAndExtendo() {
        pivot.setCollect();
        clawFlip.setCollect();
        extendo.setState(Extendo.ExtendoState.FULL);
    }

    public void setScorePivotAndClawFlipAndExtendo() {
        pivot.setScore();
        clawFlip.setScore();
        extendo.setState(Extendo.ExtendoState.RETRACTED);
    }
}
