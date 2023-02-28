package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SubSystems.V2.Claw;

@Autonomous
@Config
public class TestPivot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(this);
        claw.setSubsystemState(Claw.subsystemStates.COLLECTED);
        waitForStart();
        claw.setSubsystemState(Claw.subsystemStates.DROP);
        while(!isStopRequested());
    }
}
