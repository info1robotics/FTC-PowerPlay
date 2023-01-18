package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.*;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "test")
public class AutoTest extends AutoOpMode {
    @Override
    public void onInit() {
        task = sync (
                pause(100),
//                trajectory(...),
                async(
                        pause(100),
//                        trajectory(...),
                        inline(() -> {

                        })
                )
        );
    }
}
