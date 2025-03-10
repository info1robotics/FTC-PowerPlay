package org.firstinspires.ftc.teamcode.OpModes.V3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommonPackage.GamepadEx;

/**
 * This sample illustrates using the touchpad feature found on some gamepads.
 *
 * The Sony PS4 gamepad can detect two distinct touches on the central touchpad.
 * Other gamepads with different touchpads may provide mixed results.
 *
 * The touchpads are accessed through the standard gamepad1 and gamepad2 objects.
 *   Several new members were added to the Gamepad class in FTC SDK Rev 7
 *
 *   .touchpad_finger_1     returns true if at least one finger is detected.
 *   .touchpad_finger_1_x   finger 1 X coordinate.  Valid if touchpad_finger_1 is true
 *   .touchpad_finger_1_y   finger 1 Y coordinate.  Valid if touchpad_finger_1 is true
 *
 *   .touchpad_finger_2     returns true if a second finger is detected
 *   .touchpad_finger_2_x   finger 2 X coordinate.  Valid if touchpad_finger_2 is true
 *   .touchpad_finger_2_y   finger 2 Y coordinate.  Valid if touchpad_finger_2 is true
 *
 * Finger touches are reported with an X and Y coordinate in following coordinate system.
 *
 *   1) X is the Horizontal axis, and Y is the vertical axis
 *   2) The 0,0 origin is at the center of the touchpad.
 *   3)  1.0, 1.0 is at the top right corner of the touchpad.
 *   4) -1.0,-1.0 is at the bottom left corner of the touchpad.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */

@TeleOp(name="Full Debug")
public class FullDebug extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        int profile = 0;
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        telemetry.addData(">", "Press Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad1.touchpad) {
                profile = (profile + 1) % 2;
            }

            if (profile == 0) {

            }

            telemetry.addLine("Use touchpad to change profile");
            telemetry.update();
        }
    }
}