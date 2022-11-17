package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Mecanum {
	private DcMotor fl, fr, bl, br;

	public Mecanum(HardwareMap hardwareMap)
	{
		fl = hardwareMap.get(DcMotor.class, "DrivetrainFL");
		fr = hardwareMap.get(DcMotor.class, "DrivetrainFR");
		bl = hardwareMap.get(DcMotor.class, "DrivetrainBL");
		br = hardwareMap.get(DcMotor.class, "DrivetrainBR");


		br.setDirection(DcMotorSimple.Direction.REVERSE);
		bl.setDirection(DcMotorSimple.Direction.REVERSE);
		fl.setDirection(DcMotorSimple.Direction.FORWARD);
		fr.setDirection(DcMotorSimple.Direction.FORWARD);
	}

	public void vectorMove(double x, double y, double t, double power)
	{
		double[] targetPower = normalize( new double[]{
				(x + y + t),
				(y - x - t),
				(y - x + t),
				(x + y - t)
		});

		fl.setPower(targetPower[0] * power);
		fr.setPower(targetPower[1] * power);
		bl.setPower(targetPower[2] * power);
		br.setPower(targetPower[3] * power);

	}

	private double[] normalize(double[] values)
	{
		// Put powers in the range of -1 to 1 only if they aren't already
		// Not checking would cause us to always drive at full speed
		if (Math.abs(values[0]) > 1 || Math.abs(values[2]) > 1 ||
				Math.abs(values[1]) > 1 || Math.abs(values[3]) > 1) {
			double max;
			max = Math.max(Math.abs(values[0]), Math.abs(values[2]));
			max = Math.max(Math.abs(values[1]), max);
			max = Math.max(Math.abs(values[3]), max);

			values[0] /= max;
			values[1] /= max;
			values[2] /= max;
			values[3] /= max;
		}
		return values;
	}
}