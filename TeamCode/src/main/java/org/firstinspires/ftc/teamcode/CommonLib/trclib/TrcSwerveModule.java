/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.CommonLib.trclib;

/**
 * This class implements a platform independent Swerve Drive module. A Swerve Drive module consists of a drive motor
 * and a steer motor. The steer motor can be a PID controlled motor with a zero calibration limit switch that allows
 * an absolute steering angle to be set and held. It can also be a servo motor which has a limited range of motion.
 */
public class TrcSwerveModule
{
    private static final String moduleName = "TrcSwerveModule";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    public final TrcMotor driveMotor;
    public final TrcPidMotor steerMotor;
    public final TrcServo steerServo;
    private final TrcWarpSpace warpSpace;
    private boolean steerLimitsEnabled = false;
    private double steerLowLimit = 0.0;
    private double steerHighLimit = 0.0;
    private double optimizedWheelDir = 1.0;

    /**
     * Constructor: Create an instance of the object.
     * Note: steerMotor and steerServo are exclusive. You can either have a steerMotor or a steerServo but not both.
     *
     * @param instanceName specifies the instance name.
     * @param driveMotor   specifies the drive motor.
     * @param steerMotor   specifies the steering motor.
     * @param steerServo   specifies the steering servo.
     */
    private TrcSwerveModule(
            String instanceName, TrcMotor driveMotor, TrcPidMotor steerMotor, TrcServo steerServo)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer ?
                        TrcDbgTrace.getGlobalTracer() :
                        new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.steerServo = steerServo;
        warpSpace = new TrcWarpSpace(instanceName + ".warpSpace", 0.0, 360.0);
    }   //TrcSwerveModule

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param driveMotor   specifies the drive motor.
     * @param steerMotor   specifies the steering motor.
     */
    public TrcSwerveModule(String instanceName, TrcMotor driveMotor, TrcPidMotor steerMotor)
    {
        this(instanceName, driveMotor, steerMotor, null);
    }   //TrcSwerveModule

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param driveMotor   specifies the drive motor.
     * @param steerServo   specifies the steering servo.
     */
    public TrcSwerveModule(String instanceName, TrcMotor driveMotor, TrcServo steerServo)
    {
        this(instanceName, driveMotor, null, steerServo);
    }   //TrcSwerveModule

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method sets the hard steer limits, used for noncontinuous swerve modules. The angles must be in range
     * (-180,180]. The limits must also be at least 180 degrees apart.
     *
     * @param steerLowLimit  The low steer limit.
     * @param steerHighLimit The high steer limit.
     */
    public void setSteeringLimits(double steerLowLimit, double steerHighLimit)
    {
        if (steerHighLimit - steerLowLimit < 180.0)
        {
            throw new IllegalArgumentException("steerLowLimit must be at least 180 less than steerHighLimit!");
        }

        this.steerLimitsEnabled = true;
        this.steerLowLimit = steerLowLimit;
        this.steerHighLimit = steerHighLimit;
    }   //setSteeringLimits

    /**
     * This method disables the steer limits.
     */
    public void disableSteeringLimits()
    {
        this.steerLimitsEnabled = false;
        this.steerLowLimit = this.steerHighLimit = 0.0;
    }   //disableSteeringLimits

    /**
     * This method performs a zero calibration on the steering motor. This is not applicable for servo steering.
     */
    public void zeroCalibrateSteering()
    {
        final String funcName = "zeroCalibrateSteering";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (steerMotor != null)
        {
            steerMotor.zeroCalibrate(this::doneZeroCalibrate);
        }
        else
        {
            throw new RuntimeException("Zero calibration is not applicable for servo steering.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //zeroCalibrateSteering

    /**
     * This method is called when zero calibration is done.
     *
     * @param context not used.
     */
    private void doneZeroCalibrate(Object context)
    {
        setSteerAngle(0.0, false, true);
    }   //doneZeroCalibrate

//    /**
//     * This method calibrates the steering range of the steering servo.
//     *
//     * @param stepRate specifies the step rate of the servo.
//     */
//    public void rangeCalibrateSteering(double stepRate)
//    {
//        final String funcName = "rangeCalibrateSteering";
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "stepRate=%f", stepRate);
//        }
//
//        if (steerServo != null)
//        {
//            steerServo.rangeCalibrate(-180.0, 180.0, stepRate);
//            setSteerAngle(0.0, false, true);
//        }
//        else
//        {
//            throw new RuntimeException("Steering range calibration is only applicable for servo steering.");
//        }
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
//        }
//    }   //rangeCalibrateSteering

    /**
     * This method sets the steer angle.
     *
     * @param angle    specifies the angle in degrees to set the steer motor to. Not necessarily within [0,360).
     * @param optimize specifies true to optimize steering angle to be no greater than 90 degrees, false otherwise.
     * @param hold     specifies true to hold the angle, false otherwise.
     */
    public void setSteerAngle(double angle, boolean optimize, boolean hold)
    {
        final String funcName = "setSteerAngle";
        double prevSteerAngle = getSteerAngle();
        angle = warpSpace.getOptimizedTarget(angle, prevSteerAngle);
        double angleDelta = angle - prevSteerAngle;
        double newAngle = angle;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "angle=%f,optimize=%s,hold=%s", angle, optimize,
                hold);
        }

        // If we are not optimizing, reset wheel direction back to normal.
        optimizedWheelDir = 1.0;
        if (optimize && Math.abs(angleDelta) > 90.0)
        {
            // We are optimizing and the steer delta is greater than 90 degrees.
            // Adjust the steer delta to be within 90 degrees and flip the wheel direction.
            newAngle += angleDelta < 0.0 ? 180.0 : -180.0;
            optimizedWheelDir = -1.0;
        }

        if (steerLimitsEnabled)
        {
            double boundAngle = TrcUtil.modulo(newAngle, 360.0); // Bound angle within [0,360).
            // Convert bound angle to range (-180,180].
            boundAngle = boundAngle > 180 ? boundAngle - 360.0 : boundAngle;
            // CodeReview: do limits fall between -359 and 359, centered at zero? If so, boundAngle should never
            // fall outside the limits.
            if (boundAngle < steerLowLimit)
            {
                newAngle = boundAngle + 180;
                optimizedWheelDir *= -1;
            }
            else if (boundAngle > steerHighLimit)
            {
                newAngle = boundAngle - 180;
                optimizedWheelDir *= -1;
            }
        }

        if (steerMotor != null)
        {
            steerMotor.setTarget(newAngle, hold);
        }
        else if (steerServo != null)
        {
            steerServo.setPosition(newAngle);
        }

        if (debugEnabled)
        {
            if (optimize)
            {
                dbgTrace.traceInfo(funcName, "Optimizing steer angle for %s: %.1f -> %.1f (%.0f)",
                    instanceName, angle, newAngle, optimizedWheelDir);
            }
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, " (angle=%f)", angle);
        }
    }   //setSteerAngle

    /**
     * This method sets the steer angle.
     *
     * @param angle    specifies the angle in degrees to set the steer motor to, in the range [0,360).
     * @param optimize specifies true to optimize steering angle to be no greater than 90 degrees, false otherwise.
     */
    public void setSteerAngle(double angle, boolean optimize)
    {
        setSteerAngle(angle, optimize, true);
    }   //setSteerAngle

    /**
     * This method sets the steer angle.
     *
     * @param angle specifies the angle in degrees to set the steer motor to, in the range [0,360).
     */
    public void setSteerAngle(double angle)
    {
        setSteerAngle(angle, true, true);
    }   //setSteerAngle

    /**
     * The current angle of the turn motor. This is not necessarily the target angle.
     *
     * @return The angle of the turn motor, in degrees, in the range [0,360).
     */
    public double getSteerAngle()
    {
        final String funcName = "getSteerAngle";
        double angle = steerMotor != null ? steerMotor.getPosition() : steerServo.getPosition();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", angle);
        }

        return angle;
    }   //getSteerAngle

    /**
     * This method sets the motor output value. The value can be power or velocity percentage depending on whether
     * the motor controller is in power mode or velocity mode.
     *
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     */
    public void set(double value)
    {
        final String funcName = "set";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "value=%f", value);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        driveMotor.set(value * optimizedWheelDir);
    }   //set

}   //class TrcSwerveModule
