/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.Locale;

/**
 * This class implements a platform independent auto-assist intake subsystem. It contains a motor or a continuous
 * servo and a sensor that detects if the intake has captured objects. It provides the autoAssist method that allows
 * the caller to call the intake subsystem to pickup or dump objects on a press of a button and the intake subsystem
 * will stop itself once it is done. While it provides the auto-assist functionality to pickup or dump objects, it
 * also supports exclusive subsystem access by implementing TrcExclusiveSubsystem. This enables the intake subsystem
 * to be aware of multiple callers' access to the subsystem. While one caller starts the intake for an operation,
 * nobody can access it until the previous caller is done with the operation.
 */
public class TrcIntake implements TrcExclusiveSubsystem
{
    /**
     * This class contains all the parameters related to the intake.
     */
    public static class Parameters
    {
        public TrcDbgTrace msgTracer = null;
        public boolean motorInverted = false;
        public boolean triggerInverted = false;
        public Double analogThreshold = null;

        /**
         * This method returns the string form of all the parameters.
         *
         * @return string form of all the parameters.
         */
        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "motorInverted=%s,triggerInverted=%s,analogThreshold=%s",
                motorInverted, triggerInverted, analogThreshold);
        }   //toString

        /**
         * This method sets the message tracer for logging trace messages.
         *
         * @param tracer specifies the tracer for logging messages.
         * @return this parameter object.
         */
        public Parameters setMsgTracer(TrcDbgTrace tracer)
        {
            this.msgTracer = tracer;
            return this;
        }   //setMsgTracer

        /**
         * This method sets the direction of the motor.
         *
         * @param inverted specifies true if motor is inverted, false otherwise.
         * @return this parameter object.
         */
        public Parameters setMotorInverted(boolean inverted)
        {
            this.motorInverted = inverted;
            return this;
        }   //setMotorInverted

        /**
         * This method sets the trigger to be inverted. If it is an analog trigger, inverted means triggering when
         * sensor value is lower than threshold. If it is a digital trigger, inverted means triggering on inactive
         * state.
         *
         * @param inverted specifies true to invert the trigger, false otherwise.
         * @return this parameter object.
         */
        public Parameters setTriggerInverted(boolean inverted)
        {
            this.triggerInverted = inverted;
            return this;
        }   //setTriggerInverted

        /**
         * This method sets the anlog sensor threshold value.
         *
         * @param threshold specifies the sensor threshold value.
         * @return this parameter object.
         */
        public Parameters setAnalogThreshold(double threshold)
        {
            this.analogThreshold = threshold;
            return this;
        }   //setAnalogThreshold

    }   //class Parameters

    /**
     * This class encapsulates all the parameters required to perform the intake action.
     */
    private static class ActionParams
    {
        double power;
        TrcEvent event;
        double timeout;

        ActionParams(double power, TrcEvent event, double timeout)
        {
            this.power = power;
            this.event = event;
            this.timeout = timeout;
        }   //ActionParams

        @Override
        public String toString()
        {
            return String.format(Locale.US, "power=%.1f,event=%s,timeout=%.3f", power, event, timeout);
        }   //toString

    }   //class ActionParams

    private final String instanceName;
    private final TrcMotor motor;
    private final TrcServo servo;
    private final Parameters params;
    private final TrcTrigger sensorTrigger;
    private final TrcTimer timer;
    private final TrcEvent timerEvent;
    private ActionParams actionParams = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param motor specifies the motor object.
     * @param servo specifies the continuous servo object.
     * @param params specifies the parameters object.
     * @param sensorTrigger specifies the sensor trigger object.
     */
    private TrcIntake(
        String instanceName, TrcMotor motor, TrcServo servo, Parameters params, TrcTrigger sensorTrigger)
    {
        if (servo != null && !servo.isContinuous())
        {
            throw new RuntimeException("Servo must be a continuous servo.");
        }

        this.instanceName = instanceName;
        this.motor = motor;
        this.servo = servo;
        this.params = params;
        this.sensorTrigger = sensorTrigger;
        if (motor != null)
        {
            motor.setInverted(params.motorInverted);
        }
        else
        {
            servo.setInverted(params.motorInverted);
        }
        timer = new TrcTimer(instanceName);
        timerEvent = new TrcEvent(instanceName + ".timerEvent");
    }   //TrcIntake

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param motor specifies the motor object.
     * @param params specifies the parameters object.
     * @param sensorTrigger specifies the sensor trigger object.
     */
    public TrcIntake(
        String instanceName, TrcMotor motor, Parameters params, TrcTrigger sensorTrigger)
    {
        this(instanceName, motor, null, params, sensorTrigger);
    }   //TrcIntake

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param servo specifies the continuous servo object.
     * @param params specifies the parameters object.
     * @param sensorTrigger specifies the sensor trigger object.
     */
    public TrcIntake(
        String instanceName, TrcServo servo, Parameters params, TrcTrigger sensorTrigger)
    {
        this(instanceName, null, servo, params, sensorTrigger);
    }   //TrcIntake

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param motor specifies the motor object.
     * @param params specifies the parameters object.
     */
    public TrcIntake(String instanceName, TrcMotor motor, Parameters params)
    {
        this(instanceName, motor, null, params, null);
    }   //TrcIntake

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param servo specifies the continuous servo object.
     * @param params specifies the parameters object.
     */
    public TrcIntake(String instanceName, TrcServo servo, Parameters params)
    {
        this(instanceName, null, servo, params, null);
    }   //TrcIntake

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
     * This method returns the current motor power.
     *
     * @return current motor power.
     */
    public double getPower()
    {
        return motor != null? motor.getMotorPower(): servo.getPower();
    }   //getPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param delay specifies the delay in seconds to wait before setting the power of the motor.
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param time specifies the time period in seconds to have power set.
     * @param event specifies the event to signal when time has expired.
     */
    public void setPower(String owner, double delay, double power, double time, TrcEvent event)
    {
        if (motor != null)
        {
            motor.set(owner, delay, power, time, event);
        }
        else
        {
            servo.setPower(owner, delay, power, time, event);
        }
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param delay specifies the delay in seconds to wait before setting the power of the motor.
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param time specifies the time period in seconds to have power set.
     * @param event specifies the event to signal when time has expired.
     */
    public void setPower(double delay, double power, double time, TrcEvent event)
    {
        setPower(null, delay, power, time, event);
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param time specifies the time period in seconds to have power set.
     * @param event specifies the event to signal when time has expired.
     */
    public void setPower(double power, double time, TrcEvent event)
    {
        setPower(null, 0.0, power, time, event);
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param delay specifies the delay in seconds to wait before setting the power of the motor.
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param time specifies the time period in seconds to have power set.
     */
    public void setPower(double delay, double power, double time)
    {
        setPower(null, delay, power, time, null);
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param time specifies the time period in seconds to have power set.
     */
    public void setPower(double power, double time)
    {
        setPower(null, 0.0, power, time, null);
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     */
    public void setPower(double power)
    {
        setPower(null, 0.0, power, 0.0, null);
    }   //setPower

    /**
     * This method performs the auto-assist action.
     *
     * @param context specifies the action parameters.
     */
    private void performAutoAssist(Object context)
    {
        final String funcName = "performAutoAssist";
        ActionParams actionParams = (ActionParams) context;
        boolean objCaptured = hasObject();

        if (actionParams.power > 0.0 ^ objCaptured)
        {
            // Picking up object and we don't have one yet, or dumping object and we still have one.
            if (params.msgTracer != null)
            {
                params.msgTracer.traceInfo(funcName, "AutoAssist: %s, hasObject=%s", actionParams, objCaptured);
            }

            if (motor != null)
            {
                motor.set(actionParams.power);
            }
            else
            {
                servo.setPower(actionParams.power);
            }
            sensorTrigger.setEnabled(true);

            if (actionParams.timeout > 0.0)
            {
                timerEvent.setCallback(this::finishAutoAssist, null);
                timer.set(actionParams.timeout, timerEvent);
            }
        }
        else
        {
            // Picking up object but we already have one, or dumping object but there isn't any.
            if (params.msgTracer != null)
            {
                params.msgTracer.traceInfo(funcName, "Already done: hasObject=%s", objCaptured);
            }

            finishAutoAssist(null);
        }
    }   //performAutoAssist

    /**
     * This method is called either at the end of the timeout or when object is detected to finish the auto-assist
     * operation and signal the caller for completion.
     *
     * @param context specifies the action parameters (not used).
     */
    public void finishAutoAssist(Object context)
    {
        final String funcName = "finishAutoAssist";

        if (isAutoAssistActive())
        {
            if (params.msgTracer != null)
            {
                params.msgTracer.traceInfo(funcName, "AutoAssistTimedOut=%s", timerEvent.isSignaled());
            }

            if (motor != null)
            {
                motor.set(0.0);
            }
            else
            {
                servo.setPower(0.0);
            }
            timer.cancel();
            sensorTrigger.setEnabled(false);

            if (actionParams.event != null)
            {
                actionParams.event.signal();
                actionParams.event = null;
            }

            actionParams = null;
        }
    }   //finishAutoAssist

    /**
     * This method is an auto-assist operation. It allows the caller to start the intake spinning at the given power
     * and it will stop itself once object is picked up or dumped in the intake at which time the given event will be
     * signaled or it will notify the caller's handler.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to dump.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public void autoAssist(
        String owner, double delay, double power, TrcEvent event, double timeout)
    {
        if (sensorTrigger == null || power == 0.0)
        {
            throw new RuntimeException("Must have sensor and non-zero power to perform AutoAssist.");
        }
        //
        // This is an auto-assist operation, make sure the caller has ownership.
        //
        if (validateOwnership(owner))
        {
            actionParams = new ActionParams(power, event, timeout);
            if (delay > 0.0)
            {
                timerEvent.setCallback(this::performAutoAssist, actionParams);
                timer.set(delay, timerEvent);
            }
            else
            {
                performAutoAssist(actionParams);
            }
        }
    }   //autoAssist

    /**
     * This method is an auto-assist operation. It allows the caller to start the intake spinning at the given power
     * and it will stop itself once object is picked up or dumped in the intake at which time the given event will be
     * signaled or it will notify the caller's handler.
     *
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to dump.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public void autoAssist(double delay, double power, TrcEvent event, double timeout)
    {
        autoAssist(null, delay, power, event, timeout);
    }   //autoAssist

    /**
     * This method is an auto-assist operation. It allows the caller to start the intake spinning at the given power
     * and it will stop itself once object is picked up or dumped in the intake at which time the given event will be
     * signaled or it will notify the caller's handler.
     *
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to dump.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public void autoAssist(double power, TrcEvent event, double timeout)
    {
        autoAssist(null, 0.0, power, event, timeout);
    }   //autoAssist

    /**
     * This method is an auto-assist operation. It allows the caller to start the intake spinning at the given power
     * and it will stop itself once object is picked up or dumped in the intake at which time the given event will be
     * signaled or it will notify the caller's handler.
     *
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to dump.
     */
    public void autoAssist(double delay, double power)
    {
        autoAssist(null, delay, power, null, 0.0);
    }   //autoAssist

    /**
     * This method is an auto-assist operation. It allows the caller to start the intake spinning at the given power
     * and it will stop itself once object is picked up or dumped in the intake at which time the given event will be
     * signaled or it will notify the caller's handler.
     *
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to dump.
     */
    public void autoAssist(double power)
    {
        autoAssist(null, 0.0, power, null, 0.0);
    }   //autoAssist

    /**
     * This method returns the sensor value read from the analog sensor.
     *
     * @return analog sensor value.
     */
    public double getSensorValue()
    {
        return sensorTrigger != null && params.analogThreshold != null? sensorTrigger.getValue(): 0.0;
    }   //getSensorValue

    /**
     * This method returns the sensor state read from the digital sensor.
     *
     * @return digital sensor state.
     */
    public boolean getSensorState()
    {
        return sensorTrigger != null && params.analogThreshold == null && sensorTrigger.getState();
    }   //getSensorState

    /**
     *
     * This method checks if object is detected in the intake.
     *
     * @return true if object is detected in the intake, false otherwise.
     */
    public boolean hasObject()
    {
        boolean gotObject = false;

        if (sensorTrigger != null)
        {
            if (params.analogThreshold != null)
            {
                gotObject = getSensorValue() > params.analogThreshold;
            }
            else
            {
                gotObject = getSensorState();
            }

            if (params.triggerInverted)
            {
                gotObject = !gotObject;
            }
        }

        return gotObject;
    }   //hasObject

    /**
     * This method checks if auto-assist pickup is active.
     *
     * @return true if auto-assist is in progress, false otherwise.
     */
    public boolean isAutoAssistActive()
    {
        return actionParams != null;
    }   //isAutoAssistActive

}   //class TrcIntake
