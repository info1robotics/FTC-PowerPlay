/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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
 * This class implements a platform independent conveyor subsystem. It contains a motor and optionally an entrance
 * and an exit sensor that detects if the an object has entered or exited the conveyor. It also supports exclusive
 * subsystem access by implementing TrcExclusiveSubsystem. This enables the conveyor subsystem to be aware of multiple
 * callers' access to the subsystem. While one caller starts the conveyor for an operation, nobody can access it until
 * the previous caller is done with the operation.
 */
public class TrcPidConveyor extends TrcPidMotor
{
    /**
     * This class contains all the parameters related to the conveyor.
     */
    public static class Parameters
    {
        public TrcPidController.PidParameters pidParams;
        public double scale = 1.0;
        public double objectDistance = 0.0;
        public double movePower = 1.0;
        public int maxCapacity = 1;

        /**
         * This method sets the PID parameters of the PID controller used for PID controlling the motor actuator.
         *
         * @param pidParams specifies the PID parameters.
         * @return this parameter object.
         */
        public Parameters setPidParams(TrcPidController.PidParameters pidParams)
        {
            this.pidParams = pidParams;
            return this;
        }   //setPidParams

        /**
         * This method sets the PID parameters of the PID controller used for PID controlling the motor actuator.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         * @param tolerance specifies the tolerance.
         * @return this parameter object.
         */
        public Parameters setPidParams(double kP, double kI, double kD, double tolerance)
        {
            this.pidParams = new TrcPidController.PidParameters(kP, kI, kD, tolerance);
            return this;
        }   //setPidParams

        /**
         * This method sets the scale of the conveyor distance sensor. It allows the conveyor to report real world
         * position units such as inches instead of sensor units.
         *
         * @param scale specifies the scale multiplier to convert position sensor unit to real world unit.
         * @return this parameter object.
         */
        public Parameters setScale(double scale)
        {
            this.scale = scale;
            return this;
        }   //setScale

        /**
         * This method sets the distance between objects inside the converyor.
         *
         * @param distance specifies the distance between objects in the conveyor.
         * @return this parameter object.
         */
        public Parameters setObjectDistance(double distance)
        {
            this.objectDistance = distance;
            return this;
        }   //setObjectDistance

        /**
         * This method sets the conveyor move power.
         *
         * @param power specifies the motor power to move the object in the conveyor.
         * @return this parameter object.
         */
        public Parameters setMovePower(double power)
        {
            this.movePower = power;
            return this;
        }   //setMovePower

        /**
         * This method sets the maximum number of objects the conveyor can hold.
         *
         * @param capacity specifies maximum number of objects the conveyor can hold.
         * @return this parameter object.
         */
        public Parameters setMaxCapacity(int capacity)
        {
            this.maxCapacity = capacity;
            return this;
        }   //setMaxCapacity

    }   //class Parameters

    private final TrcDigitalInput entranceSensor;
    private final TrcDigitalInput exitSensor;
    private final Parameters params;
    private final TrcDigitalInputTrigger entranceTrigger;
    private final TrcDigitalInputTrigger exitTrigger;
    private TrcNotifier.Receiver entranceEventHandler;
    private TrcNotifier.Receiver exitEventHandler;
    private int numObjects = 0;
    private TrcEvent onFinishEvent;
    private TrcNotifier.Receiver onFinishCallback;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param motor specifies the motor object.
     * @param entranceSensor specifies the sensor object for the conveyor entrance.
     * @param exitSensor specifies the sensor object for the conveyor exit.
     * @param params specifies the parameters object.
     */
    public TrcPidConveyor(
        String instanceName, TrcMotor motor, TrcDigitalInput entranceSensor, TrcDigitalInput exitSensor,
        Parameters params)
    {
        super(instanceName, motor, params.pidParams, null, 0.0);
        this.entranceSensor = entranceSensor;
        this.exitSensor = exitSensor;
        this.params = params;

        setPositionScale(params.scale);

        if (entranceSensor != null)
        {
            entranceTrigger = new TrcDigitalInputTrigger(
                instanceName + ".entranceTrigger", entranceSensor, this::entranceEvent);
            entranceTrigger.setEnabled(true);
        }
        else
        {
            entranceTrigger = null;
        }

        if (exitSensor != null)
        {
            exitTrigger = new TrcDigitalInputTrigger(
                instanceName + ".exitTrigger", entranceSensor, this::exitEvent);
            exitTrigger.setEnabled(true);
        }
        else
        {
            exitTrigger = null;
        }
    }   //TrcPidConveyor

    /**
     * This method returns the number of objects in the conveyor.
     * Note: Conveyor can only keep track of the number of objects if it has entrance and exit sensors. Otherwise,
     * the number of objects returned will not be correct.
     *
     * @return number of objects in the conveyor.
     */
    public int getNumObjects()
    {
        return numObjects;
    }   //getNumObjects

    /**
     * This method sets the number of preloaded objects in the conveyor.
     *
     * @param num specifies the number of preloaded objects in the conveyor.
     */
    public void setPreloadedObjects(int num)
    {
        this.numObjects = num;
    }   //setPreloadedObjects

    /**
     * This method registers an event handler to be notified when the entrance sensor is triggered.
     *
     * @param handler event handler to be notified.
     */
    public void registerEntranceEventHandler(TrcNotifier.Receiver handler)
    {
        if (entranceTrigger != null)
        {
            entranceEventHandler = handler;
        }
    }   //registerEntranceEventHandler

    /**
     * This method registers an event handler to be notified when the exit sensor is triggered.
     *
     * @param handler event handler to be notified.
     */
    public void registerExitEventHandler(TrcNotifier.Receiver handler)
    {
        if (exitTrigger != null)
        {
            exitEventHandler = handler;
        }
    }   //registerExitEventHandler

    /**
     * This method returns the sensor state read from the digital sensor.
     *
     * @return digital sensor state.
     */
    public boolean isEntranceSensorActive()
    {
        return entranceSensor != null && entranceSensor.isActive();
    }   //isEntranceSensorActive

    /**
     * This method returns the sensor state read from the digital sensor.
     *
     * @return digital sensor state.
     */
    public boolean isExitSensorActive()
    {
        return exitSensor != null && exitSensor.isActive();
    }   //isExitSensorActive

    /**
     * This method advances or backs up the conveyor by the number of object units.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the conveyor.
     * @param units specifies the number of object units to advance or negative number to back up.
     * @param event specifies the event to signal when done, can be null if not provided.
     * @param callback specifies the callback handler to notify when done, can be null if not provided.
     */
    public void move(String owner, int units, TrcEvent event, TrcNotifier.Receiver callback)
    {
        if (validateOwnership(owner))
        {
            if (event != null)
            {
                event.clear();
            }
            this.onFinishEvent = event;
            this.onFinishCallback = callback;
            setTarget(units*params.objectDistance, false, 1.0, null, this::reachedTarget);
        }
    }   //move

    /**
     * This method advances or backs up the conveyor by the number of object units.
     *
     * @param units specifies the number of object units to advance or negative number to back up.
     * @param event specifies the event to signal when done, can be null if not provided.
     * @param callback specifies the callback handler to notify when done, can be null if not provided.
     */
    public void move(int units, TrcEvent event, TrcNotifier.Receiver callback)
    {
        move(null, units, event, callback);
    }   //move

    /**
     * This method advances or backs up the conveyor by the number of object units.
     *
     * @param units specifies the number of object units to advance or negative number to back up.
     */
    public void move(int units)
    {
        move(null, units, null, null);
    }   //move

    /**
     * This method advances the conveyor by the one object unit.
     */
    public void advance()
    {
        move(null, 1, null, null);
    }   //advance

    /**
     * This method backs up the conveyor by the one object unit.
     */
    public void backup()
    {
        move(null, -1, null, null);
    }   //backup

    /**
     * This method is called when the entrance sensor is triggered.
     *
     * @param active specifies true if an object has activated the sensor, false if the object has deactivated it.
     */
    private void entranceEvent(boolean active)
    {
        if (active)
        {
            numObjects++;
            setPower(params.movePower);
        }
        else
        {
            setPower(0.0);
        }

        if (entranceEventHandler != null)
        {
            entranceEventHandler.notify(active);
        }
    }   //entranceEvent

    /**
     * This method is called when the exit sensor is triggered.
     *
     * @param active specifies true if an object has activated the sensor, false if the object has deactivated it.
     */
    private void exitEvent(boolean active)
    {
        if (active)
        {
            numObjects--;
        }
        else
        {
            setPower(0.0);
        }

        if (exitEventHandler != null)
        {
            exitEventHandler.notify(active);
        }
    }   //exitEvent

    /**
     * This method is called when a previous call to move the conveyor has reached its target.
     *
     * @param context specifies the callback context (not used).
     */
    private void reachedTarget(Object context)
    {
        if (onFinishEvent != null)
        {
            onFinishEvent.signal();
            onFinishEvent = null;
        }

        if (onFinishCallback != null)
        {
            onFinishCallback.notify(null);
            onFinishCallback = null;
        }
    }   //reachedTarget

}   //class TrcPidConveyor
