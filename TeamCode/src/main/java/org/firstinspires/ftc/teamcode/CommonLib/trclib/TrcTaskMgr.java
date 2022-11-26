/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * This class provides methods for the callers to register/unregister cooperative multi-tasking tasks. It manages
 * these tasks and will work with the cooperative multi-tasking scheduler to run these tasks.
 */
public class TrcTaskMgr
{
    private static final String moduleName = "TrcTaskMgr";
    private static final boolean debugEnabled = false;
    private static final TrcDbgTrace dbgTrace = TrcDbgTrace.getGlobalTracer();

    public static final long FAST_LOOP_INTERVAL_MS = 20;        // in msec (50 Hz)
    public static final long SLOW_LOOP_INTERVAL_MS = 50;        // in msec (20 Hz)
    public static final long INPUT_THREAD_INTERVAL_MS = 20;     // in msec
    public static final long OUTPUT_THREAD_INTERVAL_MS = 20;    // in msec
    public static final long TASKTIME_THRESHOLD_MS = FAST_LOOP_INTERVAL_MS;

    /**
     * These are the task type TrcTaskMgr supports:
     */
    public enum TaskType
    {
        /**
         * START_TASK is called one time before a competition mode is about to start on the main robot thread.
         */
        START_TASK(0),

        /**
         * STOP_TASK is called one time before a competition mode is about to end on the main robot thread.
         */
        STOP_TASK(1),

        /**
         * FAST_PREPERIODIC_TASK is called periodically at a fast rate about 50Hz before runPeriodic() on the main
         * robot thread.
         */
        FAST_PREPERIODIC_TASK(2),

        /**
         * FAST_POSTPERIODIC_TASK is called periodically at a fast rate about 50Hz after runPeriodic() on the main
         * robot thread.
         */
        FAST_POSTPERIODIC_TASK(3),

        /**
         * SLOW_PREPERIODIC_TASK is called periodically at a slow rate about 20Hz before runPeriodic() on the main
         * robot thread.
         */
        SLOW_PREPERIODIC_TASK(4),

        /**
         * SLOW_POSTPERIODIC_TASK is called periodically at a slow rate about 20Hz after runPeriodic() on the main
         * robot thread.
         */
        SLOW_POSTPERIODIC_TASK(5),

        /**
         * INPUT_TASK is called periodically at a rate about 20Hz on its own thread. Typically, it runs code that
         * reads sensor input.
         */
        INPUT_TASK(6),

        /**
         * OUTPUT_TASK is called periodically at a rate about 100Hz on its own thread. Typically, it runs code that
         * updates the state of actuators.
         */
        OUTPUT_TASK(7),

        /**
         * STANDALONE_TASK is called periodically at the specified interval on its own thread. Typically, code that
         * may block for a long time requires its own thread so that it doesn't degrade the performance of the other
         * threads.
         */
        STANDALONE_TASK(8);

        public final int value;

        TaskType(int value)
        {
            this.value = value;
        }   //TaskType

    }   //enum TaskType

    /**
     * Any class that is registering a task must implement this interface.
     */
    public interface Task
    {
        /**
         * This method is called at the appropriate time this task is registered for.
         *
         * StartTask:
         *  This contains code that initializes the task before a competition mode is about to start and is run on
         *  the main robot thread. Typically, if the task is a robot subsystem, you may put last minute mode specific
         *  initialization code here. Most of the time, you don't need to register StartTask because all initialization
         *  is done in initRobot(). But sometimes, you may want to delay a certain initialization until right before
         *  competition starts. For example, you may want to reset the gyro heading right before competition starts to
         *  prevent drifting.
         *
         * StopTask:
         *  This contains code that cleans up the task before a competition mode is about to end and is run on the main
         *  robot thread. Typically, if the task is a robot subsystem, you may put code to stop the robot here. Most of
         *  the time, you don't need to register StopTask because the system will cut power to all the motors after a
         *  competition mode has ended.
         *
         * FastPrePeriodicTask:
         *  This contains code that runs periodically at a fast rate on the main robot thread before fastPeriodic().
         *  Typically, you will put code that deals with any high resolution input or sensor readings here that
         *  fastPeriodic() may depend on.
         *
         * FastPostPeriodicTask:
         *  This contains code that runs periodically at a fast rate on the main robot thread after fastPeriodic().
         *  Typically, you will put code that deals with actions that requires high frequency processing.

         * SlowPrePeriodicTask:
         *  This contains code that runs periodically at a slow rate on the main robot thread before slowPeriodic().
         *  Typically, you will put code that deals with slower input or sensor readings here that slowPeriodic may
         *  depend on.
         *
         * SlowPostPeriodicTask:
         *  This contains code that runs periodically at a slow rate on the main robot thread after slowPeriodic().
         *  Typically, you will put code that deals with slower action here that may depend on the result produced
         *  by slowPeriodic().
         *
         * InputTask:
         *  This contains code that runs periodically on the input thread. Typically, you will put code that deals
         *  with any input or sensor readings that may otherwise degrade the performance of the main robot thread.
         *
         * OutputTask:
         *  This contains code that runs periodically on the output thread. Typically, you will put code that deals
         *  with actions that may otherwise degrade the performance of the main robot thread.
         *
         * StandaloneTask:
         *  This contains code that will run on its own thread at the specified task interval. Typically, you will
         *  put code that may take a long time to execute and could affect the performance of shared threads such as
         *  the main robot thread, input or output threads.
         *
         * @param taskType specifies the type of task being run. This may be useful for handling multiple task types.
         * @param runMode specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
         */
        void runTask(TaskType taskType, TrcRobot.RunMode runMode);

    }   //interface Task

    /**
     * This class implements TaskObject that will be created whenever a class is registered as a cooperative
     * multi-tasking task. The created task objects will be entered into an array list of task objects to be
     * scheduled by the scheduler.
     */
    public static class TaskObject
    {
        private final String taskName;
        private final Task task;
        private final HashSet<TaskType> taskTypes;
        private final long[] taskStartTimes = new long[TaskType.values().length];
        private final long[] taskTotalIntervals = new long[TaskType.values().length];
        private final long[] taskTotalElapsedTimes = new long[TaskType.values().length];
        private final int[] taskTimeSlotCounts = new int[TaskType.values().length];
        private TrcPeriodicThread<Object> taskThread = null;

        /**
         * Constructor: Creates an instance of the task object with the given name
         * and the given task type.
         *
         * @param taskName specifies the instance name of the task.
         * @param task specifies the object that implements the TrcTaskMgr.Task interface.
         */
        private TaskObject(final String taskName, Task task)
        {
            this.taskName = taskName;
            this.task = task;
            taskTypes = new HashSet<>();
            for (int i = 0; i < TaskType.values().length; i++)
            {
                taskStartTimes[i] = 0;
                taskTotalIntervals[i] = 0;
                taskTotalElapsedTimes[i] = 0;
                taskTimeSlotCounts[i] = 0;
            }
        }   //TaskObject

        /**
         * This method returns the instance name of the task.
         *
         * @return instance name of the class.
         */
        @Override
        public String toString()
        {
            return taskName;
        }   //toString

        /**
         * This method adds the given task type to the task object.
         *
         * @param type specifies the task type.
         * @param taskInterval specifies the periodic interval for STANDALONE_TASK, ignore for any other task types.
         *                     If zero interval is specified, the task will be run in a tight loop.
         * @param taskPriority specifies the priority of the associated thread. Only valid for STANDALONE_TASK,
         *                     ignored for any other task types.
         * @return true if successful, false if the task with that task type is already registered in the task list.
         */
        public synchronized boolean registerTask(TaskType type, long taskInterval, int taskPriority)
        {
            if (type == TaskType.STANDALONE_TASK && taskInterval < 0)
            {
                throw new IllegalArgumentException("taskInterval must be greater than or equal to 0.");
            }

            boolean added = taskTypes.add(type);

            if (added)
            {
                if (type == TaskType.STANDALONE_TASK)
                {
                    taskThread = new TrcPeriodicThread<>(taskName, this::standaloneTask, null, taskPriority);
                    taskThread.setProcessingInterval(taskInterval);
                    taskThread.setTaskEnabled(true);
                }
                else
                {
                    taskThread = null;
                    if (type == TaskType.INPUT_TASK)
                    {
                        //
                        // There is only one global input thread. All INPUT_TASKs run on this thread.
                        // The input thread is created on first registration, so create it if not already.
                        //
                        TrcTaskMgr.startInputThread();
                    }
                    else if (type == TaskType.OUTPUT_TASK)
                    {
                        //
                        // There is only one global output thread. All OUTPUT_TASKs run on this thread.
                        // The output thread is created on first registration, so create it if not already.
                        //
                        TrcTaskMgr.startOutputThread();
                    }
                }
            }

            return added;
        }   //registerTask

        /**
         * This method adds the given task type to the task object.
         *
         * @param type specifies the task type.
         * @param taskInterval specifies the periodic interval for STANDALONE_TASK, ignore for any other task types.
         *                     If zero interval is specified, the task will be run in a tight loop.
         * @return true if successful, false if the task with that task type is already registered in the task list.
         */
        public boolean registerTask(TaskType type, long taskInterval)
        {
            return registerTask(type, taskInterval, Thread.NORM_PRIORITY);
        }   //registerTask

        /**
         * This method adds the given task type to the task object.
         *
         * @param type specifies the task type.
         * @return true if successful, false if the task with that task type is already registered in the task list.
         */
        public boolean registerTask(TaskType type)
        {
            return registerTask(type, 0, Thread.NORM_PRIORITY);
        }   //registerTask

        /**
         * This method removes the given task type from the task object.
         *
         * @param type specifies the task type.
         * @return true if successful, false if the task with that type is not found the task list.
         */
        public synchronized boolean unregisterTask(TaskType type)
        {
            if (type == TaskType.STANDALONE_TASK && taskThread != null)
            {
                taskThread.terminateTask();
            }
            taskThread = null;

            return taskTypes.remove(type);
        }   //unregisterTask

        /**
         * This method unregisters the given task object from all task types.
         *
         * @return true if successfully removed from any task type, false otherwise.
         */
        public synchronized boolean unregisterTask()
        {
            boolean removed = false;

            for (TaskType taskType : TaskType.values())
            {
                if (unregisterTask(taskType))
                {
                    removed = true;
                }
            }
            return removed;
        }   //unregisterTask

        /**
         * This method checks if the given task type is registered with this task object.
         *
         * @param type specifies the task type to be checked against.
         * @return true if this task is registered with the given type, false otherwise.
         */
        public synchronized boolean isRegistered(TaskType type)
        {
            return hasType(type);
        }   //isRegistered

        /**
         * This method checks if this task object is registered for any task type.
         *
         * @return true if this task is registered with any type, false otherwise.
         */
        public synchronized boolean isRegistered()
        {
            boolean registered = false;

            for (TaskType taskType : TaskType.values())
            {
                if (isRegistered(taskType))
                {
                    registered = true;
                    break;
                }
            }

            return registered;
        }   //isRegistered

        /**
         * This method checks if the given task type is registered with this task object.
         *
         * @param type specifies the task type to be checked against.
         * @return true if this task is registered as the given type, false otherwise.
         */
        private synchronized boolean hasType(TaskType type)
        {
            return taskTypes.contains(type);
        }   //hasType

        /**
         * This method returns the class object that was associated with this task object.
         *
         * @return class object associated with the task.
         */
        private Task getTask()
        {
            return task;
        }   //getTask

        /**
         * This method returns the task interval for TaskType.STANDALONE_TASK.
         *
         * @return task interval in msec. If there is no STANDALONE_TASK type in the task object, zero is returned.
         */
        public synchronized long getTaskInterval()
        {
            return taskThread != null? taskThread.getProcessingInterval(): 0;
        }   //getTaskInterval

        /**
         * This method sets the task interval for TaskType.STANDALONE_TASK. It has no effect for any other types.
         *
         * @param taskInterval specifies the periodic interval for STANDALONE_TASK, ignore for any other task types.
         *                     If zero interval is specified, the task will be run in a tight loop.
         */
        public synchronized void setTaskInterval(long taskInterval)
        {
            if (taskThread != null)
            {
                taskThread.setProcessingInterval(taskInterval);
            }
        }   //setTaskInterval

        /**
         * This method sets the task data for TaskType.STANDALONE_TASK. It has no effect for any other types.
         *
         * @param data specifies the thread data for STANDALONE_TASK, ignore for any other task types.
         */
        public synchronized void setTaskData(Object data)
        {
            if (taskThread != null)
            {
                taskThread.setData(data);
            }
        }   //setTaskData

        /**
         * This method runs the periodic standalone task.
         *
         * @param context specifies the context (not used).
         */
        private void standaloneTask(Object context)
        {
            recordStartTime(TaskType.STANDALONE_TASK);
            task.runTask(TaskType.STANDALONE_TASK, TrcRobot.getRunMode());
            recordElapsedTime(TaskType.STANDALONE_TASK);
        }   //standaloneTask

        /**
         * This method records the task start timestamp in the task performance arrays. It is used to calculate
         * task elapsed time after the execution of a task.
         *
         * @param taskType specifies the task type to index into the task performance arrays.
         */
        private synchronized void recordStartTime(TaskType taskType)
        {
            long currNanoTime = TrcUtil.getNanoTime();
            long taskInterval = taskStartTimes[taskType.value] > 0 ? currNanoTime - taskStartTimes[taskType.value] : 0;

            taskStartTimes[taskType.value] = currNanoTime;
            taskTotalIntervals[taskType.value] += taskInterval;
        }   //recordStartTime

        /**
         * This method records the task elapsed time in the task performance arrays.
         *
         * @param taskType specifies the task type to index into the task performance arrays.
         */
        private synchronized void recordElapsedTime(TaskType taskType)
        {
            final String funcName = "recordElapsedTime";
            long currNanoTime = TrcUtil.getNanoTime();
            long startTime = taskStartTimes[taskType.value];
            long elapsedTime = currNanoTime - startTime;

            taskTotalElapsedTimes[taskType.value] += elapsedTime;
            taskTimeSlotCounts[taskType.value]++;

            if (debugEnabled)
            {
                dbgTrace.traceVerbose(
                    funcName, "Task %s.%s: start=%.6f, elapsed=%.6f",
                    taskName, taskType, startTime/1000000000.0, elapsedTime/1000000000.0);
                long timeThreshold = getTaskInterval()*1000000; //convert to nanoseconds.
                if (timeThreshold == 0) timeThreshold = TASKTIME_THRESHOLD_MS * 1000000L;
                if (elapsedTime > timeThreshold)
                {
                    dbgTrace.traceWarn(funcName, "%s.%s takes too long (%.3f)",
                            taskName, taskType, elapsedTime/1000000000.0);
                }
            }
        }   //recordElapsedTime

        /**
         * This method returns the average task elapsed time in seconds.
         *
         * @param taskType specifies the task type to index into the task performance arrays.
         * @return average task elapsed time in seconds.
         */
        private synchronized double getAverageTaskElapsedTime(TaskType taskType)
        {
            int slotCount = taskTimeSlotCounts[taskType.value];

            return slotCount == 0 ? 0.0 : (double)taskTotalElapsedTimes[taskType.value]/slotCount/1000000000.0;
        }   //getAverageTaskElapsedTime

        /**
         * This method returns the average task interval time in seconds.
         *
         * @param taskType specifies the task type to index into the task performance arrays.
         * @return average task interval time in seconds.
         */
        private synchronized double getAverageTaskInterval(TaskType taskType)
        {
            int slotCount = taskTimeSlotCounts[taskType.value];

            return slotCount == 0 ? 0.0 : (double)taskTotalIntervals[taskType.value]/slotCount/1000000000.0;
        }   //getAverageTaskInterval

    }   //class TaskObject

    private static final List<TaskObject> taskList = new CopyOnWriteArrayList<>();
    private static TrcPeriodicThread<Object> inputThread = null;
    private static TrcPeriodicThread<Object> outputThread = null;

    /**
     * This method is called by registerTask for INPUT_TASK to create the input thread if not already.
     */
    private static synchronized void startInputThread()
    {
        if (inputThread == null)
        {
            inputThread = startThread(
                moduleName + ".inputThread", TrcTaskMgr::inputTask, INPUT_THREAD_INTERVAL_MS, Thread.MAX_PRIORITY);
        }
    }   //startInputThread

    /**
     * This method is called by registerTask for OUTPUT_TASK to create the output thread if not already.
     */
    private static synchronized void startOutputThread()
    {
        if (outputThread == null)
        {
            outputThread = startThread(
                moduleName + ".outputThread", TrcTaskMgr::outputTask, OUTPUT_THREAD_INTERVAL_MS, Thread.MAX_PRIORITY);
        }
    }   //startOutputThread

    /**
     * This method starts a periodic thread for processing tasks.
     *
     * @param instanceName specifies the instance name of the thread.
     * @param task specifies the task run by the thread.
     * @param interval specifies the processing interval of the task.
     * @param taskPriority specifies the thread priority.
     * @return the created thread.
     */
    private static TrcPeriodicThread<Object> startThread(
        String instanceName, TrcPeriodicThread.PeriodicTask task, long interval, int taskPriority)
    {
        TrcPeriodicThread<Object> thread = new TrcPeriodicThread<>(instanceName, task, null, taskPriority);
        thread.setProcessingInterval(interval);
        thread.setTaskEnabled(true);

        return thread;
    }   //startThread

    /**
     * This method creates a TRC task. If the TRC task is registered as a STANDALONE task, it is run on a separately
     * created thread. Otherwise, it is run on the main robot thread as a cooperative multi-tasking task.
     *
     * @param taskName specifies the task name.
     * @param task specifies the Task interface for this task.
     * @return created task object.
     */
    public static TaskObject createTask(final String taskName, Task task)
    {
        final String funcName = "createTask";
        TaskObject taskObj;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "taskName=%s", taskName);
        }

        taskObj = new TaskObject(taskName, task);
        taskList.add(taskObj);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", taskObj);
        }

        return taskObj;
    }   //createTask

    /**
     * This method is mainly for FtcOpMode to call at the end of the opMode loop because runOpMode could be terminated
     * before shutdown can be called especially if stopMode code is doing logging I/O (e.g. printPerformanceMetrics).
     * This provides an earlier chance for us to stop all task threads before it's too late.
     */
    public static void terminateAllThreads()
    {
        TrcTimerMgr.shutdown();

        for (TaskObject taskObj: taskList)
        {
            if (taskObj.hasType(TaskType.STANDALONE_TASK))
            {
                //
                // Task contains the type STANDALONE_TASK, unregister it so that the task thread will terminate.
                //
                taskObj.unregisterTask(TaskType.STANDALONE_TASK);
            }
        }

        if (inputThread != null)
        {
            inputThread.terminateTask();
            inputThread = null;
        }

        if (outputThread != null)
        {
            outputThread.terminateTask();
            outputThread = null;
        }
    }   //terminateAllThreads

    /**
     * This method is called at the end of the robot program (FtcOpMode in FTC or FrcRobotBase in FRC) to terminate
     * all threads if any and remove all task from the task list.
     */
    public static void shutdown()
    {
        terminateAllThreads();
        TrcNotifier.shutdown();
        taskList.clear();
    }   //shutdown

    /**
     * This method is called by the main robot thread to enumerate the task list and calls all the tasks that matches
     * the given task type.
     *
     * @param type specifies the task type to be executed.
     * @param mode specifies the robot run mode.
     */
    public static void executeTaskType(TaskType type, TrcRobot.RunMode mode)
    {
        for (TaskObject taskObj: taskList)
        {
            if (taskObj.hasType(type))
            {
                Task task = taskObj.getTask();
                taskObj.recordStartTime(type);
                task.runTask(type, mode);
                taskObj.recordElapsedTime(type);
            }
        }
    }   //executeTaskType

    /**
     * This method runs the periodic input task.
     *
     * @param context specifies the context (not used).
     */
    private static void inputTask(Object context)
    {
        executeTaskType(TaskType.INPUT_TASK, TrcRobot.getRunMode());
    }   //inputTask

    /**
     * This method runs the periodic output task.
     *
     * @param context specifies the context (not used).
     */
    private static void outputTask(Object context)
    {
        executeTaskType(TaskType.OUTPUT_TASK, TrcRobot.getRunMode());
    }   //outputTask

    /**
     * This method prints the performance metrics of all tasks with the given tracer.
     *
     * @param tracer specifies the tracer to be used for printing the task performance metrics.
     */
    public static void printTaskPerformanceMetrics(TrcDbgTrace tracer)
    {
        for (TaskObject taskObj: taskList)
        {
            StringBuilder msg = new StringBuilder(taskObj.taskName + ":");
            int taskTypeCounter = 0;

            for (TaskType taskType : TaskType.values())
            {
                double taskElapsedTime = taskObj.getAverageTaskElapsedTime(taskType);
                double taskInterval = taskObj.getAverageTaskInterval(taskType);

                if (taskElapsedTime > 0.0)
                {
                    taskTypeCounter++;
                    msg.append(String.format(Locale.US, " %s=%.6f/%.6f", taskType, taskElapsedTime, taskInterval));
                }
            }

            if (taskTypeCounter > 0)
            {
                tracer.traceInfo("TaskPerformance", "%s", msg);
            }
        }
    }   //printTaskPerformanceMetrics

    /**
     * This method prints all registered tasks with the given tracer.
     *
     * @param tracer specifies the tracer to be used for printing the task performance metrics.
     */
    public static void printAllRegisteredTasks(TrcDbgTrace tracer)
    {
        for (TaskObject taskObj : taskList)
        {
            StringBuilder msg = new StringBuilder(taskObj.toString() + ":");

            for (TaskType type : taskObj.taskTypes)
            {
                msg.append(" ");
                msg.append(type);
            }

            tracer.traceInfo("RegisteredTask", "%s", msg);
        }
    }   //printAllRegisteredTask

}   //class TaskMgr
