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

import org.opencv.core.Mat;

/**
 * This interface implements the standard methods for an OpenCV pipeline. It also provides default methods to log
 * pipeline performance metrics.
 *
 * @param <O> specifies the detected object type the pipeline will produce.
 */
public interface TrcOpenCvPipeline<O>
{
    /**
     * This class encapsulates the pipeline performance metrics.
     */
    class PerformanceMetrics
    {
        double startTime = 0.0;
        double totalProcessedTime = 0.0;
        long totalProcessedFrames = 0;

        /**
         * This method resets the pipeline performance metrics. It is typically called before enabling the pipeline.
         */
        public void reset()
        {
            startTime = TrcUtil.getCurrentTime();
            totalProcessedTime = 0.0;
            totalProcessedFrames = 0;
        }   //reset

        /**
         * This method is called to log the processing time of the pipeline.
         *
         * @param startTime specifies the timestamp when the processing starts.
         */
        public void logProcessingTime(double startTime)
        {
            totalProcessedTime += TrcUtil.getCurrentTime() - startTime;
            totalProcessedFrames++;
        }   //logProcessingTime

        /**
         * This method prints the pipeline performance metrics using the given tracer.
         */
        public void printMetrics(TrcDbgTrace tracer)
        {
            final String funcName = "printMetrics";

            if (tracer != null)
            {
                tracer.traceInfo(
                    funcName, "AvgProcessTime=%.3f sec, FrameRate=%.1f",
                    totalProcessedTime/totalProcessedFrames,
                    totalProcessedFrames/(TrcUtil.getCurrentTime() - startTime));
            }
        }   //printMetrics

    }   //class PerformanceMetrics

    PerformanceMetrics performanceMetrics = new PerformanceMetrics();

    /**
     * This method is called to reset the state of the pipeline if any.
     */
    default void reset()
    {
        performanceMetrics.reset();
    }   //reset

    /**
     * This method is called to process the input image through the pipeline.
     *
     * @param input specifies the input image to be processed.
     */
    void process(Mat input);

    /**
     * This method returns the array of detected objects.
     *
     * @return array of detected objects.
     */
    O[] getDetectedObjects();

}   //interface TrcOpenCvPipeline
