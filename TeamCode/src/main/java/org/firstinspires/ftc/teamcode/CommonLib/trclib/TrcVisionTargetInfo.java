/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 * Based on sample code by Robert Atkinson.
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

import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.Locale;

/**
 * This class calculates and stores the info for a vision detected target.
 */
public class TrcVisionTargetInfo<O extends TrcVisionTargetInfo.ObjectInfo>
{
    /**
     * This interface implements a method to get the rectangle of the detected object. This should be implemented by
     * a vision detector class.
     */
    public interface ObjectInfo
    {
        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        Rect getRect();

        /**
         * This method returns the area of the detected object.
         *
         * @return area of the detected object.
         */
        double getArea();

    }   //interface ObjectInfo

    public O detectedObj;
    public int imageWidth;
    public int imageHeight;
    public Rect rect;
    public double area;
    public Point distanceFromImageCenter;
    public Point distanceFromCamera;
    public double targetWidth;
    public double horizontalAngle;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param detectedObj specifies the detected object.
     * @param imageWidth specifies the width of the camera image.
     * @param imageHeight specifies the height of the camera image.
     * @param homographyMapper specifies the homography mapper, can be null if not provided in which case
     *        distanceFromCamera, targetWidth and horizontalAngle will not be determined.
     * @param objHeightOffset specifies the object height offset above the floor.
     * @param cameraHeight specifies the height of the camera above the floor.
     */
    public TrcVisionTargetInfo(
        O detectedObj, int imageWidth, int imageHeight, TrcHomographyMapper homographyMapper,
        double objHeightOffset, double cameraHeight)
    {
        this.detectedObj = detectedObj;
        this.imageWidth = imageWidth;
        this.imageHeight = imageHeight;
        this.rect = detectedObj.getRect();
        this.area = detectedObj.getArea();
        distanceFromImageCenter = new Point(
            rect.x + rect.width/2.0 - imageWidth/2.0, rect.y + rect.height/2.0 - imageHeight/2.0);

        if (homographyMapper != null)
        {
            Point bottomLeft = homographyMapper.mapPoint(new Point(rect.x, rect.y + rect.height));
            Point bottomRight = homographyMapper.mapPoint(new Point(rect.x + rect.width, rect.y + rect.height));
            distanceFromCamera = new Point((bottomLeft.x + bottomRight.x)/2.0, (bottomLeft.y + bottomRight.y)/2.0);
            targetWidth = bottomRight.x - bottomLeft.x;
            double horiAngleRadian = Math.atan2(distanceFromCamera.x, distanceFromCamera.y);
            horizontalAngle = Math.toDegrees(horiAngleRadian);
            if (objHeightOffset > 0.0)
            {
                // If object is elevated off the ground, the object distance would be further than it actually is.
                // Therefore, we need to calculate the distance adjustment to be subtracted from the Homography
                // reported distance. Imagine the camera is the sun casting a shadow on the object to the ground.
                // The shadow length is the distance adjustment.
                //
                //  cameraHeight / homographyDistance = objHeightOffset / adjustment
                //  adjustment = objHeightOffset * homographyDistance / cameraHeight
                double adjustment =
                    objHeightOffset * TrcUtil.magnitude(distanceFromCamera.x, distanceFromCamera.y) / cameraHeight;
                distanceFromCamera.x -= adjustment * Math.sin(horiAngleRadian);
                distanceFromCamera.y -= adjustment * Math.cos(horiAngleRadian);
            }
        }
    }   //TrcVisionTargetInfo

    /**
     * This method returns the string form of the target info.
     *
     * @return string form of the target info.
     */
    @Override
    public String toString()
    {
        String s;

        if (distanceFromCamera != null)
        {
            s = String.format(
                Locale.US,
                "Obj=%s image(w=%d,h=%d) distImageCenter=%s distCamera=%s targetWidth=%.1f horiAngle=%.1f",
                detectedObj, imageWidth, imageHeight, distanceFromImageCenter, distanceFromCamera, targetWidth,
                horizontalAngle);
        }
        else
        {
            s = String.format(
                Locale.US,
                "Obj=%s image(w=%d,h=%d) distImageCenter=%s",
                detectedObj, imageWidth, imageHeight, distanceFromImageCenter);
        }

        return s;
    }   //toString

}   //class TrcVisionTargetInfo
