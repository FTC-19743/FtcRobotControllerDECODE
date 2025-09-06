package org.firstinspires.ftc.teamcode.assemblies;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import java.util.concurrent.TimeUnit;

public abstract class OpenCVProcessor implements VisionProcessor {

    Mat submat = new Mat();



    // Convert from OpenCV Rect to Android Graphics Rect.  Useful for writing on the Android Canvas in onDrawFrame
    public android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
        return new android.graphics.Rect(left, top, right, bottom);
    }

    // Compute the average saturation in a given rectangle on an HSV mat
    // Useful for finding something against the grey background of a FTC mat!
    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    protected double getAvgValue(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[2];
    }

    protected Scalar getAverages (Mat input, Rect rect) {
        submat = input.submat(rect);
        return(Core.mean(submat));
    }

    public void configureCam (VisionPortal portal, boolean autoExposure, boolean aePriority, long exposure, int gain, boolean autoWB, int wb, boolean aFocus, int focalLength) {
        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
        GainControl gainControl = portal.getCameraControl(GainControl.class);
        WhiteBalanceControl wbControl = portal.getCameraControl(WhiteBalanceControl.class);
        FocusControl focusControl = portal.getCameraControl(FocusControl.class);

        if (autoExposure) {
            if (exposureControl.setMode(ExposureControl.Mode.AperturePriority)) {
                teamUtil.log("Set WebCam to Auto Exposure");
            } else {
                teamUtil.log("FAILED to set webcam to auto exposure");
            }
        } else {
            if (exposureControl.setMode(ExposureControl.Mode.Manual)) {
                teamUtil.log("Set WebCam to Manual Exposure");
            } else {
                teamUtil.log("FAILED to set webcam to Manual exposure");
            }
            if (exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS)) {
                teamUtil.log("Set WebCam Exposure to "+ exposure);
                teamUtil.log("WebCam Exposure now "+ exposureControl.getExposure(TimeUnit.MILLISECONDS));

            } else {
                teamUtil.log("FAILED to set WebCam Exposure");
            }
            if (gainControl.setGain(gain)) {
                teamUtil.log("Set WebCam Gain to "+ gain);
            } else {
                teamUtil.log("FAILED to set WebCam Gain");
            }
        }

        if (exposureControl.setAePriority(aePriority)) {
            teamUtil.log("Set WebCam aePriority to "+ aePriority);
        } else {
            teamUtil.log("FAILED to set WebCam aePriority");
        }

        if (autoWB) {
            if (wbControl.setMode(WhiteBalanceControl.Mode.AUTO)) {
                teamUtil.log("Set WebCam to Auto White Balance");
            } else {
                teamUtil.log("FAILED to set webcam to auto White Balance");
            }
        } else {
            if (wbControl.setMode(WhiteBalanceControl.Mode.MANUAL)) {
                teamUtil.log("Set WebCam to Manual White Balance");
            } else {
                teamUtil.log("FAILED to set webcam to Manual White Balance");
            }
            if (wbControl.setWhiteBalanceTemperature(wb)) {
                teamUtil.log("Set WebCam WB to "+ wb);
            } else {
                teamUtil.log("FAILED to set WebCam WB");
            }
        }
        if (aFocus) {
            if (focusControl.setMode(FocusControl.Mode.ContinuousAuto)) {
                teamUtil.log("Set WebCam to Continuous Auto Focus");
            } else {
                teamUtil.log("FAILED to set webcam to Continuous Auto Focus");
            }
        } else {
            if (focusControl.setMode(FocusControl.Mode.Fixed)) {
                teamUtil.log("Set WebCam to Fixed Focus");
            } else {
                teamUtil.log("FAILED to set webcam to Fixed Focus");
            }
            if (focusControl.setFocusLength(focalLength)) {
                teamUtil.log("Set WebCam focal length to "+ focalLength);
            } else {
                teamUtil.log("FAILED to set WebCam focal length");
            }
        }
    }


}
