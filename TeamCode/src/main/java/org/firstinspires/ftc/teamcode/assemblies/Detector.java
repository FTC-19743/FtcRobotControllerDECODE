package org.firstinspires.ftc.teamcode.assemblies;

import static androidx.core.math.MathUtils.clamp;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.FtcDashboard;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.libs.teamUtil;

import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.atomic.AtomicBoolean;
import com.acmerobotics.dashboard.config.Config;

@Config

public class Detector {
    Telemetry telemetry;
    HardwareMap hardwareMap;

    static public boolean details = false;




    public OpenCVSampleDetector sampleDetector = new OpenCVSampleDetector();
    final int ARDU_RESOLUTION_WIDTH = 640;
    final int ARDU_RESOLUTION_HEIGHT = 480;
    Size arduSize = new Size(ARDU_RESOLUTION_WIDTH, ARDU_RESOLUTION_HEIGHT);


    public AtomicBoolean processorOn = new AtomicBoolean(false);


    public VisionPortal portal;


    public Detector() {
        teamUtil.log("Constructing Intake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
        //blinkin = new Blinkin(hardwareMap,telemetry);
    }



    public void initCV(boolean enableLiveView){
        teamUtil.log("Initializing CV in Intake");
        CameraName testCam = (CameraName)hardwareMap.get(WebcamName.class, "Webcam 1"); //   logitechhd
        CameraCharacteristics chars = testCam.getCameraCharacteristics();

        VisionPortal.Builder armBuilder = new VisionPortal.Builder();
        armBuilder.setCamera(testCam);
        armBuilder.enableLiveView(enableLiveView);

        // Can also set resolution and stream format if we want to optimize resource usage.
        armBuilder.setCameraResolution(arduSize);
        //armBuilder.setStreamFormat(TBD);

        armBuilder.addProcessor(sampleDetector);
        portal = armBuilder.build();
        sampleDetector.setVisionPortal(portal);
        sampleDetector.viewingPipeline = enableLiveView;

        // Wait for the camera to be open
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!teamUtil.theOpMode.isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                teamUtil.pause(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        sampleDetector.configureCam(portal, true, OpenCVSampleDetector.AEPRIORITY, 1, OpenCVSampleDetector.GAIN, OpenCVSampleDetector.WHITEBALANCEAUTO, OpenCVSampleDetector.TEMPERATURE, OpenCVSampleDetector.AFOCUS, OpenCVSampleDetector.FOCUSLENGTH);
        teamUtil.pause(2000);
        sampleDetector.configureCam(portal, OpenCVSampleDetector.APEXPOSURE, OpenCVSampleDetector.AEPRIORITY, OpenCVSampleDetector.EXPOSURE, OpenCVSampleDetector.GAIN, OpenCVSampleDetector.WHITEBALANCEAUTO, OpenCVSampleDetector.TEMPERATURE, OpenCVSampleDetector.AFOCUS, OpenCVSampleDetector.FOCUSLENGTH);
        stopCVPipeline();
        teamUtil.log("Initializing CV in Intake - Finished");
    }

    public void closeCV () {
        portal.close();
    }

    // Calibrate slider and extender.


    public void setTargetColor(OpenCVSampleDetector.TargetColor targetColor){
        sampleDetector.setTargetColor(targetColor);
    }

    public boolean currentlyStreaming() {
        return (portal.getCameraState() == VisionPortal.CameraState.STREAMING);
    }

    public void startStreaming(){
        // TODO
    }

    public void stopStreaming () {
        // TODO
    }
    public void startCVPipeline () {
        sampleDetector.reset();
        portal.setProcessorEnabled(sampleDetector, true );
        processorOn.set(true);
        teamUtil.log("ProcessorOn = " + processorOn.get());
    }

    public void stopCVPipeline () {
        portal.setProcessorEnabled(sampleDetector, false );
        processorOn.set(false);
        teamUtil.log("ProcessorOn = " + processorOn.get());
    }

    public void restartCVPipeline(){
        stopCVPipeline();
        teamUtil.pause(100);
        startCVPipeline();
    }

}