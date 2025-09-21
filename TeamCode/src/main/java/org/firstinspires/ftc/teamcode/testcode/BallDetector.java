

package org.firstinspires.ftc.teamcode.testcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.MovingStatistics;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.assemblies.Detector;
import org.firstinspires.ftc.teamcode.assemblies.OctoQuadFWv3;
import org.firstinspires.ftc.teamcode.assemblies.OpenCVSampleDetector;
import org.firstinspires.ftc.teamcode.libs.LimeLightTool;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.Arrays;

//USE limelight test config for this opmode
@TeleOp
public class BallDetector extends LinearOpMode
{


    Limelight3A limelight;
    Detector detector;
    OpenCVSampleDetector.FrameData frameData = null;




    public void initCam() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }

    public String stringIdentifier(int index){
        if(index==0){
            return ("Center_X: ");
        }else if(index==1){
            return ("Center_Y: ");
        }else if(index==2){
            return ("Contour Width: ");
        }else if(index==3){
            return ("Contour Height: ");
        }else if(index==4){
            return ("Contour Area: ");
        }else if(index==6){
            return ("Center Hue: ");
        }else if(index==7){
            return ("Center Saturation: ");
        }else{
            return ("Center Value: ");
        }
    }


    //Runs Limelight and logitech simultaneously


    public void runOpMode()
    {

        teamUtil.init(this);

        //limelight init
        initCam();

        //logitech init
        detector = new Detector();
        detector.initCV(true);
        detector.startCVPipeline();

        telemetry.setMsTransmissionInterval(11);
        LimeLightTool llIt = new LimeLightTool(limelight);  // instantiate the lime light tools class, pulls IP address from LimeLight

        llIt.setDriverStationStreamSource();

        llIt.forwardAll();

        FtcDashboard.getInstance().startCameraStream(llIt.getStreamSource(),10);



        //ball detector index
        int currentPipeIndex = 4;
        limelight.pipelineSwitch(currentPipeIndex); // Switch to desired pipeline num


        waitForStart();



        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();



            //max 32 double array run through snapscript
            double[] pythonOutputs = result.getPythonOutput();



            teamUtil.log("Python output:");
            if (pythonOutputs != null && pythonOutputs.length > 0) {
                for(int i = 0; i<8; i++){
                    double var = pythonOutputs[i];
                    teamUtil.log(stringIdentifier(i)+var);
                }
            }

            telemetry.addLine("Current Pipeline Index: " + currentPipeIndex);


            if (detector.sampleDetector.frameDataQueue.peek()!=null) {
                frameData = detector.sampleDetector.frameDataQueue.peek();
                teamUtil.log("Rect Angle" + frameData.rectAngle);
            }

            if(gamepad1.dpadDownWasReleased()){
                detector.sampleDetector.configureCam(detector.portal, OpenCVSampleDetector.APEXPOSURE, OpenCVSampleDetector.AEPRIORITY, OpenCVSampleDetector.EXPOSURE, OpenCVSampleDetector.GAIN, OpenCVSampleDetector.WHITEBALANCEAUTO, OpenCVSampleDetector.TEMPERATURE, OpenCVSampleDetector.AFOCUS, OpenCVSampleDetector.FOCUSLENGTH);
            }

            if(gamepad1.dpadUpWasReleased()){
                currentPipeIndex++;
                if(currentPipeIndex>6){
                    currentPipeIndex = 0;
                }
                limelight.pipelineSwitch(currentPipeIndex); // Switch to desired pipeline num

            }






            telemetry.update();

        }
    }
}
