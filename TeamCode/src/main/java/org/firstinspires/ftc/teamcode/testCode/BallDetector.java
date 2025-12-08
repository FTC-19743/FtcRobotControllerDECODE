

package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.assemblies.Detector;
import org.firstinspires.ftc.teamcode.assemblies.OpenCVSampleDetector;
import org.firstinspires.ftc.teamcode.libs.LimeLightTool;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

//USE limelight test config for this opmode
@TeleOp
@Disabled
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
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard
        teamUtil.init(this);

        //limelight init
        initCam();

        //logitech init
        /*
        detector = new Detector();
        detector.initCV(true);
        detector.startCVPipeline();

         */

        telemetry.setMsTransmissionInterval(11);
        LimeLightTool llIt = new LimeLightTool(limelight);  // instantiate the lime light tools class, pulls IP address from LimeLight

        llIt.setDriverStationStreamSource();

        llIt.forwardAll();


        FtcDashboard.getInstance().startCameraStream(llIt.getStreamSource(),10);



        //ball detector index
        int currentPipeIndex = 6;
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
            telemetry.addData("TX : ", result.getTx());
            telemetry.addData("TY : ", result.getTy());
            telemetry.addData("TA : ", result.getTa());
            telemetry.addData("BotPOSE : ", result.getBotpose());

            if(Math.abs(result.getTx())>1){
                telemetry.addLine("Outside Shot Threshold");
            }else if(result.getTx()==0){
                telemetry.addLine("April Tag Not Seen");
            }else{
                telemetry.addLine("Within Shot Threshold");
            }




            /*
            if (detector.sampleDetector.frameDataQueue.peek()!=null) {
                frameData = detector.sampleDetector.frameDataQueue.peek();
                teamUtil.log("Rect Angle" + frameData.rectAngle);
            }

             */

            if(gamepad1.dpadDownWasReleased()){
                FtcDashboard.getInstance().stopCameraStream();
                FtcDashboard.getInstance().startCameraStream(llIt.getStreamSource(),10);
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
