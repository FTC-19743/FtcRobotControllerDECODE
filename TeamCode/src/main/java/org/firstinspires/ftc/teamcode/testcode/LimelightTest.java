

package org.firstinspires.ftc.teamcode.testcode;

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
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.Arrays;


@TeleOp
public class LimelightTest extends LinearOpMode
{


    Limelight3A limelight;
    Detector detector;
    OpenCVSampleDetector.FrameData frameData = null;




    public void initCam() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
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


        waitForStart();

        OpenCVSampleDetector.FrameData newFrame;


        int currentPipeIndex = 5;
        limelight.pipelineSwitch(currentPipeIndex); // Switch to desired pipeline num


        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();


            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
//

            } else {
                telemetry.addData("Limelight", "No Targets");
            }
            
            //max 32 double array run through snapscript
            double[] pythonOutputs = result.getPythonOutput();

            teamUtil.log("Python output:");
            if (pythonOutputs != null && pythonOutputs.length > 0) {
                for(int i = 0; i<8; i++){
                    double var = pythonOutputs[i];
                    teamUtil.log(" " + var);
                }
                double firstOutput = pythonOutputs[0];
            }

            telemetry.addLine("Current Pipeline Index: " + currentPipeIndex);



            if(gamepad1.dpadUpWasReleased()){
                currentPipeIndex++;
                if(currentPipeIndex>6){
                    currentPipeIndex = 0;
                }
                limelight.pipelineSwitch(currentPipeIndex); // Switch to desired pipeline num

            }



            if (detector.sampleDetector.frameDataQueue.peek()!=null) {
                frameData = detector.sampleDetector.frameDataQueue.peek();
                teamUtil.log("Rect Angle" + frameData.rectAngle);
            }










            telemetry.addLine("Valid?: " + result.isValid());
            telemetry.addLine("Null?: " + (result == null));
            telemetry.update();

        }
    }
}
