package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.OctoQuadFWv3;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
public class Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public OctoQuadFWv3 oq;
    public BasicDrive drive;
    public Intake intake;
    public Shooter shooter;
    public Blinkin blinkin;
    public Limelight3A limelight;

    public Servo foot;
    public static double FOOT_CALIBRATE_POS = .1;
    public static double FOOT_EXTENDED_POS = .8;

    public static boolean details = false;

    // Set teamUtil.theOpMode before calling
    public Robot() {
        telemetry = teamUtil.theOpMode.telemetry;
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        drive = new BasicDrive();
        intake = new Intake();
        shooter = new Shooter();
        foot = hardwareMap.get(Servo.class,"pushup");
        blinkin = new Blinkin(hardwareMap,telemetry);
        teamUtil.robot = this;
    }

    public void initialize() {
        drive.initialize();
        intake.initialize();
        shooter.initialize();
        blinkin.init();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //telemetry.setMsTransmissionInterval(11); //TODO This is in the limelight example code. why?
        limelight.pipelineSwitch(0);
        limelight.start();

    }

    public void outputTelemetry() {
        drive.driveMotorTelemetry();
        intake.intakeTelemetry();
        shooter.outputTelemetry();
        telemetry.update();
    }

    public void outputLLPose() {
        LLResult result = limelight.getLatestResult();
        Pose3D botpose = result.getBotpose();
        telemetry.addData("Botpose", botpose.toString());
    }

    public void calibrate() {
        drive.calibrate();
        intake.calibrate();
        shooter.calibrate();
    }

    public void setFootPos(double pos){
        foot.setPosition(pos);
    }

    public void resetRobot(){

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Auto Code

    public static double A00_MAX_SPEED_NEAR_GOAL = 1500;
    public static double A00_SHOOT_END_VELOCITY = 1000;
    public static double A00_PICKUP_VELOCITY = 1000;
    public static double A00_PICKUP_END_VELOCITY = 1000;

    public static double A05_SHOOT1_Y = 884;
    public static double A05_SHOOT1_X = 935;
    public static double A05_SHOOT1_H = 45;

    public static double A06_PICKUP1_Y = 1200;
    public static double A06_PICKUP1_X = 667;
    public static double A06_PICKUP1_H = 0;

    public static double A07_PICKUP1_Y = 1200;
    public static double A07_PICKUP1_X = 667-120;

    public void goalSide(boolean useArms) {
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A05_SHOOT1_X, A05_SHOOT1_Y, A05_SHOOT1_H, A00_SHOOT_END_VELOCITY,null,0,false,3000);
        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        if (useArms) {

        } else {
            teamUtil.pause(2000);
        }
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A06_PICKUP1_X, A06_PICKUP1_Y, A06_PICKUP1_H, A00_PICKUP_VELOCITY,null,0,false,3000);
        drive.mirroredMoveTo(A00_PICKUP_VELOCITY, A07_PICKUP1_X, A07_PICKUP1_Y, A06_PICKUP1_H, A00_PICKUP_END_VELOCITY,null,0,false,3000);
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A05_SHOOT1_X, A05_SHOOT1_Y, A05_SHOOT1_H, A00_SHOOT_END_VELOCITY,null,0,false,3000);
        drive.stopMotors();
        drive.waitForRobotToStop(1000);

    }

    public static long FIRST_UNLOAD_PAUSE = 400;
    public static long SECOND_UNLOAD_PAUSE = 600;

    public void shootAllArtifacts(){
        intake.checkLoadedArtifacts();
        int ballCount = intake.loadedBallNum();
        Intake.ARTIFACT[] loadedArtifacts = {intake.leftLoad, intake.middleLoad, intake.rightLoad};
        if(ballCount == 5){
            teamUtil.log("shootAllArtifacts called without loaded artifacts");
            return;
        }
        teamUtil.log("shootAllArtifacts starting with "+ballCount+" balls");
//        if(ballCount == 1){
//        }else if(ballCount == 2){
//        }else{
            intake.middle_flipper.setPosition(Intake.MIDDLE_FLIPPER_SHOOTER_TRANSFER);
            intake.left_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
            intake.right_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
            teamUtil.pause(FIRST_UNLOAD_PAUSE);
            intake.middle_flipper.setPosition(Intake.FLIPPER_CEILING);
            shooter.pusher.pushNNoWait(3, AxonPusher.RTP_MAX_VELOCITY, 1250);
            intake.left_flipper.setPosition(Intake.FLIPPER_CEILING);
            teamUtil.pause(SECOND_UNLOAD_PAUSE);
            intake.right_flipper.setPosition(Intake.FLIPPER_CEILING);
//        }
        teamUtil.log("shootAllArtifacts finished");
    }
    // Examples from last year's Sample Auto
    // Move to first drop
    //drive.moveTo(A00_MAX_SPEED_NEAR_BUCKET, A04_READY_FOR_BUCKET_STRAFE,A04_READY_FOR_BUCKET_STRAIGHT,0,A04_END_VELOCITY,null,0, false,5000);
    //drive.moveTo(A00_MAX_SPEED_NEAR_BUCKET, A05_1_BUCKET_STRAFE, A05_1_BUCKET_STRAIGHT,A05_1_BUCKET_HEADING,A05_1_BUCKET_END_VELOCITY,null,0, false, 5000);
    //drive.setMotorsActiveBrake();

    // Move to pickup 1st sample
    //drive.moveTo(A00_MAX_SPEED_NEAR_BUCKET,A06_1_SAMPLE_PICKUP_STRAFE,A06_1_SAMPLE_PICKUP_STRAIGHT,A06_1_SAMPLE_PICKUP_RH,A00_END_VELOCITY_FOR_PICKUP,null,0,false,3000);
    //drive.stopMotors();
    //drive.waitForRobotToStop(1000);
    //teamUtil.pause(A06_1_BRAKE_PAUSE);

    // Grab and unload (counting on bucket to be at the bottom by the time we get there!
    //boolean grabbedSample=intake.autoGoToSampleAndGrabV3(false,false,true,A12_SAMPLE_PICKUP_TIMEOUT);

    // Move to pickup 2nd sample
    //drive.moveTo(A00_MAX_SPEED_NEAR_BUCKET,A08_2_SAMPLE_PICKUP_STRAFE,A08_2_SAMPLE_PICKUP_STRAIGHT,A08_2_SAMPLE_PICKUP_HEADING,A00_END_VELOCITY_FOR_PICKUP,null,0,false,3000);
    //drive.stopMotors();
    //drive.waitForRobotToStop(1000);
    //teamUtil.pause(A06_1_BRAKE_PAUSE);

    // Move to pickup 3rd sample
    //drive.moveTo(A00_MAX_SPEED_NEAR_BUCKET,A10_3_SAMPLE_PICKUP_STRAFE,A10_3_SAMPLE_PICKUP_STRAIGHT,A10_3_SAMPLE_PICKUP_HEADING,A00_END_VELOCITY_FOR_PICKUP,null,0,false,3000);
    // drive.stopMotors();
    //drive.waitForRobotToStop(1000);
    //teamUtil.pause(A06_1_BRAKE_PAUSE);
    // Grab and unload (counting on bucket to be at the bottom by the time we get there!


}

