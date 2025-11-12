package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.OctoQuadFWv3;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public OctoQuadFWv3 oq;
    public BasicDrive drive;
    public Intake intake;
    public Shooter shooter;
    public Blinkin blinkin;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public Limelight3A limelight;

    public Servo foot;
    private ColorSensor footColorSensor;
    public static double FOOT_CALIBRATE_POS = .1;
    public static double FOOT_EXTENDED_POS = .6; // 6-7 seconds TODO: Maybe lower to closer to ground while setting up to save a second or two

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

    public void initialize(boolean useLimeLight) {
        drive.initialize();
        intake.initialize();
        shooter.initialize();
        blinkin.init();
        footColorSensor = hardwareMap.get(RevColorSensorV3.class, "floorcolorsensor");

        if (useLimeLight) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
        }
    }

    public void initCV (boolean liveStream) {
        String webCamName;
        if (teamUtil.SIDE == teamUtil.Side.HUMAN) {
            webCamName = "webcamfront";
        } else if (teamUtil.alliance == teamUtil.Alliance.BLUE) {
            webCamName = "webcamright";
        } else {
            webCamName = "webcamleft";
        }
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, webCamName), aprilTag);
        if (liveStream) {
            visionPortal.resumeLiveView();
        } else {
            visionPortal.stopLiveView();
        }
    }

    public void stopCV () {
        visionPortal.close();
    }

    public void outputTelemetry() {
        //drive.driveMotorTelemetry();
    }
    private String formatSensor (ColorSensor sensor) {
        return String.format ("(%d/%d/%d/%d)",sensor.alpha(), sensor.red(), sensor.green(), sensor.blue());
    }

    public void outputFootSensor() {
        telemetry.addLine("FootSensor(A/R/G/B): " + formatSensor(footColorSensor));
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
        teamUtil.log("Setting Foot Position to: " + pos);
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

    public static double A01_TILE_LENGTH = 610;

    public static double A05_SHOOT1_Y = 610;
    public static double A05_SHOOT1_X = 610;
    public static double A05_SHOOT1_H = 45;

    public static double A06_SETUP1_Y = 1219;
    public static double A06_SETUP1_X = 813;
    public static double A06_SETUP1_H = 0;

    public static double A07_PICKUP1_Y = 1219;
    public static double A07_PICKUP1_X = 509;
    public static double A07_PICKUP1_H = 0;

    public static double B05_SHOOT1_Y = 483;
    public static double B05_SHOOT1_X = 1397;
    public static double B05_SHOOT1_H = 45; //TODO: FIND

    public static double B06_SETUP1_Y = 1219;
    public static double B06_SETUP1_X = -1320;
    public static double B06_SETUP1_H = 0;

    public static double B07_PICKUP1_Y = 1219;
    public static double B07_PICKUP1_X = -1095;
    public static double B07_PICKUP1_H = 0;




    public void goalSide(boolean useArms) {
        //Shoot Preloads
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A05_SHOOT1_X, A05_SHOOT1_Y, A05_SHOOT1_H, A00_SHOOT_END_VELOCITY,null,0,false,3000);
        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        if (useArms) {
            shootAllArtifacts();
            intake.intakeStart();
        } else {
            teamUtil.pause(2000);
        }

        //Collect Set 1
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A06_SETUP1_X, A06_SETUP1_Y, A06_SETUP1_H, A00_PICKUP_VELOCITY,null,0,false,3000);
        drive.mirroredMoveTo(A00_PICKUP_VELOCITY, A07_PICKUP1_X, A07_PICKUP1_Y, A07_PICKUP1_H, A00_PICKUP_END_VELOCITY,null,0,false,3000);
        if (useArms) {
            intake.elevatorToFlippersV2NoWait();
        }
        //Shoot Set 1
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A05_SHOOT1_X, A05_SHOOT1_Y, A05_SHOOT1_H, A00_SHOOT_END_VELOCITY,null,0,false,3000);
        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        if (useArms) {
            shootAllArtifacts();
            intake.intakeStart();
        } else {
            teamUtil.pause(2000);
        }

        //Collect Set 2
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A06_SETUP1_X-A01_TILE_LENGTH, A06_SETUP1_Y, A06_SETUP1_H, A00_PICKUP_VELOCITY,null,0,false,3000);
        drive.mirroredMoveTo(A00_PICKUP_VELOCITY, A07_PICKUP1_X-A01_TILE_LENGTH, A07_PICKUP1_Y, A07_PICKUP1_H, A00_PICKUP_END_VELOCITY,null,0,false,3000);
        if (useArms) {
            intake.elevatorToFlippersV2NoWait();
        }
        //Shoot Set 2
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A05_SHOOT1_X, A05_SHOOT1_Y, A05_SHOOT1_H, A00_SHOOT_END_VELOCITY,null,0,false,3000);
        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        if (useArms) {
            shootAllArtifacts();
            intake.intakeStart();
        } else {
            teamUtil.pause(2000);
        }

        //Collect Set 3
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A06_SETUP1_X-2*(A01_TILE_LENGTH), A06_SETUP1_Y, A06_SETUP1_H, A00_PICKUP_VELOCITY,null,0,false,3000);
        drive.mirroredMoveTo(A00_PICKUP_VELOCITY, A07_PICKUP1_X-2*(A01_TILE_LENGTH), A07_PICKUP1_Y, A07_PICKUP1_H, A00_PICKUP_END_VELOCITY,null,0,false,3000);
        if (useArms) {
            intake.elevatorToFlippersV2NoWait();
        }
        //Shoot Set 3
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A05_SHOOT1_X, A05_SHOOT1_Y, A05_SHOOT1_H, A00_SHOOT_END_VELOCITY,null,0,false,3000);
        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        if (useArms) {
            shootAllArtifacts();
            intake.intakeStart();
        } else {
            teamUtil.pause(2000);
        }


    }

    public static long FIRST_UNLOAD_PAUSE = 400;
    public static long SECOND_UNLOAD_PAUSE = 600;
    public static double TWO_BALL_EDGE_PORTION = 1f/4;

    public void shootAllArtifacts(){
        intake.checkLoadedArtifacts();
        int ballCount = intake.loadedBallNum();
        Intake.ARTIFACT[] loadedArtifacts = {intake.leftLoad, intake.middleLoad, intake.rightLoad};
        if(ballCount == 0){
            teamUtil.log("shootAllArtifacts called without loaded artifacts");
            return;
        }
        teamUtil.log("shootAllArtifacts starting with "+ballCount+" balls");
        if(ballCount == 1){
            if(loadedArtifacts[1] != Intake.ARTIFACT.NONE){
                shootArtifactColor(loadedArtifacts[1]);
            }if(loadedArtifacts[0] != Intake.ARTIFACT.NONE){
                shootArtifactColor(loadedArtifacts[0]);
            }else{
                shootArtifactColor(loadedArtifacts[2]);
            }
        }else if(ballCount == 2){
            if(loadedArtifacts[1] == Intake.ARTIFACT.NONE){
                intake.left_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
                intake.right_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
                teamUtil.pause(FIRST_UNLOAD_PAUSE);
                intake.left_flipper.setPosition(Intake.FLIPPER_CEILING);
                teamUtil.pause((long)(EDGE_PUSHER_PAUSE*(1-TWO_BALL_EDGE_PORTION)));
                intake.right_flipper.setPosition(Intake.FLIPPER_CEILING);
                teamUtil.pause((long)(EDGE_PUSHER_PAUSE*TWO_BALL_EDGE_PORTION));
                shooter.pusher.pushNNoWait(2, AxonPusher.RTP_MAX_VELOCITY, 1000);
            }if(loadedArtifacts[0] == Intake.ARTIFACT.NONE){
                intake.middle_flipper.setPosition(Intake.MIDDLE_FLIPPER_SHOOTER_TRANSFER);
                intake.right_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
                teamUtil.pause(FIRST_UNLOAD_PAUSE);
                intake.middle_flipper.setPosition(Intake.FLIPPER_CEILING);
                shooter.pusher.pushNNoWait(2, AxonPusher.RTP_MAX_VELOCITY, 1250);
                intake.right_flipper.setPosition(Intake.FLIPPER_CEILING);
            }else{
                intake.middle_flipper.setPosition(Intake.MIDDLE_FLIPPER_SHOOTER_TRANSFER);
                intake.left_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
                teamUtil.pause(FIRST_UNLOAD_PAUSE);
                intake.middle_flipper.setPosition(Intake.FLIPPER_CEILING);
                shooter.pusher.pushNNoWait(2, AxonPusher.RTP_MAX_VELOCITY, 1250);
                intake.left_flipper.setPosition(Intake.FLIPPER_CEILING);
            }
        }else{
            intake.middle_flipper.setPosition(Intake.MIDDLE_FLIPPER_SHOOTER_TRANSFER);
            intake.left_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
            intake.right_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
            teamUtil.pause(FIRST_UNLOAD_PAUSE);
            intake.middle_flipper.setPosition(Intake.FLIPPER_CEILING);
            shooter.pusher.pushNNoWait(3, AxonPusher.RTP_MAX_VELOCITY, 1250);
            intake.left_flipper.setPosition(Intake.FLIPPER_CEILING);
            teamUtil.pause(SECOND_UNLOAD_PAUSE);
            intake.right_flipper.setPosition(Intake.FLIPPER_CEILING);
        }
        intake.intakeIn();
        teamUtil.log("shootAllArtifacts finished");
    }

    public void shootAllArtifactsNoWait(){
        teamUtil.log("Launching Thread to shootAllArtifactsNoWait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                shootAllArtifacts();
            }
        });
        thread.start();
    }

    public static long EDGE_PUSHER_PAUSE = 700;

    public void shootArtifactColor(Intake.ARTIFACT color){
        teamUtil.log("shootArtifactColor called");
        intake.checkLoadedArtifacts();
        int ballCount = intake.loadedBallNum();
        Intake.ARTIFACT[] loadedArtifacts = {intake.leftLoad, intake.middleLoad, intake.rightLoad};
        if(color == Intake.ARTIFACT.NONE){
            teamUtil.log("shootArtifactColor called with ARTIFACT.NONE");
            return;
        }
        if(ballCount == 0) {
            teamUtil.log("shootArtifactColor called without any artifacts loaded");
            return;
        }
        if(loadedArtifacts[0] != color && loadedArtifacts[1] != color && loadedArtifacts[2] != color){
            teamUtil.log("shootArtifactColor called without a loaded artifact of the specified color");
            return;
        }
        if(color == loadedArtifacts[1]){
            intake.middle_flipper.setPosition(Intake.MIDDLE_FLIPPER_SHOOTER_TRANSFER);
            teamUtil.pause(FIRST_UNLOAD_PAUSE);
            intake.middle_flipper.setPosition(Intake.FLIPPER_CEILING);
            shooter.pushOne();
        }else if(color == loadedArtifacts[0]){
            intake.left_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
            teamUtil.pause(FIRST_UNLOAD_PAUSE);
            intake.left_flipper.setPosition(Intake.FLIPPER_CEILING);
            teamUtil.pause(EDGE_PUSHER_PAUSE);
            shooter.pushOne();
        }else{
            intake.right_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
            teamUtil.pause(FIRST_UNLOAD_PAUSE);
            intake.right_flipper.setPosition(Intake.FLIPPER_CEILING);
            teamUtil.pause(EDGE_PUSHER_PAUSE);
            shooter.pushOne();
        }
        if(ballCount == 1){
            intake.intakeStart();
        }

    }

    public void shootArtifactColorNoWait(Intake.ARTIFACT color){
        teamUtil.log("Launching Thread to shootArtifactColorNoWait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                shootArtifactColor(color);
            }
        });
        thread.start();
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

