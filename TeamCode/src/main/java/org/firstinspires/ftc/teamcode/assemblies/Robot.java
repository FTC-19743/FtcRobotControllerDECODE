package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Pattern.GPP;
import static org.firstinspires.ftc.teamcode.libs.teamUtil.Pattern.PGP;
import static org.firstinspires.ftc.teamcode.libs.teamUtil.Pattern.PPG;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.OctoQuadFWv3;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

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
    //
    public void detectPattern () {
        // TODO: Get april tag detections and decide what we are looking at. Set Blinkin accordingly
        int detectionNum;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if(currentDetections.isEmpty()){
            blinkin.setSignal(Blinkin.Signals.GOLD);
            telemetry.addLine("Not Detecting Anything");
        } else{
            if(teamUtil.alliance == teamUtil.Alliance.BLUE){
                int lowestYIndex=0;
                double lowestY = 0;
                for(int element = 0; element < currentDetections.size(); element++){
                    if(currentDetections.get(element).center.y>lowestY){
                        lowestYIndex = element;
                        lowestY=currentDetections.get(element).center.y;
                    }
                }
                if(details){
                    teamUtil.log("Lowest Y" + lowestY);
                }

                detectionNum=currentDetections.get(lowestYIndex).id;
            }
            else{
                int highestYIndex=0;
                double highestY = 10000;
                for(int element = 0; element < currentDetections.size(); element++){
                    if(currentDetections.get(element).center.y<highestY){
                        highestYIndex = element;
                        highestY=currentDetections.get(element).center.y;
                    }
                }
                if(details){
                    teamUtil.log("Highest Y" + highestY);
                }
                detectionNum=currentDetections.get(highestYIndex).id;
            }

            if(detectionNum==23){
                teamUtil.pattern = PPG;
                blinkin.setSignal(teamUtil.alliance == teamUtil.Alliance.BLUE ? Blinkin.Signals.PPG_BLUE : Blinkin.Signals.PPG_RED);
            }else if(detectionNum==22){
                teamUtil.pattern = PGP;
                blinkin.setSignal(teamUtil.alliance == teamUtil.Alliance.BLUE ? Blinkin.Signals.PGP_BLUE : Blinkin.Signals.PGP_RED);
            }else{
                teamUtil.pattern = GPP;
                blinkin.setSignal(teamUtil.alliance == teamUtil.Alliance.BLUE ? Blinkin.Signals.GPP_BLUE : Blinkin.Signals.GPP_RED);
            }
            telemetry.addLine("Detection ID: " + detectionNum);
        }

    }

    public void testDetectPattern (AprilTagProcessor processor) {
        // TODO: Get april tag detections and decide what we are looking at. Set Blinkin accordingly
        int detectionNum;
        List<AprilTagDetection> currentDetections = processor.getDetections();
        if(currentDetections.isEmpty()){
            blinkin.setSignal(Blinkin.Signals.GOLD);
            telemetry.addLine("Not Detecting Anything");
        } else{
            if(teamUtil.alliance == teamUtil.Alliance.BLUE){
                int lowestYIndex=0;
                double lowestY = 0;
                for(int element = 0; element < currentDetections.size(); element++){
                    if(currentDetections.get(element).center.y>lowestY){
                        lowestYIndex = element;
                        lowestY=currentDetections.get(element).center.y;
                    }
                }
                if(details){
                    teamUtil.log("Lowest Y" + lowestY);
                }

                detectionNum=currentDetections.get(lowestYIndex).id;
            }
            else{
                int highestYIndex=0;
                double highestY = 10000;
                for(int element = 0; element < currentDetections.size(); element++){
                    if(currentDetections.get(element).center.y<highestY){
                        highestYIndex = element;
                        highestY=currentDetections.get(element).center.y;
                    }
                }
                if(details){
                    teamUtil.log("Highest Y" + highestY);
                }
                detectionNum=currentDetections.get(highestYIndex).id;
            }

            if(detectionNum==23){
                teamUtil.pattern = PPG;
                blinkin.setSignal(teamUtil.alliance == teamUtil.Alliance.BLUE ? Blinkin.Signals.PPG_BLUE : Blinkin.Signals.PPG_RED);
            }else if(detectionNum==22){
                teamUtil.pattern = PGP;
                blinkin.setSignal(teamUtil.alliance == teamUtil.Alliance.BLUE ? Blinkin.Signals.PGP_BLUE : Blinkin.Signals.PGP_RED);
            }else{
                teamUtil.pattern = GPP;
                blinkin.setSignal(teamUtil.alliance == teamUtil.Alliance.BLUE ? Blinkin.Signals.GPP_BLUE : Blinkin.Signals.GPP_RED);
            }
            telemetry.addLine("Detection ID: " + detectionNum);
        }

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
    // Shooter Code

    public boolean shooterFlyWheelsReady() {
        return Math.abs(shooter.rightFlywheel.getVelocity() - Shooter.VELOCITY_COMMANDED) < Shooter.VELOCITY_COMMANDED_THRESHOLD &&
                Math.abs(shooter.leftFlywheel.getVelocity() - Shooter.VELOCITY_COMMANDED) < Shooter.VELOCITY_COMMANDED_THRESHOLD;
    }
    public boolean shooterHeadingReady() {
        return Math.abs(drive.getHeadingODO() - drive.robotGoalHeading()) < BasicDrive.HEADING_CAN_SHOOT_THRESHOLD;
    }
    public boolean canShoot(){
        return shooterFlyWheelsReady() && shooterHeadingReady();
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

    // TODO: Maybe we can make this faster by knowing the full sequence and moving 2nd and 3rd shots into position earlier?
    // TODO: (especially while driving into shooting position)
    public boolean shootPatternAuto() {
        drive.loop();
        shooter.adjustShooterV2(drive.robotGoalDistance());
        drive.spinToHeading(drive.robotGoalHeading());
        drive.stopMotors();
        // TODO: Do we need a final spin to adjust heading if it is off?

        // Wait for Flywheels to be ready
        while (!shooterFlyWheelsReady() && teamUtil.keepGoing(3000)) {teamUtil.pause(100);}

        if (teamUtil.pattern==PPG || teamUtil.pattern==PGP) {
            shootArtifactColor(Intake.ARTIFACT.PURPLE);
        } else {
            shootArtifactColor(Intake.ARTIFACT.GREEN);
        }
        if (teamUtil.pattern==PPG || teamUtil.pattern==GPP) {
            shootArtifactColor(Intake.ARTIFACT.PURPLE);
        } else {
            shootArtifactColor(Intake.ARTIFACT.GREEN);
        }
        if (teamUtil.pattern==PGP || teamUtil.pattern==GPP) {
            shootArtifactColor(Intake.ARTIFACT.PURPLE);
        } else {
            shootArtifactColor(Intake.ARTIFACT.GREEN);
        }
        return true;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Auto Code

    public static int GOAL_LOCALIZE_X = 1617;
    public static int GOAL_LOCALIZE_Y = 824;
    public static double GOAL_LOCALIZE_H = 0;
    public static int HUMAN_LOCALIZE_X = 0;
    public static int HUMAN_LOCALIZE_Y = 0;
    public static double HUMAN_LOCALIZE_H = 0;

    public void setStartLocalizedPosition () {
        if (teamUtil.SIDE== teamUtil.Side.GOAL) {
            if (teamUtil.alliance== teamUtil.Alliance.BLUE) {
                drive.setRobotPosition(GOAL_LOCALIZE_X, GOAL_LOCALIZE_Y, GOAL_LOCALIZE_H);
            } else {
                drive.setRobotPosition(GOAL_LOCALIZE_X, -GOAL_LOCALIZE_Y, GOAL_LOCALIZE_H);
            }
        } else {
            if (teamUtil.alliance== teamUtil.Alliance.BLUE) {
                drive.setRobotPosition(HUMAN_LOCALIZE_X, HUMAN_LOCALIZE_Y, HUMAN_LOCALIZE_H);
            } else {
                drive.setRobotPosition(HUMAN_LOCALIZE_X, -HUMAN_LOCALIZE_Y, HUMAN_LOCALIZE_H);
            }
        }
    }

    public static double A00_MAX_SPEED_NEAR_GOAL = 1500;
    public static double A00_SHOOT_END_VELOCITY = 300;
    public static double A01_SHOOT_END_VELOCITY = 1000;
    public static double A00_FINAL_END_VELOCITY = 300;

    public static double A00_PICKUP_VELOCITY = 1000;
    public static double A00_PICKUP_END_VELOCITY = 1000;

    public static double A01_TILE_LENGTH = 610;

    public static double A05_DRIFT_1 = 50;
    public static double A05_DRIFT_2 = 140;

    public static double A05_SHOOT1_Y = 750;
    public static double A05_SHOOT1_X = 750;
    public static double A05_SHOOT1_H = 45;

    public static double A06_SETUP1_Y = 1220-100;
    public static double A06_SETUP1_X = 670;
    public static double A06_SETUP1_H = 0;

    public static double A07_SETUP1_Y = 1220-50;
    public static double A07_PICKUP1_Y = 1200;
    public static double A07_PICKUP1_X = 400;
    public static double A07_PICKUP1_H = 0;
    public static double A07_END_PICKUP_X_ADJUSTMENT = 100;


    public static double A08_SHOOT1_Y = 634; // TODO: Reconcile this approach with the shooter pre-aim?
    public static double A08_SHOOT1_X = 617;
    public static double A08_SHOOT1_H = 45;

    public static double A09_SHOOT1_Y = 460; // TODO: Reconcile this approach with the shooter pre-aim?
    public static double A09_SHOOT1_X = 440;
    public static double A09_SHOOT1_H = 45;



    public void logShot(int num, int targetX, int targetY, int goalDistance, double goalHeading) {
        drive.loop();
        teamUtil.log("---------------- SHOT " + num);
        teamUtil.log("Target X: " + targetX + " Actual : " + drive.oQlocalizer.posX_mm + " Diff: " + Math.abs(targetX-drive.oQlocalizer.posX_mm));
        teamUtil.log("Target Y: " + targetY + " Actual : " + drive.oQlocalizer.posY_mm + " Diff: " + Math.abs(targetY-drive.oQlocalizer.posY_mm));
        teamUtil.log("Target Distance: " + goalDistance + " Actual Distance: " + (int)drive.robotGoalDistance() + " Diff: " + (int)Math.abs(goalDistance-drive.robotGoalDistance()));
        teamUtil.log("Target Heading: " + goalHeading + " Actual Heading: " + drive.getHeadingODO() + " Diff: " + Math.abs(goalHeading-drive.getHeadingODO()));
    }

    public void goalSide(boolean useArms) {
        double goalDistance = 0;

        // Prep Shooter
        goalDistance = drive.getGoalDistance((int)A05_SHOOT1_X, (int)A05_SHOOT1_Y);
        if (useArms) shooter.adjustShooterV2(goalDistance);

        //Shoot Preloads
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A05_SHOOT1_X+A05_DRIFT_1, A05_SHOOT1_Y+A05_DRIFT_1, A05_SHOOT1_H, A00_SHOOT_END_VELOCITY,null,0,false,3000);

        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        logShot(1, (int)A05_SHOOT1_X, (int)A05_SHOOT1_Y, (int)goalDistance, A05_SHOOT1_H);

        if (useArms) {
            shootPatternAuto();
            intake.intakeStart();
        } else {
            teamUtil.pause(2000);
        }

        //Collect Set 1
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A06_SETUP1_X, A06_SETUP1_Y, A06_SETUP1_H, A00_PICKUP_VELOCITY,null,0,false,3000);
        drive.straightHoldingStrafeEncoder(A00_PICKUP_VELOCITY, A07_PICKUP1_X, A07_PICKUP1_Y, (int)A07_PICKUP1_H,A00_PICKUP_END_VELOCITY,false,null,0,2000);
        //drive.mirroredMoveTo(A00_PICKUP_VELOCITY, A07_PICKUP1_X, A07_PICKUP1_Y, A07_PICKUP1_H, A00_PICKUP_END_VELOCITY,null,0,false,3000);
        if (useArms) {
            intake.elevatorToFlippersV2NoWait();
        }
        // Prep Shooter
        goalDistance = drive.getGoalDistance((int)A08_SHOOT1_X, (int)A08_SHOOT1_Y);
        if (useArms) shooter.adjustShooterV2(goalDistance);
        //Shoot Set 1
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A08_SHOOT1_X-A05_DRIFT_2, A08_SHOOT1_Y+A05_DRIFT_2, A08_SHOOT1_H, A01_SHOOT_END_VELOCITY,null,0,false,3000);
        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        logShot(2, (int)A08_SHOOT1_X, (int)A08_SHOOT1_Y, (int)goalDistance, A08_SHOOT1_H);
        if (useArms) {
            shootPatternAuto();
            intake.intakeStart();
        } else {
            teamUtil.pause(2000);
        }

        //Collect Set 2
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A06_SETUP1_X-A01_TILE_LENGTH, A07_SETUP1_Y, A06_SETUP1_H, A00_PICKUP_VELOCITY,null,0,false,3000);
        drive.straightHoldingStrafeEncoder(A00_PICKUP_VELOCITY, A07_PICKUP1_X-A01_TILE_LENGTH, A07_PICKUP1_Y, (int)A07_PICKUP1_H,A00_PICKUP_END_VELOCITY,false,null,0,2000);
        //drive.mirroredMoveTo(A00_PICKUP_VELOCITY, A07_PICKUP1_X-A01_TILE_LENGTH, A07_PICKUP1_Y, A07_PICKUP1_H, A00_PICKUP_END_VELOCITY,null,0,false,3000);
        if (useArms) {
            intake.elevatorToFlippersV2NoWait();
        }
        // Prep Shooter
        goalDistance = drive.getGoalDistance((int)A09_SHOOT1_X, (int)A09_SHOOT1_Y);
        if (useArms) shooter.adjustShooterV2(goalDistance);

        //Shoot Set 2
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A09_SHOOT1_X-A05_DRIFT_2, A09_SHOOT1_Y+A05_DRIFT_2, A09_SHOOT1_H, A01_SHOOT_END_VELOCITY,null,0,false,3000);

        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        logShot(2, (int)A09_SHOOT1_X, (int)A09_SHOOT1_Y, (int)goalDistance, A09_SHOOT1_H);
        if (useArms) {
            shootPatternAuto();
            intake.intakeStart();
        } else {
            teamUtil.pause(2000);
        }

        shooter.stopShooter();

        if (useArms) {
            intake.intakeStart();
        }

        //Pick up last 3 balls
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A06_SETUP1_X-(2*A01_TILE_LENGTH)+A07_END_PICKUP_X_ADJUSTMENT, A07_SETUP1_Y, A06_SETUP1_H, A00_FINAL_END_VELOCITY,null,0,false,3000);
        drive.straightHoldingStrafeEncoder(A00_PICKUP_VELOCITY, A07_PICKUP1_X-(2*A01_TILE_LENGTH), A07_PICKUP1_Y, (int)A07_PICKUP1_H,A00_PICKUP_END_VELOCITY,false,null,0,2000);

        drive.stopMotors();
        intake.stopDetector();
        intake.intakeStop();


        if(true) return;
        //Collect Set 3
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A06_SETUP1_X-2*(A01_TILE_LENGTH), A06_SETUP1_Y, A06_SETUP1_H, A00_PICKUP_VELOCITY,null,0,false,3000);
        drive.mirroredMoveTo(A00_PICKUP_VELOCITY, A07_PICKUP1_X-2*(A01_TILE_LENGTH), A07_PICKUP1_Y, A07_PICKUP1_H, A00_PICKUP_END_VELOCITY,null,0,false,3000);
        if (useArms) {
            intake.elevatorToFlippersV2NoWait();
        }
        //Shoot Set 3
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A09_SHOOT1_X, A09_SHOOT1_Y, A09_SHOOT1_H, A01_SHOOT_END_VELOCITY,null,0,false,3000);
        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        if (useArms) {
            shootAllArtifacts();
            intake.intakeStart();
        } else {
            teamUtil.pause(2000);
        }
    }

    public void humanSide(boolean useArms) {
    }



}

