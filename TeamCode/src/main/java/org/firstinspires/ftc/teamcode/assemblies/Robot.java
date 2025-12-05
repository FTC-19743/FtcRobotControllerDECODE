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
    public enum Location{LEFT, CENTER, RIGHT}; // correspond to the flippers (reversed if facing the robot)

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
        if (details) {
            teamUtil.log("shooterFlyWheelsReady Waiting: TVel: " + Shooter.VELOCITY_COMMANDED + " RVel: " + shooter.rightFlywheel.getVelocity() + " LVel: " + shooter.leftFlywheel.getVelocity());
        }
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

    public void shootArtifactLocation(Robot.Location location){ //
        teamUtil.log("shootArtifactLocation called");
        // consider adding checks?
        if(location == Robot.Location.CENTER){
            intake.middle_flipper.setPosition(Intake.MIDDLE_FLIPPER_SHOOTER_TRANSFER);
            teamUtil.pause(FIRST_UNLOAD_PAUSE);
            intake.middle_flipper.setPosition(Intake.FLIPPER_CEILING);
            shooter.pushOne();
        }else if(location == Robot.Location.LEFT){
            intake.left_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
            teamUtil.pause(FIRST_UNLOAD_PAUSE);
            intake.left_flipper.setPosition(Intake.FLIPPER_CEILING);
            teamUtil.pause(EDGE_PUSHER_PAUSE);
            shooter.pushOne();
        }else{ // right
            intake.right_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
            teamUtil.pause(FIRST_UNLOAD_PAUSE);
            intake.right_flipper.setPosition(Intake.FLIPPER_CEILING);
            teamUtil.pause(EDGE_PUSHER_PAUSE);
            shooter.pushOne();
        }
        int ballCount = intake.loadedBallNum();
        if(ballCount == 1){
            intake.intakeStart();
        }

    }

    public void autoShootArtifacts(teamUtil.Pattern loadPattern){
        if(teamUtil.pattern == PPG){
            if(loadPattern == teamUtil.Pattern.PPG){
                shootArtifactLocation(Location.LEFT);
                shootArtifactLocation(Location.CENTER);
                shootArtifactLocation(Location.RIGHT); // green is right and last
            }else if(loadPattern == teamUtil.Pattern.PGP){
                shootArtifactLocation(Location.LEFT);
                shootArtifactLocation(Location.RIGHT);
                shootArtifactLocation(Location.CENTER); // green is center and last
            }else{//gpp
                shootArtifactLocation(Location.RIGHT);
                shootArtifactLocation(Location.CENTER);
                shootArtifactLocation(Location.LEFT); // green is left and last
            }
        }else if(teamUtil.pattern == PGP){
            if(loadPattern == teamUtil.Pattern.PPG){
                shootArtifactLocation(Location.LEFT);
                shootArtifactLocation(Location.RIGHT); // green is right and center
                shootArtifactLocation(Location.CENTER);
            }else if(loadPattern == teamUtil.Pattern.PGP){
                shootArtifactLocation(Location.LEFT);
                shootArtifactLocation(Location.CENTER); // green is center and center
                shootArtifactLocation(Location.RIGHT);
            }else{//gpp
                shootArtifactLocation(Location.RIGHT);
                shootArtifactLocation(Location.LEFT); // green is left and center
                shootArtifactLocation(Location.CENTER);
            }
        }else{ // GPP
            if(loadPattern == teamUtil.Pattern.PPG){
                shootArtifactLocation(Location.RIGHT); // green is right and first
                shootArtifactLocation(Location.LEFT);
                shootArtifactLocation(Location.CENTER);
            }else if(loadPattern == teamUtil.Pattern.PGP){
                shootArtifactLocation(Location.CENTER); // green is center and first
                shootArtifactLocation(Location.LEFT);
                shootArtifactLocation(Location.RIGHT);
            }else{//gpp
                shootArtifactLocation(Location.LEFT); // green is left and first
                shootArtifactLocation(Location.RIGHT);
                shootArtifactLocation(Location.CENTER);
            }
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
    public boolean shootPatternAuto(teamUtil.Pattern loadPattern) {
        teamUtil.log("shootPatternAuto");
        drive.loop();
        shooter.adjustShooterV2(drive.robotGoalDistance());
        drive.spinToHeadingV2(drive.robotGoalHeading(), 3000); // Will not change distance to target
        drive.stopMotors();

        // Wait for Flywheels to be ready
        long now = System.currentTimeMillis();
        long timeOutTime = System.currentTimeMillis() + 3000; // max time to wait for flywheels
        while (!shooterFlyWheelsReady() && teamUtil.keepGoing(timeOutTime)) {
            teamUtil.pause(100); // TODO: shorten pause
        }
        teamUtil.log("Waited " + (System.currentTimeMillis()-now) + "millisecs for flywheels");
        /*
        if (teamUtil.loadPattern==PPG || teamUtil.loadPattern==PGP) {
            shootArtifactColor(Intake.ARTIFACT.PURPLE);
            teamUtil.log("Shot purple");
        } else {
            shootArtifactColor(Intake.ARTIFACT.GREEN);
            teamUtil.log("Shot green");
        }
        if (teamUtil.loadPattern==PPG || teamUtil.loadPattern==GPP) {
            shootArtifactColor(Intake.ARTIFACT.PURPLE);
            teamUtil.log("Shot purple");
        } else {
            shootArtifactColor(Intake.ARTIFACT.GREEN);
            teamUtil.log("Shot green");
        }
        if (teamUtil.loadPattern==PGP || teamUtil.loadPattern==GPP) {
            shootArtifactColor(Intake.ARTIFACT.PURPLE);
            teamUtil.log("Shot purple");
        } else {
            shootArtifactColor(Intake.ARTIFACT.GREEN);
            teamUtil.log("Shot green");
        }
        teamUtil.log("shootPatternAuto Finished");
        */
        autoShootArtifacts(loadPattern);
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
    public static double A05_SHOT1_VEL = 1200;

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

    public static double A90_PARK_DRIFT = 200;
    public static double A90_PARK_X = 0 - A90_PARK_DRIFT;
    public static double A90_PARK_END_VELOCITY = 2000;

    public static double A99_GRAB_LAST_THREE_TIME = 4000;
    public static double A99_PARK_TIME = 1500;
    public static double A99_MOVE_OFF_LINE_TIME = 1500;


    public void logShot(int num, int targetX, int targetY, int goalDistance, double goalHeading) {
        drive.loop();
        teamUtil.log("---------------- SHOT " + num);
        teamUtil.log("Target X: " + targetX + " Actual : " + drive.oQlocalizer.posX_mm + " Diff: " + Math.abs(targetX-drive.oQlocalizer.posX_mm));
        teamUtil.log("Target Y: " + targetY + " Actual : " + drive.oQlocalizer.posY_mm + " Diff: " + Math.abs(targetY-drive.oQlocalizer.posY_mm));
        teamUtil.log("Target Distance: " + goalDistance + " Actual Distance: " + (int)drive.robotGoalDistance() + " Diff: " + (int)Math.abs(goalDistance-drive.robotGoalDistance()));
        teamUtil.log("Target Heading: " + goalHeading + " Actual Heading: " + drive.getHeadingODO() + " Diff: " + Math.abs(goalHeading-drive.getHeadingODO()));
    }

    public boolean seeLine(){
        return  (teamUtil.alliance== teamUtil.Alliance.BLUE && footColorSensor.blue() > LIFT_AUTO_ALIGN_BLUE_THRESHOLD) ||
                (teamUtil.alliance== teamUtil.Alliance.RED && footColorSensor.red() > LIFT_AUTO_ALIGN_RED_THRESHOLD);
    }

    public static double B00_MAX_SPEED = 2200;
    public static double B00_CORNER_VELOCITY = 1800;
    public static double B00_SHOOT_VELOCITY = 100;
    public static double B00_PICKUP_VELOCITY = 2000;
    public static double B00_PICKUP_END_VELOCITY = 2000;
    public static double B01_TILE_LENGTH = 610;

    public static double B05_DRIFT_1 = 50;
    public static double B05_DRIFT_2 = 140;

    public static double B05_SHOOT1_END_VEL = 400;
    public static double B05_SHOOT1_Y = 850;
    public static double B05_SHOOT1_X = 850;
    public static double B05_SHOOT1_H = 45;
    public static double B05_SHOT1_VEL = 1200;

    public static double B06_PICKUP1_Y = 1200;
    public static double B06_SETUP_Y_DRIFT = 200;
    public static double B06_SETUP1_Y = B06_PICKUP1_Y-B06_SETUP_Y_DRIFT;
    public static double B06_SETUP1_X = 670;
    public static double B06_SETUP1_DH = 90;
    public static double B06_SETUP1_H = 0;
    public static double B06_SETUP_END_VEL = B00_CORNER_VELOCITY;
    public static long B06_SETUP1_PAUSE = 150;

    public static double B07_PICKUP1_X = 420;
    public static double B07_PICKUP1_H = 180;
    public static double B07_END_PICKUP_X_ADJUSTMENT = 100;
    public static double B07_SETUP2_DH = 105;
    public static double B07_SETUP_Y_DRIFT = 50;
    public static double B07_SETUP2_Y = B06_PICKUP1_Y-B07_SETUP_Y_DRIFT;

    public static double B08_SHOOT2_Y = 650; // TODO: Reconcile this approach with the shooter pre-aim?
    public static double B08_SHOOT2_DRIFT = 200;
    public static double B08_SHOOT2_X = 650;
    public static double B08_SHOOT2_H = 45;
    public static double B08_SHOOT2_DH = 315;
    public static double B08_SHOOT2_END_VEL = 400;

    public static double B08_SHOOT3_Y = B08_SHOOT2_Y; // TODO: Reconcile this approach with the shooter pre-aim?
    public static double B08_SHOOT3_DRIFT = 100;
    public static double B08_SHOOT3_X = B08_SHOOT2_X;
    public static double B08_SHOOT3_H = B08_SHOOT2_H;
    public static double B08_SHOOT3_DH = 340;
    public static double B08_SHOOT3_END_VEL = 400;

    public static double B08_SHOOT4_Y = B08_SHOOT3_Y; // TODO: Reconcile this approach with the shooter pre-aim?
    public static double B08_SHOOT4_DRIFT = 100;
    public static double B08_SHOOT4_X = B08_SHOOT3_X;
    public static double B08_SHOOT4_H = B08_SHOOT3_H;
    public static double B08_SHOOT4_DH = 350;
    public static double B08_SHOOT4_END_VEL = 400;

    public static double B90_PARK_DRIFT = 200;
    public static double B90_PARK_X = 0 - A90_PARK_DRIFT;
    public static double B90_PARK_END_VELOCITY = 2000;

    public static double B00_FINAL_END_VELOCITY = 300;

    public static double B99_GRAB_LAST_THREE_TIME = 4000;
    public static double B99_PARK_TIME = 1500;
    public static double B99_MOVE_OFF_LINE_TIME = 1500;

    // Move while shooting adjusting robot heading and shooter as needed
    // Assume 600+600+900 for 3 color ordered shots
    // Assume 1000 for 3 fast
    public boolean driveWhileShooting(boolean useArms, boolean pattern, double driveHeading, double velocity, long timeOut) {
        teamUtil.log("driveWhileShooting driveH: " + driveHeading + " Vel: " + velocity);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        long shot3Time = System.currentTimeMillis() + (pattern ? 2100 : 1100);

        if (!useArms) {
            blinkin.setSignal(Blinkin.Signals.GOLD);
        }
        while (teamUtil.keepGoing(timeOutTime)) {
            drive.loop();
            double shotHeading = drive.robotGoalHeading();
            drive.driveMotorsHeadingsFR(driveHeading, shotHeading, velocity);
            // TODO: Need code in here to shoot all available artifacts either fast or in pattern order
            if (System.currentTimeMillis() > shot3Time) {
                break;
            }
        }
        if (!useArms) {
            blinkin.setSignal(Blinkin.Signals.OFF);
        }
        if (System.currentTimeMillis() <= timeOutTime) {
            teamUtil.log("driveWhileShooting Finished");
            return true;
        } else {
            teamUtil.log("driveWhileShooting TIMED OUT");
            return false;
        }
    }

    public void goalSideV2(boolean useArms) {
        double nextGoalDistance = 0;
        long startTime = System.currentTimeMillis();
        double savedDeclination;

/*
        // Prep Shooter
        nextGoalDistance = drive.getGoalDistance((int)B05_SHOOT1_X, (int)B05_SHOOT1_Y * (teamUtil.alliance== teamUtil.Alliance.RED ? -1 : 1));
        if (useArms) {
            //shooter.adjustShooterV2(goalDistance);
            shooter.setShootSpeed(B05_SHOT1_VEL); // PID loop is taking a long time to spin up and stabilize at these lower speeds (720->812)
            shooter.VELOCITY_COMMANDED = B05_SHOT1_VEL;
        }
*/

        /////////////////////////////Shoot Preloads

        // Drive fast to shooting zone
        if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED,B05_SHOOT1_X, B05_SHOOT1_Y, B05_SHOOT1_H-180, B05_SHOOT1_H,B05_SHOOT1_END_VEL, null, 0, 2000)) return;
        // Shoot preloads
        if (!driveWhileShooting(useArms, true, teamUtil.alliance== teamUtil.Alliance.BLUE ? (B05_SHOOT1_H-180) : 360-B05_SHOOT1_H-180,B00_SHOOT_VELOCITY,3000)) return;

        /////////////////////////////Intake 2nd group and shoot
        // Setup to pickup group 2
        if (!drive.mirroredMoveToYHoldingLine(B00_MAX_SPEED, B06_SETUP1_Y,B06_SETUP1_X,B06_SETUP1_DH, B06_SETUP1_H, B06_SETUP_END_VEL, null, 0, 1500)) return;
        drive.stopMotors(); // help kill the sideways momentum
        teamUtil.pause(B06_SETUP1_PAUSE);
        // Pickup group 2
        if (useArms) { intake.intakeIn(); }
        if (!drive.mirroredMoveToXHoldingLine(B00_PICKUP_VELOCITY, B07_PICKUP1_X,B06_PICKUP1_Y,B07_PICKUP1_H, B06_SETUP1_H, B00_CORNER_VELOCITY, null, 0, 1500)) return;
        // Drive back to shooting zone
        if (!drive.mirroredMoveToYHoldingLine(B00_MAX_SPEED, B08_SHOOT2_Y+B08_SHOOT2_DRIFT,B08_SHOOT2_X,B08_SHOOT2_DH, B08_SHOOT2_H, B08_SHOOT2_END_VEL, null, 0, 2000)) return;
        // shoot second set of balls
        if (!driveWhileShooting(useArms, true, teamUtil.alliance== teamUtil.Alliance.BLUE ? (B08_SHOOT2_H) : 360-B08_SHOOT2_H,B00_SHOOT_VELOCITY,3000)) return;

        /////////////////////////////Intake 3rd group and shoot
        // Setup to pickup group 3
        if (!drive.mirroredMoveToYHoldingLine(B00_MAX_SPEED, B07_SETUP2_Y,B06_SETUP1_X-B01_TILE_LENGTH,B07_SETUP2_DH, B06_SETUP1_H, B06_SETUP_END_VEL, null, 0, 1500)) return;
        drive.stopMotors(); // help kill the sideways momentum
        teamUtil.pause(B06_SETUP1_PAUSE);
        // Pickup group 3
        if (useArms) { intake.intakeIn(); }
        if (!drive.mirroredMoveToXHoldingLine(B00_PICKUP_VELOCITY, B07_PICKUP1_X-A01_TILE_LENGTH,B06_PICKUP1_Y,B07_PICKUP1_H, B06_SETUP1_H, B00_CORNER_VELOCITY, null, 0, 3000)) return;
        // Drive back to shooting zone
        if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED, B08_SHOOT3_X-B08_SHOOT3_DRIFT,B08_SHOOT3_Y,B08_SHOOT3_DH, B08_SHOOT3_H, B08_SHOOT3_END_VEL, null, 0, 3000)) return;
        // shoot 3rd set of balls
        if (!driveWhileShooting(useArms, true, teamUtil.alliance== teamUtil.Alliance.BLUE ? (B08_SHOOT3_H) : 360-B08_SHOOT3_H,B00_SHOOT_VELOCITY,3000)) return;

        /////////////////////////////Intake 4th group and shoot
        // pickup group 4
        if (useArms) { intake.intakeIn(); }
        if (!drive.mirroredMoveToXHoldingLine(B00_PICKUP_VELOCITY, B07_PICKUP1_X-A01_TILE_LENGTH*2,B06_PICKUP1_Y,B07_PICKUP1_H, B06_SETUP1_H, B00_CORNER_VELOCITY, null, 0, 3000)) return;
        // Drive back to shooting zone
        if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED, B08_SHOOT4_X-B08_SHOOT4_DRIFT,B08_SHOOT4_Y,B08_SHOOT4_DH, B08_SHOOT4_H, B08_SHOOT4_END_VEL, null, 0, 4000)) return;
        // shoot 4th set of balls
        if (!driveWhileShooting(useArms, true, teamUtil.alliance== teamUtil.Alliance.BLUE ? (B08_SHOOT4_H) : 360-B08_SHOOT4_H,B00_SHOOT_VELOCITY,3000)) return;

        drive.stopMotors();
        if (true) return;

        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A05_SHOOT1_X+A05_DRIFT_1, A05_SHOOT1_Y+A05_DRIFT_1, A05_SHOOT1_H, A00_SHOOT_END_VELOCITY,null,0,false,3000);

        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        logShot(1, (int)A05_SHOOT1_X, (int)A05_SHOOT1_Y, (int) nextGoalDistance, A05_SHOOT1_H);

        if (useArms) {
            //shootPatternAuto();
            intake.intakeStart();
        } else {
            teamUtil.pause(2000);
        }

        //Collect Set 2
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A06_SETUP1_X, A06_SETUP1_Y, A06_SETUP1_H, A00_PICKUP_VELOCITY,null,0,false,3000);
        drive.straightHoldingStrafeEncoder(A00_PICKUP_VELOCITY, A07_PICKUP1_X, A07_PICKUP1_Y * (teamUtil.alliance== teamUtil.Alliance.RED ? -1 : 1), (int)A07_PICKUP1_H,A00_PICKUP_END_VELOCITY,false,null,0,2000);
        //drive.mirroredMoveTo(A00_PICKUP_VELOCITY, A07_PICKUP1_X, A07_PICKUP1_Y, A07_PICKUP1_H, A00_PICKUP_END_VELOCITY,null,0,false,3000);
        if (useArms) {
            intake.elevatorToFlippersV2NoWait();
        }
        // Prep Shooter
        nextGoalDistance = drive.getGoalDistance((int)A08_SHOOT1_X, (int)A08_SHOOT1_Y * (teamUtil.alliance== teamUtil.Alliance.RED ? -1 : 1));
        if (useArms) shooter.adjustShooterV2(nextGoalDistance);
        //Shoot Set 2
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A08_SHOOT1_X-A05_DRIFT_2, A08_SHOOT1_Y+A05_DRIFT_2, A08_SHOOT1_H, A01_SHOOT_END_VELOCITY,null,0,false,3000);
        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        logShot(2, (int)A08_SHOOT1_X, (int)A08_SHOOT1_Y, (int) nextGoalDistance, A08_SHOOT1_H);
        if (useArms) {
            //shootPatternAuto();
            intake.intakeStart();
        } else {
            teamUtil.pause(2000);
        }

        //Collect Set 3
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A06_SETUP1_X-A01_TILE_LENGTH, A07_SETUP1_Y, A06_SETUP1_H, A00_PICKUP_VELOCITY,null,0,false,3000);
        drive.straightHoldingStrafeEncoder(A00_PICKUP_VELOCITY, A07_PICKUP1_X-A01_TILE_LENGTH, A07_PICKUP1_Y * (teamUtil.alliance== teamUtil.Alliance.RED ? -1 : 1), (int)A07_PICKUP1_H,A00_PICKUP_END_VELOCITY,false,null,0,2000);
        //drive.mirroredMoveTo(A00_PICKUP_VELOCITY, A07_PICKUP1_X-A01_TILE_LENGTH, A07_PICKUP1_Y, A07_PICKUP1_H, A00_PICKUP_END_VELOCITY,null,0,false,3000);
        if (useArms) {
            intake.elevatorToFlippersV2NoWait();
        }
        // Prep Shooter
        nextGoalDistance = drive.getGoalDistance((int)A09_SHOOT1_X, (int)A09_SHOOT1_Y * (teamUtil.alliance== teamUtil.Alliance.RED ? -1 : 1));
        if (useArms) shooter.adjustShooterV2(nextGoalDistance);

        //Shoot Set 3
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A09_SHOOT1_X-A05_DRIFT_2, A09_SHOOT1_Y+A05_DRIFT_2, A09_SHOOT1_H, A01_SHOOT_END_VELOCITY,null,0,false,3000);

        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        logShot(3, (int)A09_SHOOT1_X, (int)A09_SHOOT1_Y, (int) nextGoalDistance, A09_SHOOT1_H);
        if (useArms) {
            //shootPatternAuto();
            intake.intakeStart();
        } else {
            teamUtil.pause(2000);
        }

        shooter.stopShooter();

        if (System.currentTimeMillis()-startTime > 30000-A99_GRAB_LAST_THREE_TIME) {
            if (System.currentTimeMillis()-startTime > 30000-A99_MOVE_OFF_LINE_TIME) {
                teamUtil.log("OUT OF TIME, shutting down");
                drive.stopMotors();
                intake.stopDetector();
                intake.intakeStop();
                shooter.stopShooter();
                return;
            } else {
                teamUtil.log("NO TIME FOR LAST GRAB. Moving off of line");
                intake.stopDetector();
                intake.intakeStop();
                shooter.stopShooter();
                drive.moveToX(.7f,0,teamUtil.alliance== teamUtil.Alliance.RED ? 225 : 135, 0);
                return;
            }
        }

        if (useArms) {
            intake.intakeStart();
        }

        //Pick up last 3 balls
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A06_SETUP1_X-(2*A01_TILE_LENGTH)+A07_END_PICKUP_X_ADJUSTMENT, A07_SETUP1_Y, A06_SETUP1_H, A00_FINAL_END_VELOCITY,null,0,false,3000);
        drive.straightHoldingStrafeEncoder(A00_PICKUP_VELOCITY, A07_PICKUP1_X-(2*A01_TILE_LENGTH), A07_PICKUP1_Y * (teamUtil.alliance== teamUtil.Alliance.RED ? -1 : 1), (int)A07_PICKUP1_H,A00_PICKUP_END_VELOCITY,false,null,0,2000);

        if (System.currentTimeMillis()-startTime > 30000-A99_PARK_TIME) {
            teamUtil.log("OUT OF TIME, not parking");
            drive.stopMotors();
            intake.stopDetector();
            intake.intakeStop();
            shooter.stopShooter();
            return;
        }

        // Park in front of gate
        drive.straightHoldingStrafeEncoder(A00_MAX_SPEED_NEAR_GOAL, A90_PARK_X, A07_PICKUP1_Y * (teamUtil.alliance== teamUtil.Alliance.RED ? -1 : 1), (int)A07_PICKUP1_H,A90_PARK_END_VELOCITY,false,null,0,2000);

        drive.stopMotors();
        intake.stopDetector();
        intake.intakeStop();
        shooter.stopShooter();

        if (true) return;
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

    public void goalSide(boolean useArms) {
        double goalDistance = 0;
        long startTime = System.currentTimeMillis();


        // Prep Shooter
        goalDistance = drive.getGoalDistance((int)A05_SHOOT1_X, (int)A05_SHOOT1_Y * (teamUtil.alliance== teamUtil.Alliance.RED ? -1 : 1));
        if (useArms) {
            //shooter.adjustShooterV2(goalDistance);
            shooter.setShootSpeed(A05_SHOT1_VEL); // PID loop is taking a long time to spin up and stabilize at these lower speeds (720->812)
            shooter.VELOCITY_COMMANDED = A05_SHOT1_VEL;
        }

        //Shoot Preloads
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A05_SHOOT1_X+A05_DRIFT_1, A05_SHOOT1_Y+A05_DRIFT_1, A05_SHOOT1_H, A00_SHOOT_END_VELOCITY,null,0,false,3000);

        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        logShot(1, (int)A05_SHOOT1_X, (int)A05_SHOOT1_Y, (int)goalDistance, A05_SHOOT1_H);

        if (useArms) {
            shootPatternAuto(teamUtil.Pattern.PPG);
            intake.intakeStart();
        } else {
            teamUtil.pause(2000);
        }

        //Collect Set 2
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A06_SETUP1_X, A06_SETUP1_Y, A06_SETUP1_H, A00_PICKUP_VELOCITY,null,0,false,3000);
        drive.straightHoldingStrafeEncoder(A00_PICKUP_VELOCITY, A07_PICKUP1_X, A07_PICKUP1_Y * (teamUtil.alliance== teamUtil.Alliance.RED ? -1 : 1), (int)A07_PICKUP1_H,A00_PICKUP_END_VELOCITY,false,null,0,2000);
        //drive.mirroredMoveTo(A00_PICKUP_VELOCITY, A07_PICKUP1_X, A07_PICKUP1_Y, A07_PICKUP1_H, A00_PICKUP_END_VELOCITY,null,0,false,3000);
        if (useArms) {
            intake.elevatorToFlippersV2NoWait();
        }
        // Prep Shooter
        goalDistance = drive.getGoalDistance((int)A08_SHOOT1_X, (int)A08_SHOOT1_Y * (teamUtil.alliance== teamUtil.Alliance.RED ? -1 : 1));
        if (useArms) shooter.adjustShooterV2(goalDistance);
        //Shoot Set 2
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A08_SHOOT1_X-A05_DRIFT_2, A08_SHOOT1_Y+A05_DRIFT_2, A08_SHOOT1_H, A01_SHOOT_END_VELOCITY,null,0,false,3000);
        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        logShot(2, (int)A08_SHOOT1_X, (int)A08_SHOOT1_Y, (int)goalDistance, A08_SHOOT1_H);
        if (useArms) {
            if(teamUtil.alliance == teamUtil.Alliance.BLUE) { // balls are reversed from audience
                shootPatternAuto(teamUtil.Pattern.GPP);
            }else{
                shootPatternAuto(teamUtil.Pattern.PPG);
            }
            intake.intakeStart();
        } else {
            teamUtil.pause(2000);
        }

        //Collect Set 3
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A06_SETUP1_X-A01_TILE_LENGTH, A07_SETUP1_Y, A06_SETUP1_H, A00_PICKUP_VELOCITY,null,0,false,3000);
        drive.straightHoldingStrafeEncoder(A00_PICKUP_VELOCITY, A07_PICKUP1_X-A01_TILE_LENGTH, A07_PICKUP1_Y * (teamUtil.alliance== teamUtil.Alliance.RED ? -1 : 1), (int)A07_PICKUP1_H,A00_PICKUP_END_VELOCITY,false,null,0,2000);
        //drive.mirroredMoveTo(A00_PICKUP_VELOCITY, A07_PICKUP1_X-A01_TILE_LENGTH, A07_PICKUP1_Y, A07_PICKUP1_H, A00_PICKUP_END_VELOCITY,null,0,false,3000);
        if (useArms) {
            intake.elevatorToFlippersV2NoWait();
        }
        // Prep Shooter
        goalDistance = drive.getGoalDistance((int)A09_SHOOT1_X, (int)A09_SHOOT1_Y * (teamUtil.alliance== teamUtil.Alliance.RED ? -1 : 1));
        if (useArms) shooter.adjustShooterV2(goalDistance);

        //Shoot Set 3
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A09_SHOOT1_X-A05_DRIFT_2, A09_SHOOT1_Y+A05_DRIFT_2, A09_SHOOT1_H, A01_SHOOT_END_VELOCITY,null,0,false,3000);

        drive.stopMotors();
        drive.waitForRobotToStop(1000);
        logShot(3, (int)A09_SHOOT1_X, (int)A09_SHOOT1_Y, (int)goalDistance, A09_SHOOT1_H);
        if (useArms) {
            shootPatternAuto(teamUtil.Pattern.PGP); // middle needs no check
            intake.intakeStart();
        } else {
            teamUtil.pause(2000);
        }

        shooter.stopShooter();

        if (System.currentTimeMillis()-startTime > 30000-A99_GRAB_LAST_THREE_TIME) {
            if (System.currentTimeMillis()-startTime > 30000-A99_MOVE_OFF_LINE_TIME) {
                teamUtil.log("OUT OF TIME, shutting down");
                drive.stopMotors();
                intake.stopDetector();
                intake.intakeStop();
                shooter.stopShooter();
                return;
            } else {
                teamUtil.log("NO TIME FOR LAST GRAB. Moving off of line");
                intake.stopDetector();
                intake.intakeStop();
                shooter.stopShooter();
                drive.moveToX(.7f,0,teamUtil.alliance== teamUtil.Alliance.RED ? 225 : 135, 0);
                return;
            }
        }

        if (useArms) {
            intake.intakeStart();
        }

        //Pick up last 3 balls
        drive.mirroredMoveTo(A00_MAX_SPEED_NEAR_GOAL, A06_SETUP1_X-(2*A01_TILE_LENGTH)+A07_END_PICKUP_X_ADJUSTMENT, A07_SETUP1_Y, A06_SETUP1_H, A00_FINAL_END_VELOCITY,null,0,false,3000);
        drive.straightHoldingStrafeEncoder(A00_PICKUP_VELOCITY, A07_PICKUP1_X-(2*A01_TILE_LENGTH), A07_PICKUP1_Y * (teamUtil.alliance== teamUtil.Alliance.RED ? -1 : 1), (int)A07_PICKUP1_H,A00_PICKUP_END_VELOCITY,false,null,0,2000);

        if (System.currentTimeMillis()-startTime > 30000-A99_PARK_TIME) {
            teamUtil.log("OUT OF TIME, not parking");
            drive.stopMotors();
            intake.stopDetector();
            intake.intakeStop();
            shooter.stopShooter();
            return;
        }

        // Park in front of gate
        drive.straightHoldingStrafeEncoder(A00_MAX_SPEED_NEAR_GOAL, A90_PARK_X, A07_PICKUP1_Y * (teamUtil.alliance== teamUtil.Alliance.RED ? -1 : 1), (int)A07_PICKUP1_H,A90_PARK_END_VELOCITY,false,null,0,2000);

        drive.stopMotors();
        intake.stopDetector();
        intake.intakeStop();
        shooter.stopShooter();

        if (true) return;
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Auto Park Code
    // This code assumes the robot is already at a specified heading (315 for BLUE, ??? for RED)
    public static int LIFT_AUTO_ALIGN_VELOCITY = 200;
    public static int LIFT_AUTO_ALIGN_BLUE_THRESHOLD = 2500;
    public static int LIFT_AUTO_ALIGN_RED_THRESHOLD = 2500;


    // Back up until robot sees a line then stop
    public boolean alignForLift() {
        drive.loop();
        double robotHeading = drive.adjustAngle(drive.getHeadingODO());
        teamUtil.log("alignForLift. Robot Heading: " + robotHeading );
        long timeOutTime;

        drive.movingAutonomously.set(true);
        drive.manualInterrupt.set(false);

        teamUtil.log("Moving to tape");
        timeOutTime = System.currentTimeMillis() + 3000;
        // Wait for robot to get over the tape
        while ( ( (teamUtil.alliance== teamUtil.Alliance.BLUE && footColorSensor.blue() < LIFT_AUTO_ALIGN_BLUE_THRESHOLD) ||
                (teamUtil.alliance== teamUtil.Alliance.RED && footColorSensor.red() < LIFT_AUTO_ALIGN_RED_THRESHOLD) )
                && teamUtil.keepGoing(timeOutTime) && !drive.manualInterrupt.get()) {
            drive.loop();
            drive.driveMotorsHeadingsFR(drive.getHeadingODO(), drive.getHeadingODO(), LIFT_AUTO_ALIGN_VELOCITY);
        }
        drive.stopMotors();
        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("TIME OUT ---------------AlignForLift Timed Out looking for  tape");
            drive.movingAutonomously.set(false);
            drive.manualInterrupt.set(false);
            return false;
        }
        drive.movingAutonomously.set(false);
        return true;
    }

    public void alignForLiftNoWait(){
        if (drive.movingAutonomously.get()) {
            teamUtil.log("WARNING: Attempt to launch thread to alignForLift while in an autonmous moving operation. Ignored");
            return;
        }
        teamUtil.log("Launching Thread to alignForLift");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                alignForLift();
            }
        });
        thread.start();
    }

    public static int LIFT_AUTO_ALIGN_RED_HEADING = 45;
    public static int LIFT_AUTO_ALIGN_RED_DRIVE_HEADING1 = 0;
    public static int LIFT_AUTO_ALIGN_RED_DRIVE_HEADING2 = 270;
    public static int LIFT_AUTO_ALIGN_BLUE_HEADING = 315;
    public static int LIFT_AUTO_ALIGN_HEADING_ERROR_THRESHOLD = 5;
    public static int LIFT_AUTO_ALIGN_BLUE_DRIVE_HEADING1 = 270;
    public static int LIFT_AUTO_ALIGN_BLUE_DRIVE_HEADING2 = 180;
    public static int LIFT_AUTO_ALIGN_BLUE_ON_THRESHOLD = 3500;
    public static int LIFT_AUTO_ALIGN_BLUE_OFF_THRESHOLD = 2000;
    public static int LIFT_AUTO_ALIGN_RED_ON_THRESHOLD = 3500;
    public static int LIFT_AUTO_ALIGN_RED_OFF_THRESHOLD = 2000;

    // Try to use both lines to align perfectly...TODO: Not really working yet
    public boolean alignForLiftV2() {
        drive.loop();
        double robotHeading = drive.adjustAngle(drive.getHeadingODO());
        teamUtil.log("alignForLift. Robot Heading: " + robotHeading );
        long timeOutTime;

        if (teamUtil.alliance== teamUtil.Alliance.BLUE && (Math.abs(robotHeading-LIFT_AUTO_ALIGN_BLUE_HEADING) > LIFT_AUTO_ALIGN_HEADING_ERROR_THRESHOLD) ||
            teamUtil.alliance== teamUtil.Alliance.RED && (Math.abs(robotHeading-LIFT_AUTO_ALIGN_RED_HEADING) > LIFT_AUTO_ALIGN_HEADING_ERROR_THRESHOLD))
        {
            teamUtil.log("BAD HEADING for AlignForLift. Ignoring");
            drive.movingAutonomously.set(false);
            drive.manualInterrupt.set(false);
            return false;
        }
        drive.movingAutonomously.set(true);
        drive.manualInterrupt.set(false);

        teamUtil.log("Moving to first tape");
        timeOutTime = System.currentTimeMillis() + 3000;
        // Wait for robot to get over the tape
        while ( ( (teamUtil.alliance== teamUtil.Alliance.BLUE && footColorSensor.blue() < LIFT_AUTO_ALIGN_BLUE_ON_THRESHOLD) ||
                  (teamUtil.alliance== teamUtil.Alliance.RED && footColorSensor.red() < LIFT_AUTO_ALIGN_RED_ON_THRESHOLD) )
                && teamUtil.keepGoing(timeOutTime) && !drive.manualInterrupt.get()) {
            drive.loop();
            if (teamUtil.alliance== teamUtil.Alliance.BLUE) {
                drive.driveMotorsHeadingsFR(LIFT_AUTO_ALIGN_BLUE_DRIVE_HEADING1, LIFT_AUTO_ALIGN_BLUE_HEADING, LIFT_AUTO_ALIGN_VELOCITY);
            } else {
                drive.driveMotorsHeadingsFR(LIFT_AUTO_ALIGN_RED_DRIVE_HEADING1, LIFT_AUTO_ALIGN_RED_HEADING, LIFT_AUTO_ALIGN_VELOCITY);
            }
        }
        drive.stopMotors();
        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("TIME OUT ---------------AlignForLift Timed Out looking for first tape");
            drive.movingAutonomously.set(false);
            drive.manualInterrupt.set(false);
            return false;
        }
        drive.waitForRobotToStop(1000);
        teamUtil.log("Moving to second tape");

        timeOutTime = System.currentTimeMillis() + 3000;
        while ( ( (teamUtil.alliance== teamUtil.Alliance.BLUE && footColorSensor.blue() > LIFT_AUTO_ALIGN_BLUE_OFF_THRESHOLD) ||
                (teamUtil.alliance== teamUtil.Alliance.RED && footColorSensor.red() > LIFT_AUTO_ALIGN_RED_OFF_THRESHOLD) )
                && teamUtil.keepGoing(timeOutTime) && !drive.manualInterrupt.get()) {
            // Wait for robot to move off the end of the tape
            drive.loop();
            if (teamUtil.alliance== teamUtil.Alliance.BLUE) {
                drive.driveMotorsHeadingsFR(LIFT_AUTO_ALIGN_BLUE_DRIVE_HEADING2, LIFT_AUTO_ALIGN_BLUE_HEADING, LIFT_AUTO_ALIGN_VELOCITY);
            } else {
                drive.driveMotorsHeadingsFR(LIFT_AUTO_ALIGN_RED_DRIVE_HEADING2, LIFT_AUTO_ALIGN_RED_HEADING, LIFT_AUTO_ALIGN_VELOCITY);
            }
        }
        drive.stopMotors();
        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("TIME OUT ---------------AlignForLift Timed Out looking for second tape");
            drive.movingAutonomously.set(false);
            drive.manualInterrupt.set(false);
            return false;
        }
        return true;
    }

}

