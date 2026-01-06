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
import java.util.concurrent.atomic.AtomicBoolean;

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
    public static double FOOT_CALIBRATE_POS = .0;
    public static double FOOT_EXTENDED_POS = .6; // 1 second

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
        //setupLoadMap(); // obsolete
    }

    public void initialize(boolean useLimeLight) {
        drive.initialize();
        intake.initialize();
        shooter.initialize();
        blinkin.init();
        footColorSensor = hardwareMap.get(RevColorSensorV3.class, "floorcolorsensor");

        if (useLimeLight) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(PIPELINE_IDLE); // minimize CPU on the LL
            limelight.setPollRateHz(60);
        }
    }

    private static boolean LIME_LIGHT_ACTIVE;
    public static int PIPELINE_IDLE = 0;
    public static int PIPELINE_VIEW = 1;
    public static int PIPELINE_INTAKE = 9;
    public static int PIPELINE_INTAKE_POLLRATE = 60;

    public boolean limeLightActive() {
        return LIME_LIGHT_ACTIVE;
    }
    public boolean startLimeLightPipeline(int pipeline) {
        teamUtil.log("startLimeLightPipeline with pipeline: " + pipeline);
        if (limelight == null) {
            teamUtil.log("ERROR: Attempt to switch pipeline without initializing Limelight" + pipeline);
            return false;
        }
        boolean pipelineSwitch = limelight.pipelineSwitch(pipeline); // minimize CPU on the LL
        if(!pipelineSwitch){
            teamUtil.log("ERROR: Pipeline Switch Failed in StartLimeLightPipeline");
            return false;
        }

        if (!LIME_LIGHT_ACTIVE) {
            limelight.setPollRateHz(PIPELINE_INTAKE_POLLRATE);
            limelight.start();
        }
        LIME_LIGHT_ACTIVE = true;
        return true;
    }
    public boolean stopLimeLight() {
        teamUtil.log("stopLimeLight");
        if (limelight == null) {
            teamUtil.log("ERROR: Attempt to stop Limelight without initializing Limelight");
            return false;
        }
        if (!LIME_LIGHT_ACTIVE) {
            teamUtil.log("WARNING: stopLimeLight called while Limelight is already stopped");
        }
        boolean pipelineSwitch = limelight.pipelineSwitch(PIPELINE_IDLE); // minimize CPU on the LL
        if(!pipelineSwitch){
            teamUtil.log("ERROR: Pipeline Switch To IDLE Failed in StopLimeLight");
            return false;
        }
        limelight.pause();
        LIME_LIGHT_ACTIVE = false;
        return true;
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

    public static double GoalSizeThreshold = 254; //10 inches in millimeters

    public boolean shooterHeadingReady() {
        double distToCornerY = teamUtil.alliance == teamUtil.Alliance.BLUE ? Math.abs(BasicDrive.RED_ALLIANCE_WALL-drive.oQlocalizer.posY_mm):Math.abs(BasicDrive.BLUE_ALLIANCE_WALL-drive.oQlocalizer.posY_mm);
        double distToCornerX = BasicDrive.SCORE_X - drive.oQlocalizer.posX_mm;
        double headingCanShootThreshold = 90-Math.toDegrees(Math.atan((distToCornerX-GoalSizeThreshold)/distToCornerY))-Math.toDegrees(Math.atan((distToCornerY-GoalSizeThreshold)/distToCornerX));
        //TODO: Take out details
        if(details){
            teamUtil.log("Heading can shoot threshold: " + headingCanShootThreshold + " Current Declination: " + Math.abs(drive.getHeadingODO() - drive.robotGoalHeading()) + " DistX : " + distToCornerX + " DistY: " + distToCornerY);
        }
        return Math.abs(drive.getHeadingODO() - drive.robotGoalHeading()) < (headingCanShootThreshold/2);
    }


    public boolean canShoot(){
        return shooterFlyWheelsReady() && shooterHeadingReady();
    }

    public static long FIRST_UNLOAD_PAUSE = 400;
    public static long SECOND_UNLOAD_PAUSE = 600;
    public static double TWO_BALL_EDGE_PORTION = 1f/4;



    public static long EDGE_PUSHER_PAUSE = 700;

//    public void shootArtifactColor(Intake.ARTIFACT color){
//        teamUtil.log("shootArtifactColor called");
//        intake.detectLoadedArtifacts();
//        int ballCount = intake.loadedBallNum();
//        Intake.ARTIFACT[] loadedArtifacts = {intake.leftLoad, intake.middleLoad, intake.rightLoad};
//        if(color == Intake.ARTIFACT.NONE){
//            teamUtil.log("shootArtifactColor called with ARTIFACT.NONE");
//            return;
//        }
//        if(ballCount == 0) {
//            teamUtil.log("shootArtifactColor called without any artifacts loaded");
//            return;
//        }
//        if(loadedArtifacts[0] != color && loadedArtifacts[1] != color && loadedArtifacts[2] != color){
//            teamUtil.log("shootArtifactColor called without a loaded artifact of the specified color");
//            return;
//        }
//        if(color == loadedArtifacts[1]){
//            intake.middle_flipper.setPosition(Intake.MIDDLE_FLIPPER_SHOOTER_TRANSFER);
//            teamUtil.pause(FIRST_UNLOAD_PAUSE);
//            intake.middle_flipper.setPosition(Intake.FLIPPER_CEILING);
//            shooter.pushOne();
//            teamUtil.log("shootArtifactColor: Moved middle flipper");
//        }else if(color == loadedArtifacts[0]){
//            intake.left_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
//            teamUtil.pause(FIRST_UNLOAD_PAUSE);
//            intake.left_flipper.setPosition(Intake.FLIPPER_CEILING);
//            teamUtil.pause(EDGE_PUSHER_PAUSE);
//            shooter.pushOne();
//            teamUtil.log("shootArtifactColor: Moved left flipper");
//        }else{
//            intake.right_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
//            teamUtil.pause(FIRST_UNLOAD_PAUSE);
//            intake.right_flipper.setPosition(Intake.FLIPPER_CEILING);
//            teamUtil.pause(EDGE_PUSHER_PAUSE);
//            shooter.pushOne();
//            teamUtil.log("shootArtifactColor: Moved right flipper");
//        }
//        if(ballCount == 1){
//            intake.intakeStart();
//        }
//
//    }
//    public void shootArtifactColorNoWait(Intake.ARTIFACT color){
//        teamUtil.log("Launching Thread to shootArtifactColorNoWait");
//        Thread thread = new Thread(new Runnable() {
//            @Override
//            public void run() {
//                shootArtifactColor(color);
//            }
//        });
//        thread.start();
//    }

    public void shootArtifactLocation(Intake.Location location){ //
        teamUtil.log("shootArtifactLocation: " + location);
        // consider adding checks?
        if(location == Intake.Location.CENTER){
            intake.middle_flipper.setPosition(Intake.MIDDLE_FLIPPER_SHOOTER_TRANSFER);
            teamUtil.pause(FIRST_UNLOAD_PAUSE);
            intake.middle_flipper.setPosition(Intake.FLIPPER_CEILING_MIDDLE);
            intake.middleLoad = Intake.ARTIFACT.NONE;
            intake.signalArtifacts();
            shooter.pushOne();
            teamUtil.log("shootArtifactLocation: Moved middle flipper");
        }else if(location == Intake.Location.LEFT){
            intake.left_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
            teamUtil.pause(FIRST_UNLOAD_PAUSE);
            intake.left_flipper.setPosition(Intake.FLIPPER_CEILING);
            intake.leftLoad = Intake.ARTIFACT.NONE;
            intake.signalArtifacts();
            teamUtil.pause(EDGE_PUSHER_PAUSE);
            shooter.pushOne();
            teamUtil.log("shootArtifactLocation: Moved left flipper");
        }else{ // right
            intake.right_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
            teamUtil.pause(FIRST_UNLOAD_PAUSE);
            intake.right_flipper.setPosition(Intake.FLIPPER_CEILING);
            intake.rightLoad = Intake.ARTIFACT.NONE;
            intake.signalArtifacts();
            teamUtil.pause(EDGE_PUSHER_PAUSE);
            shooter.pushOne();
            teamUtil.log("shootArtifactLocation: Moved right flipper");
        }
        int ball_num = intake.numBallsInFlippers();
        if(ball_num == 0){
            intake.intakeStart();
        }
    }

    public void shootArtifactLocationNoWait(Intake.Location location){ //
        teamUtil.log("Launching Thread to shootArtifactLocationNoWait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                shootArtifactLocation(location);
            }
        });
        thread.start();
    }

    public void resetFlippersAndPusher(long pause){
        intake.flippersToCeiling();
        shooter.pushOneBackwards(pause);
        shooter.pushOne();
    }

    public void resetFlippersAndPusherNoWait(long pause){
        teamUtil.log("Launching Thread to shootArtifactLocationNoWait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                resetFlippersAndPusher(pause);
            }
        });
        thread.start();
    }



    public int left = 1;
    public int middle = 2;
    public int right = 3;
    public Intake.ARTIFACT[] autoRemainingLoaded = new Intake.ARTIFACT[4];
    public Intake.Location[] shotOrder = new Intake.Location[4];

    public Intake.Location determineShotAutoPattern(Intake.ARTIFACT color) {
        if (autoRemainingLoaded[middle] == color) {
            autoRemainingLoaded[middle] = Intake.ARTIFACT.NONE;
            return Intake.Location.CENTER;
        } else if (autoRemainingLoaded[left] == color) {
            autoRemainingLoaded[left] = Intake.ARTIFACT.NONE;
            return Intake.Location.LEFT;
        } else if (autoRemainingLoaded[right] == color) {
            autoRemainingLoaded[right] = Intake.ARTIFACT.NONE;
            return Intake.Location.RIGHT;
        } else { // Just because we don't have the right color for this shot doesn't mean we don't have stuff loaded
            if (autoRemainingLoaded[middle] != Intake.ARTIFACT.NONE) {
                autoRemainingLoaded[middle] = Intake.ARTIFACT.NONE;
                return Intake.Location.CENTER;
            } else if (autoRemainingLoaded[left] != Intake.ARTIFACT.NONE) {
                autoRemainingLoaded[left] = Intake.ARTIFACT.NONE;
                return Intake.Location.LEFT;
            } else if (autoRemainingLoaded[right] != Intake.ARTIFACT.NONE) {
                autoRemainingLoaded[right] = Intake.ARTIFACT.NONE;
                return Intake.Location.RIGHT;
            } else {
                return Intake.Location.NONE;
            }
        }
    }

    public void determineShotOrderAutoPattern () {
        teamUtil.log("determineShotOrderAutoPattern with loads: " + Intake.leftLoad + "/" + Intake.middleLoad + "/" + Intake.rightLoad);
        autoRemainingLoaded[left] = Intake.leftLoad;
        autoRemainingLoaded[middle] = Intake.middleLoad;
        autoRemainingLoaded[right] = Intake.rightLoad;
        shotOrder[1] = determineShotAutoPattern(teamUtil.pattern == PPG || teamUtil.pattern == PGP ? Intake.ARTIFACT.PURPLE : Intake.ARTIFACT.GREEN);
        shotOrder[2] = determineShotAutoPattern(teamUtil.pattern == PPG || teamUtil.pattern == GPP ? Intake.ARTIFACT.PURPLE : Intake.ARTIFACT.GREEN);
        shotOrder[3] = determineShotAutoPattern(teamUtil.pattern == GPP || teamUtil.pattern == PGP ? Intake.ARTIFACT.PURPLE : Intake.ARTIFACT.GREEN);
    }

    // Loads the next Artifact into the shooter in a separate thread
    public boolean loadPatternShotNoWait(int shot) {
        if (shot > 3 || shotOrder[shot] == Intake.Location.NONE) {
            teamUtil.log ("Empty Flipper, not unloading");
            return false;
        }
        teamUtil.log("Launching Thread to loadPatternShotNoWait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                int nextShot = shot+1;
                while (nextShot < 4 && shotOrder[nextShot] == Intake.Location.NONE ) {
                    nextShot++;
                }
                intake.unloadFlipper(shotOrder[shot], nextShot < 4 ? shotOrder[nextShot] : Intake.Location.NONE);
                intake.signalArtifacts();
            }
        });
        thread.start();
        return true;
    }

    public void stopRobot(){
        drive.stopMotors();
        intake.intakeStop();
        shooter.stopShooter();
    }

    // Attempts to run elevator to flippers and then load the first pattern shot
    // If balls are missing, will move to the next. Its possible that nothing will be loaded if nothing was in the intake
    public AtomicBoolean transferring = new AtomicBoolean(false);
    public void autoTransferAndLoadV2 (long pause, long timeOut) {
        transferring.set(true);
        teamUtil.log("autoTransferAndLoadV2 with pause:  " + pause);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        teamUtil.pause(pause);
        if (intake.elevatorToFlippersV2(false)) {
            determineShotOrderAutoPattern(); // sets global shotOrder
            teamUtil.log("Shot Order: " + shotOrder[1] + "/"+ shotOrder[2] + "/"+ shotOrder[3]);
            intake.unloadFlipper(shotOrder[1], shotOrder[2]); // preload next if possible
            intake.signalArtifacts();
        } else { // intake failed in some way
            // TODO: Maybe pause then try again for some amount of time?
            teamUtil.log("ElevatorToFlippersV2 failed. Giving up on Transfer.");
        }
        transferring.set(false);
        teamUtil.log("autoTransferAndLoadV2 Finished");
    }

    public void autoTransferAndLoadNoWait (long pause, long timeOut) {
        if (transferring.get()) {
            teamUtil.log("WARNING: Attempt to autoTransferAndLoadNoWait while transferring. Ignored.");
            return;
        }
        transferring.set(true);
        teamUtil.log("Launching Thread to autoTransferAndLoadNoWait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                autoTransferAndLoadV2(pause, timeOut);
            }
        });
        thread.start();
    }

    public void logShot(double flyWheelVelocity) {
        drive.loop();
        double goalDistance = drive.robotGoalDistance();
        double midSpeed = shooter.getVelocityNeeded(goalDistance);
        teamUtil.log("----------------------------------- SHOT " );
        teamUtil.log("------------ Robot X: " + drive.oQlocalizer.posX_mm + " Y: " + drive.oQlocalizer.posY_mm + " Goal Distance: " + goalDistance);
        teamUtil.log("------------ Target Heading: " + drive.robotGoalHeading() + " Actual Heading: " + drive.getHeadingODO() + " Diff: " + Math.abs(drive.robotGoalHeading()-drive.getHeadingODO()));
        teamUtil.log("------------ Actual Flywheel: " + flyWheelVelocity + " Aimer Pitch: " + shooter.currentAim());
        teamUtil.log("------------ Ideal Flywheel: " + midSpeed + " Aimer Pitch: " + shooter.calculatePitch(goalDistance, midSpeed));
    }

    // Checks a number of conditions to make sure we can shoot and then launches if possible
    // Returns true if it did shoot, false otherwise
    public boolean shootIfCan(){
        // Don't attempt to shoot if we are currently shooting
        if (shooter.pusher.moving.get()) return false;

        // Don't attempt to shoot if our heading is not accurate enough
        if(!shooterHeadingReady()) return false;

        // Don't attempt to shoot if there is no ball in the shooter
        if(!shooter.isLoaded()){
            return false;
        }

        double goalDistance = drive.robotGoalDistance();
        double flyWheelVelocity = shooter.leftFlywheel.getVelocity();

        // Don't attempt to shoot if flywheel speed is not in acceptable range
        if (!shooter.flywheelSpeedOK(goalDistance, flyWheelVelocity)) return false;

        // Adjust the pitch of the shooter to match distance and flywheel velocity
        shooter.changeAim(goalDistance, flyWheelVelocity);

        // Launch it
        shooter.pushOneNoWait();
        logShot(flyWheelVelocity);
        return true;
    }

    public static short SHOOT_VELOCITY_THRESHOLD = 1000; // mm/s

    public boolean shootIfCanTeleop(){
        boolean details = false;
        // Don't shoot if the robot is moving too fast
        if(Math.sqrt(Math.pow(drive.oQlocalizer.velX_mmS, 2) + Math.pow(drive.oQlocalizer.velY_mmS, 2)) > SHOOT_VELOCITY_THRESHOLD){
            if(details){
                teamUtil.log("Shootifcan fail: Robot moving too fast too shoot");
            }
            return false;
        }
        // Don't attempt to shoot if we are currently shooting
        if (shooter.pusher.moving.get()){
            if(details){
                teamUtil.log("Shootifcan fail: Robot currently shooting");
            }
            return false;
        }

        // Don't attempt to shoot if our heading is not accurate enough
        if(!shooterHeadingReady()){
            if(details){
                teamUtil.log("Shootifcan fail: Heading not accurate enough to shoot");
            }
            return false;
        }

        // Don't attempt to shoot if there is no ball in the shooter
        if(!shooter.isLoaded()){
            if(details){
                teamUtil.log("Shootifcan fail: No ball in shooter");
            }
            return false;
        }

        double goalDistance = drive.robotGoalDistance();
        double flyWheelVelocity = shooter.leftFlywheel.getVelocity();

        // Don't attempt to shoot if flywheel speed is not in acceptable range
        if (!shooter.flywheelSpeedOK(goalDistance, flyWheelVelocity)){
            if(details){
                teamUtil.log("Shootifcan fail: Flywheel Speed not in acceptable range");
            }
            return false;
        }
        // Launch it
        shooter.pushOneNoWait();
        logShot(flyWheelVelocity);
        teamUtil.log("Old Optimal Shooter Velocity: " + shooter.getVelocityNeeded(goalDistance));
        //0 balls in the flippers
        if(intake.numBallsInFlippers()==0){
            intake.intakeStart();
        }
        intake.flipNextFastNoWait();
        return true;
    }

    public static long AUTO_PATTERN_SHOT_LOAD_LIMIT = 1500; // Skip shot if it takes longer than this to load it into shooter
    // Move while shooting adjusting robot heading and shooter as needed
    // TODO: Shooter should have something loaded before this starts. Bail out if not true?
    public boolean driveWhileShootingPattern(boolean useArms, double driveHeading, double velocity, long timeOut) {
        teamUtil.log("driveWhileShootingPattern driveH: " + driveHeading + " Vel: " + velocity);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        long nextShotTimeLimit = System.currentTimeMillis() + AUTO_PATTERN_SHOT_LOAD_LIMIT;
        long shot3Time =  2100;
        boolean loadingNextShot = false;
        int numShots = 0;

        blinkin.setSignal(Blinkin.Signals.GOLD);
        while (teamUtil.keepGoing(timeOutTime) && numShots < 3) {
            drive.loop();
            double shotHeading = drive.robotGoalHeading();
            drive.driveMotorsHeadingsFR(driveHeading, shotHeading, velocity);
            if (useArms) {
                // TODO: Should we make sure a minimum amount of time has passed since last shot to make sure they hit ramp in the correct order?
                if (shootIfCan()) {
                    velocity = 0; // stop driving once we have a good shot (but keep rotating!)
                    nextShotTimeLimit = System.currentTimeMillis() + AUTO_PATTERN_SHOT_LOAD_LIMIT; // reset load timer
                    numShots++;

                    if (numShots < 3) {
                        while (numShots < 3) {
                            if (loadPatternShotNoWait(numShots+1)) { // something to load
                                break;
                            }
                            numShots++; // skip the last load (nothing in flipper
                            teamUtil.log ("Nothing to load--Skipping Shot " + numShots);
                        }
                    }
                } else if (System.currentTimeMillis() > nextShotTimeLimit) { // Taking too long to load for some unknown reason
                    nextShotTimeLimit = System.currentTimeMillis() + AUTO_PATTERN_SHOT_LOAD_LIMIT; // reset load timer
                    numShots++; // Move on to the next shot
                    teamUtil.log("------- Took Too long to load shot " + numShots + ". Moving on to next");
                    if (numShots < 3) {
                        while (numShots < 3) {
                            if (loadPatternShotNoWait(numShots+1)) { // something to load
                                break;
                            }
                            numShots++; // skip the last load (nothing in flipper
                            teamUtil.log ("Nothing to load--Skipping Shot " + numShots);
                        }
                    }
                }

            } else if (System.currentTimeMillis() > shot3Time) {
                    break;
            }
        }
        // Empty out shooter in case something got left behind. Not worried about aiming at this point.
        if (useArms){
            while (shooter.isLoaded() && !shooter.pusher.moving.get() && teamUtil.keepGoing(timeOutTime)) {
                teamUtil.log("driveWhileShootingPattern --------------- Leftovers in shooter! Emptying");
                shooter.pushOneNoWait();
                logShot(shooter.leftFlywheel.getVelocity());
            }
        }
        blinkin.setSignal(Blinkin.Signals.OFF);
        if (System.currentTimeMillis() <= timeOutTime) {
            // Wait for last shot to finish before moving TODO: This could wrap up a bit earlier...as soon as pusher connects the ball with the flywheels
            while (shooter.pusher.moving.get() && teamUtil.keepGoing(timeOutTime)) {
                teamUtil.pause(25);
            }
            teamUtil.log("driveWhileShootingPattern Finished");
            return true;
        } else {
            teamUtil.log("driveWhileShootingPattern TIMED OUT");
            return false;
        }
    }

    public void autoShootFastPreload() {
        intake.flipNextFastNoWait();
    }

    // Assumes robot is in position, 3 balls were in flippers and flipNextFast has already been called once.
    // stops motors and fires all 3 shots as quickly as possible
    public boolean autoShootFast(boolean useArms, long timeOut) {
        long timeOutTime = System.currentTimeMillis() + timeOut;
        long shot3Time =  2100;
        int numshots = 0;
        int totalShots = intake.numBallsInFlippers() + 1; // previous call to flipNextFast unloaded one
        teamUtil.log("autoShootFast. Total Planned Shots: " + totalShots);

        blinkin.setSignal(Blinkin.Signals.GOLD);

        while (teamUtil.keepGoing(timeOutTime) && numshots < totalShots) {
            drive.loop();
            double shotHeading = drive.robotGoalHeading();
            drive.driveMotorsHeadingsFR(shotHeading, shotHeading, 0); // continue to rotate to match shot heading
            if (useArms) {
                if (shootIfCan()) { // try to take a shot asap
                    numshots++;
                    if (numshots < totalShots) {
                        // TODO: Do we need a pause here to make sure previous shot (still sitting in shooter) doesn't trigger flipNextFast to think it is done?
                        intake.flipNextFastNoWait(); // load next when the shot happens
                    }
                    intake.signalArtifacts();
                }
            } else if (System.currentTimeMillis() > shot3Time) {
                break;
            }
        }
        // FAILSAFE: Empty out shooter in case something got left behind. Not worried about aiming at this point.
        while (shooter.isLoaded() && !shooter.pusher.moving.get() && teamUtil.keepGoing(timeOutTime)) {
            teamUtil.log("autoShootFast --------------- Leftovers in shooter! Emptying");
            shooter.pushOneNoWait();
            logShot(shooter.leftFlywheel.getVelocity());
        }

        blinkin.setSignal(Blinkin.Signals.OFF);
        if (System.currentTimeMillis() <= timeOutTime) {
            // Wait for last shot to finish before moving TODO: This could wrap up a bit earlier...as soon as pusher connects the ball with the flywheels
            while (shooter.pusher.moving.get() && teamUtil.keepGoing(timeOutTime)) {
                teamUtil.pause(25);
            }
            teamUtil.log("autoShootFast Finished");
            return true;
        } else {
            teamUtil.log("autoShootFast TIMED OUT");
            return false;
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Main Auto Code

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

    public static double B00_MAX_SPEED = 2200;
    public static double B00_CORNER_VELOCITY = 1800;
    public static double B00_SHOOT_VELOCITY = 0;
    public static double B00_PICKUP_VELOCITY = 2000;
    public static double B00_PICKUP_END_VELOCITY = 2000;
    public static double B01_TILE_LENGTH = 610;

    public static double B05_DRIFT_1 = 50;
    public static double B05_DRIFT_2 = 140;

    public static double B05_SHOOT1_END_VEL = 400;
    public static double B05_SHOOT1_Y = 850;
    public static double B05_SHOOT1_X = 850;
    public static double B05_SHOOT1_H = 45;
    public static double B05_SHOT1_VEL = 820;

    public static double B06_PICKUP1_Y = 1200;
    public static double B06_SETUP_Y_DRIFT = 200;
    public static double B06_SETUP1_Y = B06_PICKUP1_Y-B06_SETUP_Y_DRIFT;
    public static double B06_SETUP1_X = 670;
    public static double B06_SETUP1_DH = 90;
    public static double B06_SETUP1_H = 0;
    public static double B06_SETUP_END_VEL = B00_CORNER_VELOCITY;
    public static long B06_SETUP1_PAUSE = 150;
    public static double B06_SHOT34_VELOCITY = 840;


    public static double B07_PICKUP1_X = 420;
    public static double B07_PICKUP_RAMP_END_VEL = 750;
    public static double B07_PICKUP1_H = 180;
    public static double B07_RAMP_VELOCITY = 600;
    public static double B07_RAMP_Y = B06_PICKUP1_Y+190;
    public static double B07_RAMP_X = 200;
    public static double B07_RAMP_X_DRIFT = 100;
    public static double B07_RAMP_H = 90;
    public static double B07_RAMP_FY = B06_PICKUP1_Y;
    public static double B07_RAMP_FH = 270;
    public static long B07_RAMP_TIMEOUT = 1000;
    public static long B07_PICKUP2_INTAKE_PAUSE = 500;
    public static long B07_PICKUP3_INTAKE_PAUSE = 0;
    public static long B07_PICKUP4_INTAKE_PAUSE = 0;

    public static double B07_END_PICKUP_X_ADJUSTMENT = 100;
    public static double B07_SETUP2_DH = 105;
    public static double B07_SETUP_Y_DRIFT = 100;
    public static double B07_SETUP2_Y = B06_PICKUP1_Y-B07_SETUP_Y_DRIFT;
    public static double B07_SETUP2_X = B06_SETUP1_X-B01_TILE_LENGTH+50;


    public static double B08_SHOOT2_Y = 650;
    public static double B08_SHOOT2_DRIFT = 200;
    public static double B08_SHOOT2_X = 650;
    public static double B08_SHOOT2_H = 45;
    public static double B08_SHOOT2_DH = 315;
    public static double B08_SHOOT2_END_VEL = 400;

    public static long B08_PICKUP3_PAUSE = 150;
    public static long B08_PICKUP4_PAUSE = 200;

    public static double B08_SHOOT3_Y = B08_SHOOT2_Y;
    public static double B08_SHOOT3_DRIFT = 100;
    public static double B08_SHOOT3_X = B08_SHOOT2_X;
    public static double B08_SHOOT3_H = B08_SHOOT2_H;
    public static double B08_SHOOT3_DH = 340;
    public static double B08_SHOOT3_END_VEL = 400;

    public static double B08_SHOOT4_Y = B08_SHOOT3_Y;
    public static double B08_SHOOT4_DRIFT = 100;
    public static double B08_SHOOT4_X = B08_SHOOT3_X;
    public static double B08_SHOOT4_H = B08_SHOOT3_H;
    public static double B08_SHOOT4_DH = 350;
    public static double B08_SHOOT4_END_VEL = 400;

    public static double B99_GRAB_LAST_THREE_TIME = 4000;
    public static double B99_PARK_TIME = 1500;
    public static double B99_MOVE_OFF_LINE_TIME = 1500;

    public static double B08_MORE_BALLS_THRESHOLD = 6000;


    public static boolean emptyRamp = true;
    public static int emptyRampPause = 2000;
    public static long gateElapsedTime = 0;

    public void goalSideV2(boolean useArms, boolean useIntakeDetector, long gateLeaveTime, boolean getMore) {
        double nextGoalDistance = 0;
        long startTime = System.currentTimeMillis();
        double savedDeclination;
        teamUtil.log("##################################################################################");

        teamUtil.log("#########################  Starting GoalSideV2 Auto ##############################");
        shooter.flywheelStartup(); // set flywheel to fast start PIDF coefs
        intake.setLoadedArtifacts(PPG); // Assumes artifacts are preloaded in this order!!
        //intake.setIntakeArtifacts(PPG);
        // Prep Shooter
        nextGoalDistance = drive.getGoalDistance((int)B05_SHOOT1_X, (int)B05_SHOOT1_Y * (teamUtil.alliance== teamUtil.Alliance.RED ? -1 : 1));
        if (useArms) {
            shooter.setShootSpeed(B05_SHOT1_VEL); // TODO: Determine optimal speed for first 3 shots
            Shooter.VELOCITY_COMMANDED = B05_SHOT1_VEL;
            autoShootFastPreload(); // go fast on preloads--don't bother with pattern

            /*
            teamUtil.Pattern stored = teamUtil.pattern;
            teamUtil.pattern = PPG; // fake out Shot Order to ensure relatively fast shots on preloads
            determineShotOrderAutoPattern(); // sets up the data for loadPattern
            teamUtil.pattern = stored;
            loadPatternShotNoWait(1); // get the first ARTIFACT in the shooter
             */
        }
        if (useIntakeDetector) {
            if (intake.startDetector()) {
                intake.detectorMode = Intake.DETECTION_MODE.INTAKE; // start in intake mode
                teamUtil.log("Started Intake Detector");
            } else {
                useIntakeDetector = false;
                teamUtil.log("FAILED to Start Intake Detector, failing over to hardcoded mode instead");
            }
        } else {
            teamUtil.log("NOT using Detector. Running in hardcoded mode for patterns.");
        }


        /////////////////////////////Shoot Preloads (Group 1)
        teamUtil.log("==================== Preloads ================");
        // Drive fast to shooting zone
        if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED,B05_SHOOT1_X, B05_SHOOT1_Y, B05_SHOOT1_H-180, B05_SHOOT1_H,B05_SHOOT1_END_VEL, null, 0, 2000)) return;
        // Shoot preloads
        intake.signalArtifacts(); // flippers were operating in another thread while we were moving to this point.
        //shooter.flywheelNormal(); // set flywheel to normal PIDF coefs
        if (!autoShootFast(useArms,5000)) return; // Don't bother with pattern on preloads since we are going to empty the ramp
        //if (!driveWhileShootingPattern(useArms, teamUtil.alliance== teamUtil.Alliance.BLUE ? (B05_SHOOT1_H-180) : 360-B05_SHOOT1_H-180,B00_SHOOT_VELOCITY,5000)) return;


        /////////////////////////////Intake 2nd group and shoot
        // Setup to pickup group 2
        teamUtil.log("==================== Group 2 ================");
        if (useArms) { intake.getReadyToIntakeNoWait(); }
        if (!drive.mirroredMoveToYHoldingLine(B00_MAX_SPEED, B06_SETUP1_Y,B06_SETUP1_X,B06_SETUP1_DH, B06_SETUP1_H, B06_SETUP_END_VEL, null, 0, 1500)) return;
        drive.stopMotors(); // help kill the sideways momentum
        teamUtil.pause(B06_SETUP1_PAUSE);
        // Pickup group 2
        if (emptyRamp) {
            teamUtil.log("==================== Empty Ramp ");
            // Pickup 2nd set of artifacts, slowing at end
            if (!drive.mirroredMoveToXHoldingLine(B00_PICKUP_VELOCITY, B07_RAMP_X + B07_RAMP_X_DRIFT,B06_PICKUP1_Y,B07_PICKUP1_H, B06_SETUP1_H, B07_PICKUP_RAMP_END_VEL, null, 0, 1500)) return;
            if (useIntakeDetector) {
                intake.detectIntakeArtifactsV2();
            } else {
                // Manually set what is loaded in intake in case detector fails
                if (teamUtil.alliance == teamUtil.Alliance.BLUE) { // balls are reversed from audience
                    intake.setIntakeArtifacts(GPP);
                    intake.setLoadedArtifacts(GPP); // Assumes artifacts are preloaded in this order!!

                } else {
                    intake.setIntakeArtifacts(PPG);
                    intake.setLoadedArtifacts(PPG); // Assumes artifacts are preloaded in this order!!
                }
            }
            intake.signalArtifacts();
            if (useArms) autoTransferAndLoadNoWait(B07_PICKUP2_INTAKE_PAUSE, 3000); // TODO: Need to test for failure here and do something smart to avoid massive penalities
            // push the gate allowing for timeout
            drive.mirroredMoveToYHoldingLine(B07_RAMP_VELOCITY, B07_RAMP_Y, B07_RAMP_X, B07_RAMP_H, B06_SETUP1_H, 0, null, 0, B07_RAMP_TIMEOUT);
            drive.stopMotors();
            teamUtil.pause(emptyRampPause);
            long pause = gateLeaveTime - (System.currentTimeMillis() - startTime);
            teamUtil.pause(pause);
            // get clear of 3rd group before rotating
            if (!drive.mirroredMoveToYHoldingLine(B00_MAX_SPEED, B07_RAMP_FY, B07_RAMP_X + B07_RAMP_X_DRIFT, B07_RAMP_FH, B06_SETUP1_H, B00_CORNER_VELOCITY, null, 0, 1500)) return;
        } else {
            // pickup 2nd set of artifacts
            if (!drive.mirroredMoveToXHoldingLine(B00_PICKUP_VELOCITY, B07_PICKUP1_X,B06_PICKUP1_Y,B07_PICKUP1_H, B06_SETUP1_H, B00_CORNER_VELOCITY, null, 0, 1500)) return;
            if (useArms) autoTransferAndLoadNoWait(B07_PICKUP2_INTAKE_PAUSE, 3000);
        }
        // TODO: Adjust flywheel speed for 2nd 3 shots
        // Drive back to shooting zone
        if (!drive.mirroredMoveToYHoldingLine(B00_MAX_SPEED, B08_SHOOT2_Y+B08_SHOOT2_DRIFT,B08_SHOOT2_X,B08_SHOOT2_DH, B08_SHOOT2_H, B08_SHOOT2_END_VEL, null, 0, 2000)) return;
        // shoot second set of balls
        if (!driveWhileShootingPattern(useArms, teamUtil.alliance== teamUtil.Alliance.BLUE ? (B08_SHOOT2_H) : 360-B08_SHOOT2_H,B00_SHOOT_VELOCITY,5000)) return;
        if(intake.failedOut.get()){
            teamUtil.log("Auto has FAILED OUT because of a jammed intake");
            stopRobot();
            return;
        }

        /////////////////////////////Intake 3rd group and shoot
        shooter.setShootSpeed(B06_SHOT34_VELOCITY); // TODO: Determine optimal speed for first 3 shots
        Shooter.VELOCITY_COMMANDED = B06_SHOT34_VELOCITY;
        // Setup to pickup group 3
        teamUtil.log("==================== Group 3 ================");
        if (useArms) { intake.getReadyToIntakeNoWait(); }
        if (!drive.mirroredMoveToYHoldingLine(B00_MAX_SPEED, B07_SETUP2_Y,B07_SETUP2_X,B07_SETUP2_DH, B06_SETUP1_H, B06_SETUP_END_VEL, null, 0, 1500)) return;
        drive.stopMotors(); // help kill the sideways momentum
        teamUtil.pause(B06_SETUP1_PAUSE);
        // Pickup group 3
        if (!drive.mirroredMoveToXHoldingLine(B00_PICKUP_VELOCITY, B07_PICKUP1_X-B01_TILE_LENGTH,B06_PICKUP1_Y,B07_PICKUP1_H, B06_SETUP1_H, B00_CORNER_VELOCITY, null, 0, 3000)) return;
        // Manually set what is loaded in intake in case detector fails
        if (useIntakeDetector) {
            intake.detectIntakeArtifactsV2();
        } else {
            intake.setIntakeArtifacts(PGP);
            intake.setLoadedArtifacts(PGP); // Assumes artifacts are preloaded in this order!!
        }
        intake.signalArtifacts();
        drive.stopMotors(); // kill some forward momentum
        teamUtil.pause(B08_PICKUP3_PAUSE);
        if (useArms) autoTransferAndLoadNoWait(B07_PICKUP3_INTAKE_PAUSE, 3000); // TODO: Need to test for failure here and do something smart to avoid massive penalities
        // TODO: Adjust flywheel speed for 3rd group

        // Drive back to shooting zone
        if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED, B08_SHOOT3_X-B08_SHOOT3_DRIFT,B08_SHOOT3_Y,B08_SHOOT3_DH, B08_SHOOT3_H, B08_SHOOT3_END_VEL, null, 0, 3000)) return;
        // shoot 3rd set of balls
        if (!driveWhileShootingPattern(useArms, teamUtil.alliance== teamUtil.Alliance.BLUE ? (B08_SHOOT3_H) : 360-B08_SHOOT3_H,B00_SHOOT_VELOCITY,5000)) return;
        if(intake.failedOut.get()){
            teamUtil.log("Auto has FAILED OUT because of a jammed intake");
            stopRobot();
            return;
        }

        /////////////////////////////Intake 4th group and shoot
        // pickup group 4
        teamUtil.log("==================== Group 4 ================");
        if (useArms) { intake.getReadyToIntakeNoWait(); }
        if (!drive.mirroredMoveToXHoldingLine(B00_PICKUP_VELOCITY, B07_PICKUP1_X-B01_TILE_LENGTH*2,B06_PICKUP1_Y,B07_PICKUP1_H, B06_SETUP1_H, B00_CORNER_VELOCITY, null, 0, 3000)) return;
        // Manually set what is loaded in intake in case detector fails
        if (useIntakeDetector) {
            intake.detectIntakeArtifactsV2();
        } else {
            if (teamUtil.alliance == teamUtil.Alliance.BLUE) { // balls are reversed from audience
                intake.setIntakeArtifacts(PPG);
                intake.setLoadedArtifacts(PPG); // Assumes artifacts are preloaded in this order!!

            } else {
                intake.setIntakeArtifacts(GPP);
                intake.setLoadedArtifacts(GPP); // Assumes artifacts are preloaded in this order!!
            }
        }
        intake.signalArtifacts();

        drive.stopMotors(); // kill some forward momentum
        teamUtil.pause(B08_PICKUP4_PAUSE);
        if (useArms) autoTransferAndLoadNoWait(B07_PICKUP4_INTAKE_PAUSE, 3000); // TODO: Need to test for failure here and do something smart to avoid massive penalities
        // TODO: Adjust flywheel speed for 4th group
        // Drive back to shooting zone
        if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED, B08_SHOOT4_X-B08_SHOOT4_DRIFT,B08_SHOOT4_Y,B08_SHOOT4_DH, B08_SHOOT4_H, B08_SHOOT4_END_VEL, null, 0, 4000)) return;
        // shoot 4th set of balls
        if (!driveWhileShootingPattern(useArms, teamUtil.alliance== teamUtil.Alliance.BLUE ? (B08_SHOOT4_H) : 360-B08_SHOOT4_H,B00_SHOOT_VELOCITY,5000)) return;


        /////////////////////////////Park
        boolean enoughTime = System.currentTimeMillis() - startTime < 30000 - B08_MORE_BALLS_THRESHOLD; // check that there is enough time and we want to get more
        teamUtil.log("==================== Park ================");
        shooter.stopShooter();
        if(getMore && enoughTime){
            getMoreBalls(); // grab stuff from the loading zone

            // park
            if (!drive.mirroredMoveToXHoldingLine(C03_PARK_VELOCITY, B07_RAMP_X - C03_PARK_DRIFT_X,B06_PICKUP1_Y,0, 0, B07_PICKUP_RAMP_END_VEL, null, 0, 2500)) return;
        }else {
            intake.intakeStop();
            intake.stopDetector();
            if (!drive.mirroredMoveToYHoldingLine(B00_MAX_SPEED, B06_SETUP1_Y, B06_SETUP1_X, B06_SETUP1_DH, B06_SETUP1_H, B06_SETUP_END_VEL, null, 0, 1500))
                return;
            drive.stopMotors(); // help kill the sideways momentum
            teamUtil.pause(B06_SETUP1_PAUSE);
            if (!drive.mirroredMoveToXHoldingLine(B00_PICKUP_VELOCITY, B07_RAMP_X + B07_RAMP_X_DRIFT, B06_PICKUP1_Y, B07_PICKUP1_H, B06_SETUP1_H, B07_PICKUP_RAMP_END_VEL, null, 0, 1500))
                return;
        }
        /////////////////////////////Wrap up
        stopRobot();
    }

    public static double C01_FAST_APPROACH_X = -400;
    public static double C01_FAST_APPROACH_Y = 1250;
    public static double C01_FAST_APPROACH_VELOCITY = B00_MAX_SPEED;
    public static double C01_FAST_APPROACH_DRIVE_HEADING = 145;
    public static double C01_FAST_APPROACH_ROBOT_HEADING = 180 + C01_FAST_APPROACH_DRIVE_HEADING;
    public static double C01_FAST_APPROACH_END_VELOCITY = B00_CORNER_VELOCITY;
    public static double C02_BALL_APPROACH_WALL_TARGET = 1650;
    public static double C02_BALL_APPROACH_X = -500;
    public static double C02_BALL_APPROACH_DRIVE_HEADING = 90;
    public static double C02_BALL_APPROACH_ROBOT_HEADING = 0;
    public static double C02_BALL_APPROACH_VELOCITY = 800;
    public static long C02_BALL_APPROACH_TIMEOUT = 1000;
    public static float C02_BALL_APPROACH_POWER = .25f;
    public static double C02_BALL_APPROACH_STALL_VEL = 10;
    public static double C01_FAST_APPROACH_Y_OFFSET = -140;
    public static double C01_FAST_APPROACH_X_OFFSET = 200;


    public static int C02_GRAB_VEL = 600;
    public static int C02_GRAB_X_LIMIT = -1430;
    public static double C02_GRAB_Y_WALL_OFFSET = 15;
    public static double C02_GRAB_INTAKE_POWER = .9;
    public static long C02_GRAB_TIME = 2000;
    public static double C03_PARK_DRIFT_X = 500;
    public static double C03_PARK_VELOCITY = B00_PICKUP_VELOCITY;

    public static double C04_FAST_APPROACH_VELOCITY = B00_MAX_SPEED;
    public static double C04_FAST_APPROACH_X = -900;
    public static double C04_FAST_APPROACH_Y = 1100;
    public static double C04_FAST_APPROACH_DRIVE_HEADING = 170;
    public static double C04_FAST_APPROACH_ROBOT_HEADING = 180 + C04_FAST_APPROACH_DRIVE_HEADING;
    public static double C04_FAST_APPROACH_END_VELOCITY = B00_CORNER_VELOCITY;

    public static int C04_GRAB_VEL = 1000;
    public static int C04_GRAB_Y_LIMIT = 1430;
    public static long C04_GRAB_TIME = 2000;
    public static float C04_GRAB_POWER = .3f;
    public static long C04_GRAB_PAUSE = 250;

    public boolean getMoreBallsV2(){
        // get Intake Ready
        double stored = Intake.INTAKE_IN_POWER;
        Intake.INTAKE_IN_POWER = C02_GRAB_INTAKE_POWER; // adjust intake speed for this operation
        intake.getReadyToIntakeNoWait();

        // Drive towards wall fast
        if (!drive.mirroredMoveToXHoldingLine(C04_FAST_APPROACH_VELOCITY, C04_FAST_APPROACH_X,C04_FAST_APPROACH_Y,C04_FAST_APPROACH_DRIVE_HEADING, C04_FAST_APPROACH_ROBOT_HEADING, C04_FAST_APPROACH_END_VELOCITY, null, 0, 2100)) return false;
        intake.intakeNum = 0; // don't return instantly from grab3
        if (!grab3V2(C04_GRAB_VEL, C04_GRAB_Y_LIMIT, C04_GRAB_TIME)) return false;

        Intake.INTAKE_IN_POWER = stored; // restore intake speed default
        return true;
    }

    // rolls straight at 90 or 270 trying to pickup 3 balls
    // returns when it has 3, runs out of time, or when it reaches a certain y threshold
    // assumes intake is on and intake detector is running
    public boolean grab3V2(int velocity, int yThreshold, long timeOut) {
        teamUtil.log("grab3V2");
        long timeOutTime = System.currentTimeMillis()+timeOut;
        double driveHeading = teamUtil.alliance== teamUtil.Alliance.BLUE ? 90 : 270;
        double robotHeading = teamUtil.alliance== teamUtil.Alliance.BLUE ? 270 : 90;
        while (teamUtil.keepGoing(timeOutTime) && intake.intakeNum < 3 && ( teamUtil.alliance== teamUtil.Alliance.BLUE ? drive.oQlocalizer.posY_mm < yThreshold : drive.oQlocalizer.posY_mm > -yThreshold)) {
            drive.loop();
            intake.detectIntakeArtifactsV2();
            intake.signalArtifacts();
            drive.driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
        }
        if (Math.abs( drive.oQlocalizer.posY_mm) >= yThreshold ) {
            teamUtil.log("grab3V2 Reached Y Threshold: " + drive.oQlocalizer.posY_mm);
        }
        drive.driveMotorsHeadingsFRPower(driveHeading, robotHeading, C04_GRAB_POWER);
        teamUtil.pause(C04_GRAB_PAUSE); // give a little more time for intake to do its thing while pushing/stalling towards the wall
        drive.stopMotors();
        intake.detectIntakeArtifactsV2();
        intake.signalArtifacts();
        if (System.currentTimeMillis() >= timeOutTime) {
            teamUtil.log("grab3V2 TIMED OUT.");
            intake.intakeStop();
            return false;
        }
        teamUtil.log("grab3V2 Finished with " + intake.intakeNum + " artifacts.");
        return true;
    }

    public boolean getMoreBalls(){
        // get Intake Ready
        double stored = Intake.INTAKE_IN_POWER;
        Intake.INTAKE_IN_POWER = C02_GRAB_INTAKE_POWER; // adjust intake speed for this operation
        intake.getReadyToIntakeNoWait();

        // Drive towards wall fast
        if (!drive.mirroredMoveToXHoldingLine(C01_FAST_APPROACH_VELOCITY, C01_FAST_APPROACH_X+C01_FAST_APPROACH_X_OFFSET,C01_FAST_APPROACH_Y+C01_FAST_APPROACH_Y_OFFSET,C01_FAST_APPROACH_DRIVE_HEADING, C01_FAST_APPROACH_ROBOT_HEADING, C01_FAST_APPROACH_END_VELOCITY, null, 0, 1500)) return false;
        if (!drive.mirroredMoveToXHoldingLine(C01_FAST_APPROACH_VELOCITY, C01_FAST_APPROACH_X,C01_FAST_APPROACH_Y,C01_FAST_APPROACH_DRIVE_HEADING, 0, C01_FAST_APPROACH_END_VELOCITY, null, 0, 1500)) return false;
        // spin to final heading and snug up against wall using stall detection. shouldn't time out but OK if it does
        drive.mirroredStallY(C02_BALL_APPROACH_POWER, C02_BALL_APPROACH_DRIVE_HEADING, C02_BALL_APPROACH_ROBOT_HEADING, C02_BALL_APPROACH_STALL_VEL, C02_BALL_APPROACH_TIMEOUT);
        // spin to final heading and snug up against wall (intended to time out) (before stall detection)
        //drive.mirroredMoveToYHoldingLine(C02_BALL_APPROACH_VELOCITY, C02_BALL_APPROACH_WALL_TARGET,C02_BALL_APPROACH_X,C02_BALL_APPROACH_DRIVE_HEADING, C02_BALL_APPROACH_ROBOT_HEADING, C02_BALL_APPROACH_VELOCITY, null, 0, C02_BALL_APPROACH_TIMEOUT);

        // move off the wall just a bit to make intake work better
        if (!drive.mirroredMoveToYHoldingLine(C02_GRAB_VEL, Math.abs(drive.oQlocalizer.posY_mm)-C02_GRAB_Y_WALL_OFFSET,drive.oQlocalizer.posX_mm,C02_BALL_APPROACH_DRIVE_HEADING+180, C02_BALL_APPROACH_ROBOT_HEADING, C02_GRAB_VEL, null, 0, 1500)) return false;

        intake.intakeNum = 0; // dont return instantly from grab3

        // pick up the balls
        if (!grab3(C02_GRAB_VEL, C02_GRAB_X_LIMIT, C02_GRAB_TIME)) return false;

        Intake.INTAKE_IN_POWER = stored; // restore intake speed default
        return true;
    }

    // rolls straight at 180 trying to pickup 3 balls
    // returns when it has 3, runs out of time, or when it reaches a certain x threshold
    // assumes intake is on and intake detector is running
    public boolean grab3(int velocity, int xThreshold, long timeOut) {
        teamUtil.log("grab3");
        long timeOutTime = System.currentTimeMillis()+timeOut;
        while (teamUtil.keepGoing(timeOutTime) && intake.intakeNum < 3 && drive.oQlocalizer.posX_mm > xThreshold) {
            drive.loop();
            intake.detectIntakeArtifactsV2();
            intake.signalArtifacts();
            drive.driveMotorsHeadingsFR(180, 0, velocity);
        }
        drive.stopMotors();
        if (System.currentTimeMillis() >= timeOutTime) {
            teamUtil.log("grab3 TIMED OUT.");
            intake.intakeStop();
            return false;
        }
        if (drive.oQlocalizer.posX_mm <= xThreshold) {
            teamUtil.log("grab3 Reached X Threshold.");
        }
        teamUtil.log("grab3 Finished with " + intake.intakeNum + " artifacts.");
        return true;
    }





    ///  ///////////////////////////////////////////////////////
    /// // OLD CODE
    ///
    /*

    /* This is dated now, but might want it for a fast shoot in auto?
    public void shootAllArtifacts(){
        intake.detectLoadedArtifacts();
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
                teamUtil.log("shootAllArtifacts: Moved left and right flippers");
                shooter.pusher.pushNNoWait(2, AxonPusher.RTP_MAX_VELOCITY, 1000);
            }if(loadedArtifacts[0] == Intake.ARTIFACT.NONE){
                intake.middle_flipper.setPosition(Intake.MIDDLE_FLIPPER_SHOOTER_TRANSFER);
                intake.right_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
                teamUtil.pause(FIRST_UNLOAD_PAUSE);
                intake.middle_flipper.setPosition(Intake.FLIPPER_CEILING);
                shooter.pusher.pushNNoWait(2, AxonPusher.RTP_MAX_VELOCITY, 1250);
                intake.right_flipper.setPosition(Intake.FLIPPER_CEILING);
                teamUtil.log("shootAllArtifacts: Moved middle and right flippers");
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
            teamUtil.log("shootAllArtifacts: Moved all flippers");
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

    public void autoShootAllArtifactsNoWait(){
        teamUtil.log("Launching Thread to shootAllArtifactsNoWait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                autoShootAllArtifacts();
            }
        });
        thread.start();
    }

    public void autoShootAllArtifacts(){
        intake.detectLoadedArtifacts();
        int ballCount = intake.loadedBallNum();
        Intake.ARTIFACT[] loadedArtifacts = {intake.leftLoad, intake.middleLoad, intake.rightLoad};
        if(ballCount == 0){
            teamUtil.log("shootAllArtifacts called without loaded artifacts");
            return;
        }
        teamUtil.log("shootAllArtifacts starting with "+ballCount+" balls");

        intake.middle_flipper.setPosition(Intake.MIDDLE_FLIPPER_SHOOTER_TRANSFER);
        intake.left_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);
        intake.right_flipper.setPosition(Intake.EDGE_FLIPPER_SHOOTER_TRANSFER);

        teamUtil.pause(FIRST_UNLOAD_PAUSE);
        intake.middle_flipper.setPosition(Intake.FLIPPER_CEILING);
        teamUtil.log("Shot 1 "+ shootIfCan());
        intake.left_flipper.setPosition(Intake.FLIPPER_CEILING);
        teamUtil.pause(SECOND_UNLOAD_PAUSE);
        teamUtil.log("Shot 2 "+ shootIfCan());
        intake.right_flipper.setPosition(Intake.FLIPPER_CEILING);
        teamUtil.pause(SECOND_UNLOAD_PAUSE);
        teamUtil.log("Shot 3 "+ shootIfCan());
        teamUtil.log("autoShootAllArtifacts: Moved all flippers");
    }

        public void autoShootArtifacts(teamUtil.Pattern loadPattern){
        if(teamUtil.pattern == PPG){
            if(loadPattern == teamUtil.Pattern.PPG){
                shootArtifactLocation(Intake.Location.LEFT);
                shootArtifactLocation(Intake.Location.CENTER);
                shootArtifactLocation(Intake.Location.RIGHT); // green is right and last
            }else if(loadPattern == teamUtil.Pattern.PGP){
                shootArtifactLocation(Intake.Location.LEFT);
                shootArtifactLocation(Intake.Location.RIGHT);
                shootArtifactLocation(Intake.Location.CENTER); // green is center and last
            }else{//gpp
                shootArtifactLocation(Intake.Location.RIGHT);
                shootArtifactLocation(Intake.Location.CENTER);
                shootArtifactLocation(Intake.Location.LEFT); // green is left and last
            }
        }else if(teamUtil.pattern == PGP){
            if(loadPattern == teamUtil.Pattern.PPG){
                shootArtifactLocation(Intake.Location.LEFT);
                shootArtifactLocation(Intake.Location.RIGHT); // green is right and center
                shootArtifactLocation(Intake.Location.CENTER);
            }else if(loadPattern == teamUtil.Pattern.PGP){
                shootArtifactLocation(Intake.Location.LEFT);
                shootArtifactLocation(Intake.Location.CENTER); // green is center and center
                shootArtifactLocation(Intake.Location.RIGHT);
            }else{//gpp
                shootArtifactLocation(Intake.Location.RIGHT);
                shootArtifactLocation(Intake.Location.LEFT); // green is left and center
                shootArtifactLocation(Intake.Location.CENTER);
            }
        }else{ // GPP
            if(loadPattern == teamUtil.Pattern.PPG){
                shootArtifactLocation(Intake.Location.RIGHT); // green is right and first
                shootArtifactLocation(Intake.Location.LEFT);
                shootArtifactLocation(Intake.Location.CENTER);
            }else if(loadPattern == teamUtil.Pattern.PGP){
                shootArtifactLocation(Intake.Location.CENTER); // green is center and first
                shootArtifactLocation(Intake.Location.LEFT);
                shootArtifactLocation(Intake.Location.RIGHT);
            }else{//gpp
                shootArtifactLocation(Intake.Location.LEFT); // green is left and first
                shootArtifactLocation(Intake.Location.RIGHT);
                shootArtifactLocation(Intake.Location.CENTER);
            }
        }
    }


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
            teamUtil.pause(100);
        }
        teamUtil.log("Waited " + (System.currentTimeMillis()-now) + "millisecs for flywheels");


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

        autoShootArtifacts(loadPattern);
        return true;
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


    public static double A08_SHOOT1_Y = 634;
    public static double A08_SHOOT1_X = 617;
    public static double A08_SHOOT1_H = 45;

    public static double A09_SHOOT1_Y = 460;
    public static double A09_SHOOT1_X = 440;
    public static double A09_SHOOT1_H = 45;

    public static double A90_PARK_DRIFT = 200;
    public static double A90_PARK_X = 0 - A90_PARK_DRIFT;
    public static double A90_PARK_END_VELOCITY = 2000;

    public static double A99_GRAB_LAST_THREE_TIME = 4000;
    public static double A99_PARK_TIME = 1500;
    public static double A99_MOVE_OFF_LINE_TIME = 1500;
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
        //logShot(1, (int)A05_SHOOT1_X, (int)A05_SHOOT1_Y, (int)goalDistance, A05_SHOOT1_H);

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
        //logShot(2, (int)A08_SHOOT1_X, (int)A08_SHOOT1_Y, (int)goalDistance, A08_SHOOT1_H);
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
        //logShot(3, (int)A09_SHOOT1_X, (int)A09_SHOOT1_Y, (int)goalDistance, A09_SHOOT1_H);
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
*/


    public void humanSide(boolean useArms) {
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Auto Park Code
    // This code assumes the robot is already at a specified heading (315 for BLUE, ??? for RED)
    public static int LIFT_AUTO_ALIGN_VELOCITY = 200;
    public static int LIFT_AUTO_ALIGN_BLUE_THRESHOLD = 2500;
    public static int LIFT_AUTO_ALIGN_RED_THRESHOLD = 2500;

    public boolean seeLine(){
        return  (teamUtil.alliance== teamUtil.Alliance.BLUE && footColorSensor.blue() > LIFT_AUTO_ALIGN_BLUE_THRESHOLD) ||
                (teamUtil.alliance== teamUtil.Alliance.RED && footColorSensor.red() > LIFT_AUTO_ALIGN_RED_THRESHOLD);
    }

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
    public boolean alignForLiftV2(){
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


    /// ////////////////////////////////////
    // OLDER Code

    /*
        /// ///////////////////////////////////////////////////////////////
    /// Dated now, trying V2 stuff
    public  Intake.Location[][][][][] loadMap;
    public void setupLoadMap() {
        loadMap = new Intake.Location[3][3][3][3][4]; // pattern, left loaded, center loaded, right loaded, shot num
        loadMap[PPG.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][1] = Intake.Location.CENTER;
        loadMap[PPG.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][2] = Intake.Location.LEFT;
        loadMap[PPG.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][3] = Intake.Location.RIGHT;

        loadMap[PPG.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][1] = Intake.Location.LEFT;
        loadMap[PPG.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][2] = Intake.Location.RIGHT;
        loadMap[PPG.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][3] = Intake.Location.CENTER;

        loadMap[PPG.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][1] = Intake.Location.CENTER;
        loadMap[PPG.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][2] = Intake.Location.RIGHT;
        loadMap[PPG.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][3] = Intake.Location.LEFT;

        loadMap[PGP.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][1] = Intake.Location.CENTER;
        loadMap[PGP.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][2] = Intake.Location.RIGHT;
        loadMap[PGP.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][3] = Intake.Location.LEFT;

        loadMap[PGP.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][1] = Intake.Location.LEFT;
        loadMap[PGP.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][2] = Intake.Location.CENTER;
        loadMap[PGP.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][3] = Intake.Location.RIGHT;

        loadMap[PGP.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][1] = Intake.Location.CENTER;
        loadMap[PGP.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][2] = Intake.Location.LEFT;
        loadMap[PGP.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][3] = Intake.Location.RIGHT;

        loadMap[GPP.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][1] = Intake.Location.RIGHT;
        loadMap[GPP.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][2] = Intake.Location.CENTER;
        loadMap[GPP.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][3] = Intake.Location.LEFT;

        loadMap[GPP.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][1] = Intake.Location.CENTER;
        loadMap[GPP.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][2] = Intake.Location.LEFT;
        loadMap[GPP.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][3] = Intake.Location.RIGHT;

        loadMap[GPP.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][1] = Intake.Location.LEFT;
        loadMap[GPP.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][2] = Intake.Location.CENTER;
        loadMap[GPP.ordinal()][Intake.ARTIFACT.GREEN.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][Intake.ARTIFACT.PURPLE.ordinal()][3] = Intake.Location.RIGHT;

        // Any lookup in this array using ARTIFACT.NONE.ordinal() will hit the zero entry for column 2-4 which will be zero, which is Intake.Location.LEFT
        // We could add a NONE value to the Location enum and set all these to NONE to make this a bit cleaner

    }

    public Intake.Location nextLoad(int num) {
        return loadMap[teamUtil.pattern.ordinal()][intake.leftLoad.ordinal()][intake.middleLoad.ordinal()][intake.rightLoad.ordinal()][num];
    }

    public boolean patternShotAvailable (int num) {
        if (num <1 || num > 3) return false;
        Intake.Location location = nextLoad(num);
        if ((location== Intake.Location.LEFT && intake.leftLoad == Intake.ARTIFACT.NONE) ||
                (location== Intake.Location.CENTER && intake.middleLoad == Intake.ARTIFACT.NONE) ||
                (location== Intake.Location.RIGHT && intake.rightLoad == Intake.ARTIFACT.NONE)) { // nothing loaded in the next flipper to unload
            return false;
        } else {
            return true;
        }
    }

        public boolean loadPatternShotNoWaitOLD(int num) {
        if (!patternShotAvailable(num)) {
            teamUtil.log ("Empty Flipper, not unloading");
            return false;
        }
        teamUtil.log("Launching Thread to loadPatternShotNoWait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                int nextShot = num+1;
                while (!patternShotAvailable(nextShot) && nextShot < 4) {
                    nextShot++;
                }
                if (nextShot > 3) { // No next shot to pin
                    intake.unloadServo(nextLoad(num), Intake.Location.NONE);
                } else { // load the next shot and pin the next one if possible
                    intake.unloadServo(nextLoad(num), nextLoad(nextShot));
                }
            }
        });
        thread.start();
        return true;
    }


    // Attempts to run elevator to flippers and then load the first pattern shot
    // If balls are missing, will move to the next. Its possible that nothing will be loaded if nothing was in the intake
    public void autoTransferAndLoad (long pause, long timeOut) {
        transferring.set(true);
        teamUtil.log("autoTransferAndLoad with pause:  " + pause);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        teamUtil.pause(pause);
        if (intake.elevatorToFlippersV2(false)) {
            // TODO: This might be a good place to detect what is loaded

            int shot = 1; // load first shot that is available
            int nextShot;
            while (!patternShotAvailable(shot) && shot < 4) {
                shot++;
            }
            nextShot = shot+1;
            while (!patternShotAvailable(nextShot) && nextShot < 4) {
                nextShot++;
            }
            if (shot>3) { // nothing to transfer
                teamUtil.log("autoTransferAndLoad: Nothing to load");
                transferring.set(false);
            } else { // We can at least load one into the shooter
                if (nextShot > 3) { // No next shot to pin
                    intake.unloadServo(nextLoad(shot), Intake.Location.NONE);
                } else { // load the next shot and pin the next one if possible
                    intake.unloadServo(nextLoad(shot), nextLoad(nextShot));
                }
            }
        } else { // intake failed in some way
            // TODO: Maybe pause then try again for some amount of time?
            teamUtil.log("ElevatorToFlippersV2 failed. Giving up on Transfer.");
        }
        transferring.set(false);
        teamUtil.log("autoTransferAndLoad Finished");
    }


     */
}

