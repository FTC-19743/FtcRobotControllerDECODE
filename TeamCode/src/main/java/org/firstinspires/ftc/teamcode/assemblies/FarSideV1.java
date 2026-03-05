package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Pattern.GPP;
import static org.firstinspires.ftc.teamcode.libs.teamUtil.Pattern.PPG;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
public class FarSideV1 {

    Robot robot;
    Telemetry telemetry;
    Shooter shooter;
    Intake intake;
    BasicDrive drive;

    public FarSideV1(Robot theRobot, Telemetry theTelemetry) {
        robot = theRobot;
        telemetry = theTelemetry;
        intake = theRobot.intake;
        shooter = theRobot.shooter;
        drive = theRobot.drive;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Main Auto Code

    // This is what we are aiming for throughout auto for max consistency while minimizing travel distances
    public static int IDEAL_GOAL_DISTANCE = 3200;
    public static int IDEAL_FLYWHEEL = 1460; // was 1550-1570
    public static float IDEAL_PITCH = 0.5f;


    public static double B00_MAX_SPEED = 2200;
    public static double B00_CORNER_VELOCITY = 1800;
    public static double B00_PICKUP_VELOCITY = 2000;
    public static double B00_TILE_LENGTH = 610;
    public static double B00_PICKUP1_Y = 1200;
    public static double B00_PICKUP_H = 180;
    public static long B00_INTAKE_REVERSE_PAUSE = 200;

    public static long B09_PARK_TIME_CUTOFF = 2000;
    public static double B09_PARK_END_VELOCITY = 1500;
    public static double B09_PARK_X = 200;
    public static double B09_PARK_Y = 1000;
    public static double B09_PARK_DRIVE = 105;

    public static boolean emptyRamp = true;
    public static int emptyRampPause = 1000;

    public static boolean details = false;

    public void stopRobot(){
        drive.stopMotors();
        intake.intakeStop();
        shooter.stopShooter();
    }

    public static double VELOCITY_THRESHOLD = 30;
    public boolean flywheelsReady() {
        if (details) {
            teamUtil.log("shooterFlyWheelsReady Waiting: TVel: " + Shooter.VELOCITY_COMMANDED + " RVel: " + shooter.rightFlywheel.getVelocity() + " LVel: " + shooter.leftFlywheel.getVelocity());
        }

        double velo = Shooter.VELOCITY_COMMANDED;
        double minVelo = velo - VELOCITY_THRESHOLD;
        double maxVelo = velo + VELOCITY_THRESHOLD;

        return (shooter.rightFlywheel.getVelocity() < maxVelo /*&& shooter.leftFlywheel.getVelocity() < maxVelo*/ &&
                shooter.rightFlywheel.getVelocity() > minVelo /*&& shooter.leftFlywheel.getVelocity() > minVelo*/);

    }
    public static double HEADING_ERROR_THRESHOLD = 1;
    public static double RED_H_ADJUST = -1.5;
    public static double BLUE_H_ADJUST = 2;
    public double robotGoalHeading() {
        return drive.robotGoalHeading() + (teamUtil.alliance== teamUtil.Alliance.RED ? RED_H_ADJUST : BLUE_H_ADJUST);
    }
    public boolean headingReady() {
        return Math.abs(drive.getHeadingODO() - robotGoalHeading()) < HEADING_ERROR_THRESHOLD;
    }
    public void holdShotHeading() {
        drive.loop();
        double shotHeading = robotGoalHeading();
        drive.driveMotorsHeadingsFR(shotHeading, shotHeading, 0); // continue to rotate to match shot heading
    }

    public static long FAR_SIDE_PUSH_PAUSE = 400;
    public boolean farSideShoot3 (boolean useArms, long flyWheelPause, long timeOut) {
        teamUtil.log("farSideShoot3 Starting");
        long startTime = System.currentTimeMillis();
        long timeOutTime = startTime + timeOut;
        long FlywheelTime = startTime + flyWheelPause;
        drive.stopMotors();
        drive.waitForRobotToStop(2000); // kill any remaining momentum before we start looking for the shot
        // wait for initial transfer/flip to complete
        if (robot.transferring.get() || intake.flipping.get()) {
            teamUtil.log("farSideShoot3 waiting on transfer/flip");
            while ((robot.transferring.get() || intake.flipping.get()) && teamUtil.keepGoing(timeOutTime)) {
                holdShotHeading();
            }
        }
        if (useArms) {
            // Make sure flywheels are spun up and heading is good
            while ((!headingReady() || !shooter.isLoaded() || !flywheelsReady() || System.currentTimeMillis() < FlywheelTime) && teamUtil.keepGoing(timeOutTime)) {
                if (details) teamUtil.log("Waiting Heading: "+headingReady()+" Loaded: " + shooter.isLoaded() + " Flywheels: " + flywheelsReady());
                holdShotHeading();
            }
        }
        robot.drive.stopMotors();

        if (System.currentTimeMillis() >=timeOutTime) {
            teamUtil.log("farSideShoot3 TIMED OUT waiting for valid shooting conditions");
            return false;
        } else {
            long storeLeft = Shooter.SF_LEFT_PUSH_PAUSE_FAR;
            long storeRight = Shooter.SF_RIGHT_PUSH_PAUSE_FAR;
            Shooter.SF_LEFT_PUSH_PAUSE_FAR = FAR_SIDE_PUSH_PAUSE;
            Shooter.SF_RIGHT_PUSH_PAUSE_FAR = FAR_SIDE_PUSH_PAUSE;
            //boolean result = robot.autoShootSuperFast(useArms, false,5000); // Don't bother with pattern

            double goalDistance = drive.robotGoalDistance();
            shooter.shootSuperFastNoWait(Intake.leftLoad != Intake.ARTIFACT.NONE, true, true, true, goalDistance); // launch the shot sequence in another thread
            // wait for it to finish while adjusting robot heading if needed
            while (shooter.superFastShooting.get() && teamUtil.keepGoing(timeOutTime)) {
                holdShotHeading();
            }
            Shooter.SF_LEFT_PUSH_PAUSE_FAR = storeLeft;
            Shooter.SF_RIGHT_PUSH_PAUSE_FAR = storeRight;
            return true;
        }
    }
    ///  //////////////////////////////////////////////////////////////// Pre Loads (Group 1--"B01")
    public static double B01_SHOT_X = -1400;
    public static double B01_SHOT_Y = 400;
    public static double B01_SHOT_H = 21;
    public static double B01_SHOT_END_VEL = 300;
    public static double B01_FLYWHEEL_VEL = IDEAL_FLYWHEEL;
    public static float B01_SHOT_PITCH = IDEAL_PITCH;
    public static long B01_FLYWHEEL_PAUSE = 0;
    public boolean preloads(boolean useArms) {
        teamUtil.log("==================== Preloads (Group 1) ================");
        // Drive fast to shooting zone
        if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED, B01_SHOT_X, B01_SHOT_Y, 0, B01_SHOT_H, B01_SHOT_END_VEL, null, 0, 2000)) return false;
        // Shoot preloads
        intake.signalArtifacts();
        return farSideShoot3(useArms, B01_FLYWHEEL_PAUSE, 5000);
   }


    public static long B02_INTAKE_PAUSE = 300;

    public static double B02_OFF_WALL_X = -1450;
    public static double B02_OFF_WALL_Y = 1250;
    public static double B02_OFF_WALL_VELOCITY = B00_MAX_SPEED;
    public static double B02_OFF_WALL_DRIVE_HEADING = 305;
    public static double B02_OFF_WALL_ROBOT_HEADING = 270;
    public static double B02_OFF_WALL_END_VELOCITY = B00_CORNER_VELOCITY;

    public static double B02_FAST_APPROACH_X = B01_SHOT_X;
    public static double B02_FAST_APPROACH_Y = B01_SHOT_Y;
    public static double B02_FAST_APPROACH_VELOCITY = B00_MAX_SPEED;
    public static double B02_FAST_APPROACH_DRIVE_HEADING = 270;
    public static double B02_FAST_APPROACH_END_VELOCITY = 500;

    public static double B02_SPIN_CLIP = 35;

    public boolean getCornerBallsAndShoot(boolean useArms) {
        teamUtil.log("==================== getCornerBallsAndShoot ================");
        if (getMoreBalls()) {
            if (useArms) intake.elevatorToFlippersV2NoWait( false);// elevator up, but don't transfer yet because of crazy spins below

            // Drive back to shooting zone
            shooter.setShootSpeed(B01_FLYWHEEL_VEL);
            double store = BasicDrive.SPIN_CLIP;
            BasicDrive.SPIN_CLIP = B02_SPIN_CLIP;
            if (!drive.mirroredMoveToYHoldingLine(B02_OFF_WALL_VELOCITY, B02_OFF_WALL_Y,B02_OFF_WALL_X,B02_OFF_WALL_DRIVE_HEADING, B02_OFF_WALL_ROBOT_HEADING, B02_OFF_WALL_END_VELOCITY, null, 0, 2100)) return false;
            if (!drive.mirroredMoveToYHoldingLine(B02_FAST_APPROACH_VELOCITY, B02_FAST_APPROACH_Y,B02_FAST_APPROACH_X,B02_FAST_APPROACH_DRIVE_HEADING, B01_SHOT_H, B02_FAST_APPROACH_END_VELOCITY, null, 0, 2100)) return false;
            BasicDrive.SPIN_CLIP = store;
            if (useArms) robot.autoShootSuperFastPreloadNoWait();

            return farSideShoot3(useArms, 0, 5000);
        } else {
            return false;
        }
    }

    public static double B03_PICKUP_X = -950;
    public static double B03_PICKUP_Y = 1200;
    public static double B03_PICKUP_END_VELOCITY = 500;
    public static double B03_PICKUP_DH = 0;
    public static double B03_PICKUP_H = 180;
    public static double B03_PICKUP_VELOCITY = 1000;
    public static long  B03_PICKUP_PAUSE = 0;
    public static long  B03_PICKUP_INTAKE_PAUSE = 500;

    public static double B03_SETUP_Y = B03_PICKUP_Y-100;
    public static double B03_SETUP_X = -1350;
    public static double B03_SETUP_DH = 90;
    public static double B03_SETUP_END_VELOCITY = 500;

    public static double B03_CORNER_X = B03_SETUP_X+200;
    public static double B03_CORNER_VELOCITY = 300;

    public static double B03_SPIN_CLIP = 45;

    public boolean getPatternBallsAndShoot(boolean useArms, boolean useIntakeDetector) {
        // pickup group 4
        teamUtil.log("==================== Pattern Set ================");
        if (useArms) { intake.getReadyToIntakeNoWait(); }
        if (!drive.mirroredMoveToYHoldingLine(B00_MAX_SPEED, B03_SETUP_Y, B03_SETUP_X, B03_SETUP_DH, B03_PICKUP_H, B03_SETUP_END_VELOCITY, null, 0, 3000)) return false;
        if (!drive.mirroredMoveToXHoldingLine(B03_PICKUP_VELOCITY, B03_PICKUP_X, B03_PICKUP_Y, B03_PICKUP_DH, B03_PICKUP_H, B03_PICKUP_END_VELOCITY, null, 0, 3000)) return false;
        // Manually set what is loaded in intake in case detector fails
        if (useIntakeDetector) {
            intake.detectIntakeArtifactsV2();
        } else {
            intake.logDetectorOutput(); // for debugging purposes
            if (teamUtil.alliance == teamUtil.Alliance.BLUE) { // balls are reversed from audience
                intake.setIntakeArtifacts(GPP);
                intake.setLoadedArtifacts(GPP); // Assumes artifacts are preloaded in this order!!

            } else {
                intake.setIntakeArtifacts(PPG);
                intake.setLoadedArtifacts(PPG); // Assumes artifacts are preloaded in this order!!
            }
        }
        intake.signalArtifacts();

        drive.stopMotors(); // kill some forward momentum
        teamUtil.pause(B03_PICKUP_PAUSE);
        if (useArms) intake.elevatorToFlippersV2NoWait( false);// elevator up, but don't transfer yet because of crazy spins below

        // Drive back to shooting zone
        shooter.setShootSpeed(B01_FLYWHEEL_VEL);
        double store = BasicDrive.SPIN_CLIP;
        BasicDrive.SPIN_CLIP = B03_SPIN_CLIP;
        if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED, B03_CORNER_X,B03_PICKUP_Y,B03_PICKUP_DH+180, B03_PICKUP_H, B03_CORNER_VELOCITY, null, 0, 2100)) return false;
        if (!drive.mirroredMoveToYHoldingLine(B02_FAST_APPROACH_VELOCITY, B02_FAST_APPROACH_Y,B02_FAST_APPROACH_X,B02_FAST_APPROACH_DRIVE_HEADING, B01_SHOT_H, B02_FAST_APPROACH_END_VELOCITY, null, 0, 2100)) return false;
        BasicDrive.SPIN_CLIP = store;
        if (useArms) robot.autoShootSuperFastPreloadNoWait();

        return farSideShoot3(useArms, 0, 5000);
    }


    public static long CYCLE_TIME = 6000;
    public static long PARK_TIME = 2000;

    public static double B05_PARK_X = -950;
    public static double B05_PARK_Y = B01_SHOT_Y;
    public static double B05_PARK_END_VELOCITY = 300;

    ///  ////////////////////////////////////////////////////////////////  GOAL SIDE V3
    public void farSideV1(boolean useArms, boolean useIntakeDetector, boolean getPatternBalls, long cycle1Time, long cycle2Time, long cycle3Time, long cycle4Time) {
        double nextGoalDistance = 0;
        long startTime = System.currentTimeMillis();
        double savedDeclination;
        teamUtil.log("##################################################################################");
        teamUtil.log("#########################  Starting FarSideV1 Auto ##############################");
        shooter.flywheelEnhancedFar(); // set flywheel to fast start PIDF coefs
        intake.setLoadedArtifacts(PPG); // Assumes artifacts are preloaded in this order!!
        intake.setIntakeArtifacts(PPG);
        intake.signalArtifacts();
        // Prep Shooter
        if (useArms) {
            shooter.setShootSpeed(B01_FLYWHEEL_VEL);
            Shooter.VELOCITY_COMMANDED = B01_FLYWHEEL_VEL;
            shooter.aim(B01_SHOT_PITCH);
            shooter.sidePushersHold();
            //autoShootSuperFastPreloadNoWait(); // Don't need this any more, load directly to shooter
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
            teamUtil.log("NOT using Detector. Running in hardcoded mode.");
        }

        /////////////////////////////Shoot Preloads (Group 1)
        if (!preloads(useArms)) {
            stopRobot();
            return;
        }
        shooter.setShootSpeed(Shooter.MAX_IDLE_FLYWHEEL_VELOCITY);

        /////////////////////////////Get the Pre-Set corner ones and shoot (Group 2)
        if (!getCornerBallsAndShoot(useArms)) {
            stopRobot();
            return;
        }
        shooter.setShootSpeed(Shooter.MAX_IDLE_FLYWHEEL_VELOCITY);

        /////////////////////////////Get the nearest pattern pre-set group and shoot (Group 3 optional)
        if (getPatternBalls) {
            if (!getPatternBallsAndShoot(useArms, useIntakeDetector)) {
                stopRobot();
                return;
            }
            shooter.setShootSpeed(Shooter.MAX_IDLE_FLYWHEEL_VELOCITY);
        }

        ///////////////////////////// Cycle Corner 1
        if (cycle1Time > 0 && System.currentTimeMillis()- startTime + CYCLE_TIME + PARK_TIME < 30000) {// if we can do cycle1 and still park, go for it
            while (System.currentTimeMillis() - startTime < cycle1Time) {    // wait until its time
                teamUtil.pause(50);
            }
            if (!getCornerBallsAndShoot(useArms)) {stopRobot();return;}
            shooter.setShootSpeed(Shooter.MAX_IDLE_FLYWHEEL_VELOCITY);
        }
        ///////////////////////////// Cycle Corner 2
        if (cycle2Time > 0 && System.currentTimeMillis() - startTime + CYCLE_TIME + PARK_TIME < 30000) {// if we can do cycle2 and still park, go for it
            while (System.currentTimeMillis() - startTime < cycle2Time) {    // wait until its time
                teamUtil.pause(50);
            }
            if (!getCornerBallsAndShoot(useArms)) {stopRobot();return;}
            shooter.setShootSpeed(Shooter.MAX_IDLE_FLYWHEEL_VELOCITY);
        }
        ///////////////////////////// Cycle Corner 3
        if (cycle3Time > 0 && System.currentTimeMillis() - startTime + CYCLE_TIME + PARK_TIME < 30000) {// if we can do cycle3 and still park, go for it
            while (System.currentTimeMillis() - startTime < cycle3Time) {    // wait until its time
                teamUtil.pause(50);
            }
            if (!getCornerBallsAndShoot(useArms)) {stopRobot();return;}
            shooter.setShootSpeed(Shooter.MAX_IDLE_FLYWHEEL_VELOCITY);
        }
        ///////////////////////////// Cycle Corner 4
        if (cycle4Time > 0 && System.currentTimeMillis() - startTime + CYCLE_TIME + PARK_TIME < 30000) {// if we can do cycle4 and still park, go for it
            while (System.currentTimeMillis() - startTime < cycle4Time) {    // wait until its time
                teamUtil.pause(50);
            }
            if (!getCornerBallsAndShoot(useArms)) {stopRobot();return;}
            shooter.setShootSpeed(Shooter.MAX_IDLE_FLYWHEEL_VELOCITY);
        }

        /////////////////////////////Park off the line
        if(System.currentTimeMillis() - startTime < 30000 - PARK_TIME){ // 2 seconds left
            if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED, B05_PARK_X, B05_PARK_Y, 0, 0, B05_PARK_END_VELOCITY, null, 0, 2000)) {stopRobot();return;}
            drive.stopMotors();
        }else{
            teamUtil.log("Not enough time to park. Stopping");
        }

        /////////////////////////////Wrap up
        stopRobot();
    }




    public static double C01_FAST_APPROACH_VELOCITY = B00_MAX_SPEED;
    public static double C01_FAST_APPROACH_X = -1550;
    public static double C01_FAST_APPROACH_Y = 1250;
    public static double C01_FAST_APPROACH_DRIVE_HEADING = 90;
    public static double C01_FAST_APPROACH_ROBOT_HEADING = 270;
    public static double C01_FAST_APPROACH_END_VELOCITY = 500;
    public static double C01_GRAB_INTAKE_POWER = .9;


    public static int C04_GRAB_VEL = 1000;
    public static int C04_GRAB_Y_LIMIT = 1430;
    public static long C04_GRAB_TIME = 3000;
    public static float C04_GRAB_POWER = .3f;
    public static long C04_GRAB_PAUSE = 250;

    public boolean getMoreBalls(){
        teamUtil.log("getMoreBalls");

        // get Intake Ready
        double stored = Intake.INTAKE_IN_POWER;
        Intake.INTAKE_IN_POWER = C01_GRAB_INTAKE_POWER; // adjust intake speed for this operation
        intake.getReadyToIntakeNoWait();

        // Drive towards wall fast
        if (!drive.mirroredMoveToYHoldingLine(C01_FAST_APPROACH_VELOCITY, C01_FAST_APPROACH_Y,C01_FAST_APPROACH_X,C01_FAST_APPROACH_DRIVE_HEADING, C01_FAST_APPROACH_ROBOT_HEADING, C01_FAST_APPROACH_END_VELOCITY, null, 0, 2100)) return false;
        intake.intakeNum = 0; // don't return instantly from grab3
        if (!grab3V2(C04_GRAB_VEL, C04_GRAB_Y_LIMIT, C04_GRAB_TIME)) return false;

        Intake.INTAKE_IN_POWER = stored; // restore intake speed default
        teamUtil.log("getMoreBalls Finished");
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

    public void park() {

    }

}
