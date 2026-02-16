package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Pattern.GPP;
import static org.firstinspires.ftc.teamcode.libs.teamUtil.Pattern.PGP;
import static org.firstinspires.ftc.teamcode.libs.teamUtil.Pattern.PPG;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
public class GoalSideV4 {

    Robot robot;
    Telemetry telemetry;
    Shooter shooter;
    Intake intake;
    BasicDrive drive;

    public GoalSideV4(Robot theRobot, Telemetry theTelemetry) {
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
    public static int IDEAL_GOAL_DISTANCE = 1220;
    public static int IDEAL_FLYWHEEL = 800;
    public static float IDEAL_PITCH = 0.32f;


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

    public void stopRobot(){
        drive.stopMotors();
        intake.intakeStop();
        shooter.stopShooter();
    }

    ///  //////////////////////////////////////////////////////////////// Pre Loads (Group 1--"B01")
    public static double B01_SHOT_X = 780;
    public static double B01_SHOT_Y = 780;
    public static double B01_SHOT_H = 45;
    public static double B01_SHOT_END_VEL = 500;
    public static double B01_FLYWHEEL_VEL = IDEAL_FLYWHEEL-20; // a little less for the super fast shots up front
    public static float B01_SHOT_PITCH = IDEAL_PITCH;
    public boolean preloads(boolean useArms) {
        teamUtil.log("==================== Preloads (Group 1) ================");
        // Drive fast to shooting zone
        if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED, B01_SHOT_X, B01_SHOT_Y, B01_SHOT_H -180, B01_SHOT_H, B01_SHOT_END_VEL, null, 0, 2000)) return false;
        // Shoot preloads
        intake.signalArtifacts(); // flippers were operating in another thread while we were moving to this point.
        return robot.autoShootSuperFast(useArms, false,5000); // Don't bother with pattern on preloads since we are going to empty the ramp
    }

    ///  //////////////////////////////////////////////////////////////// Group 2--"B02")
    public static double B02_SETUP_Y_DRIFT = 170;
    public static double B02_SETUP_Y = B00_PICKUP1_Y - B02_SETUP_Y_DRIFT;
    public static double B02_SETUP_X = 670;
    public static double B02_SETUP_DH = 90;
    public static double B02_SETUP_H = 0;
    public static double B02_SETUP_END_VEL = B00_CORNER_VELOCITY;
    public static long B02_SETUP_PAUSE = 150;
    public static double B02_PICKUP_X = 420;
    public static double B02_PICKUP_ADJ = 0;
    public static long B02_PICKUP_INTAKE_PAUSE = 0;

    public static double B02_SHOT_MID_X = 320;
    public static double B02_SHOT_MID_Y = 1100;
    public static double B02_SHOT_X = 550;
    public static double B02_SHOT_Y = 600;
    public static double B02_SHOT_H = 45;
    public static double B02_SHOT_DH = 305;
    public static double B02_SHOT_END_VEL = 1000; // was 400 before GoalSideV3
    public static double B02_SHOT_DRIFT = 300;
    public boolean group2(boolean useArms, boolean useIntakeDetector) {
        // Setup to pickup group 2
        teamUtil.log("==================== Group 2 ================");
        if (useArms) { intake.getReadyToIntakeNoWait(); }
        if (!drive.mirroredMoveToYHoldingLine(B00_MAX_SPEED, B02_SETUP_Y, B02_SETUP_X, B02_SETUP_DH, B02_SETUP_H, B02_SETUP_END_VEL, null, 0, 1500)) return false;
        drive.stopMotors(); // help kill the sideways momentum
        teamUtil.pause(B02_SETUP_PAUSE);
        // Pickup group 2
        if (!drive.mirroredMoveToXHoldingLine(B00_PICKUP_VELOCITY, B02_PICKUP_X + B02_PICKUP_ADJ, B00_PICKUP1_Y, B00_PICKUP_H, B02_SETUP_H, B00_CORNER_VELOCITY, null, 0, 1500)) return false;
        // prepare to shoot
        if (useIntakeDetector) {
            intake.detectIntakeArtifactsV2();
        } else {
            intake.logDetectorOutput(); // for debugging purposes
            // Manually set what is loaded in intake
            if (teamUtil.alliance == teamUtil.Alliance.BLUE) { // balls are reversed from audience
                intake.setIntakeArtifacts(GPP);
                intake.setLoadedArtifacts(GPP); // Assumes artifacts are preloaded in this order!!
            } else {
                intake.setIntakeArtifacts(PPG);
                intake.setLoadedArtifacts(PPG); // Assumes artifacts are preloaded in this order!!
            }
        }
        // Drive back to shooting zone
        drive.stopMotors(); // kill forward momentum without tipping robot (odo pods) off of ground
        teamUtil.pause(B00_INTAKE_REVERSE_PAUSE);
        intake.signalArtifacts();
        if (useArms) robot.autoTransferAndLoadSuperFastNoWait(B02_PICKUP_INTAKE_PAUSE, 3000);
        // back up without turning robot to ensure we clear group 3
        if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED, B02_SHOT_MID_X, B02_SHOT_MID_Y, B02_SHOT_DH, B02_SETUP_H, B00_CORNER_VELOCITY, null, 0, 1500)) return false;
        // rotate and strafe to shooting position
        if (!drive.mirroredMoveToYHoldingLine(B00_MAX_SPEED, B02_SHOT_Y + B02_SHOT_DRIFT, B02_SHOT_X, B02_SHOT_DH, B02_SHOT_H, B02_SHOT_END_VEL, null, 0, 2000)) return false;
        if(intake.failedOut.get()){
            teamUtil.log("Auto has FAILED OUT because of a jammed intake");
            stopRobot();
            return false;
        }
        // shoot second set of balls
        return robot.autoShootSuperFast(useArms, false,5000); // Don't bother with pattern on 2nd group since we are going to empty the ramp
    }

    ///  //////////////////////////////////////////////////////////////// Group 3--"B03"  Also Ramp Release
    public static double B03_SETUP_DH = 105;
    public static double B03_SETUP_Y_DRIFT = 120;  //was 170
    public static double B03_SETUP_Y = B00_PICKUP1_Y - B03_SETUP_Y_DRIFT;
    public static double B03_SETUP_X = B02_SETUP_X - B00_TILE_LENGTH +50;
    public static long B03_PICKUP_INTAKE_PAUSE = 0;

    public static double B03_PICKUP_RAMP_END_VEL = 750;
    public static double B00_GATE_END_VEL = 500;
    public static double B00_GATE_VELOCITY = 1000; // was 600 before GoalSideV3
    public static double B00_GATE_Y = B00_PICKUP1_Y +190;
    public static double B00_GATE_X = 200;
    public static double B00_GATE_H = 90;
    public static long B00_GATE_BACKUP_X = -200;

    public static long B00_GATE_TIMEOUT = 750;

    public static double B03_SHOT_X = 650;
    public static double B03_SHOT_Y = 800;
    public static double B03_SHOT_DH = 340;
    public static double B03_SHOT_END_VEL = 800; // was 400 before GoalSideV3
    public static double B03_SHOT_DRIFT = 100;
    public static double B03_SHOT_H = B02_SHOT_H;
    public static double B03_FLYWHEEL_VELOCITY = IDEAL_FLYWHEEL;
    public static float B03_SHOT_PITCH = IDEAL_PITCH;
    public boolean group3(boolean useArms, boolean useIntakeDetector, long startTime, long gateLeaveTime) {
        if (useArms) {
            shooter.setShootSpeed(B03_FLYWHEEL_VELOCITY);
            Shooter.VELOCITY_COMMANDED = B03_FLYWHEEL_VELOCITY;
            shooter.aim(B03_SHOT_PITCH);
        }
        // Setup to pickup group 3
        teamUtil.log("==================== Group 3 (empty ramp) ================");
        if (useArms) { intake.getReadyToIntakeNoWait(); }
        if (!drive.mirroredMoveToYHoldingLine(B00_MAX_SPEED, B03_SETUP_Y, B03_SETUP_X, B03_SETUP_DH, B02_SETUP_H, B02_SETUP_END_VEL, null, 0, 1500)) return false;
        drive.stopMotors(); // help kill the sideways momentum
        teamUtil.pause(B02_SETUP_PAUSE);
        // Pickup group 3
        if (!drive.mirroredMoveToXHoldingLine(B00_PICKUP_VELOCITY, B02_PICKUP_X - B00_TILE_LENGTH, B00_PICKUP1_Y, B00_PICKUP_H, B02_SETUP_H, B00_CORNER_VELOCITY, null, 0, 3000)) return false;
        // back up a bit for gate
        drive.stopMotors(); // kill forward momentum without tipping robot (odo pods) off of ground
        teamUtil.pause(B00_INTAKE_REVERSE_PAUSE);
        if (!drive.mirroredMoveToXHoldingLine(B00_PICKUP_VELOCITY, B00_GATE_BACKUP_X, B00_PICKUP1_Y,180- B00_PICKUP_H, B02_SETUP_H, B03_PICKUP_RAMP_END_VEL, null, 0, 1500)) return false;
        // push the gate allowing for timeout
        drive.mirroredMoveToYHoldingLine(B00_GATE_VELOCITY, B00_GATE_Y, B00_GATE_X, B00_GATE_H, B02_SETUP_H, B00_GATE_END_VEL, null, 0, B00_GATE_TIMEOUT);
        drive.stopMotors();
        // Determine what is loaded
        if (useIntakeDetector) {
            intake.detectIntakeArtifactsV2();
        } else {
            intake.logDetectorOutput(); // for debugging purposes
            intake.setIntakeArtifacts(PGP);
            intake.setLoadedArtifacts(PGP); // Assumes artifacts are preloaded in this order!!
        }
        intake.signalArtifacts();
        // prepare shot
        if (useArms) robot.autoTransferAndLoadNoWait(B03_PICKUP_INTAKE_PAUSE, false, 3000);
        // Allow balls to exit ramp
        teamUtil.pause(emptyRampPause);
        // Allow more time for alliance partner
        long pause = gateLeaveTime - (System.currentTimeMillis() - startTime);
        teamUtil.pause(pause);
        // Drive back to shooting zone
        if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED, B03_SHOT_X - B03_SHOT_DRIFT, B03_SHOT_Y, B03_SHOT_DH, B03_SHOT_H, B03_SHOT_END_VEL, null, 0, 3000)) return false;
        if(intake.failedOut.get()){
            teamUtil.log("Auto has FAILED OUT because of a jammed intake");
            stopRobot();
            return false;
        }
        // shoot 3rd set of balls
        return robot.autoShootPattern(useArms,5000);
        //return driveWhileShootingPattern(useArms, teamUtil.alliance== teamUtil.Alliance.BLUE ? (B08_SHOOT3_H) : 360-B08_SHOOT3_H,B00_SHOOT_VELOCITY,5000);

    }

    ///  //////////////////////////////////////////////////////////////// Group 4--"B04"
    public static double B04_SETUP_X = B02_SETUP_X - B00_TILE_LENGTH;
    public static double B04_SETUP4_Y = B00_PICKUP1_Y -100;
    public static double B04_PICKUP_X = -700;
    public static long B04_PICKUP_PAUSE = 300;
    public static long B04_PICKUP_INTAKE_PAUSE = 0;

    public static double B04_SHOT_X = 600; // was 650
    public static double B04_SHOT_Y = 650; // was 700
    public static double B04_SHOT_RH = 35;
    public static double B04_SHOT_END_VEL = 1000; // was 400 before GoalSideV3
    public static double B04_SHOT_STRAIGHT_PERCENT = .4;
    public static double B04_SHOT_DRIFT_PERCENT = .9;
    public static double B04_FLYWHEEL_VELOCITY = IDEAL_FLYWHEEL;
    public static float B04_SHOT_PITCH = IDEAL_PITCH;
    public boolean group4(boolean useArms, boolean useIntakeDetector) {
        if (useArms) {
            shooter.setShootSpeed(B04_FLYWHEEL_VELOCITY);
            Shooter.VELOCITY_COMMANDED = B04_FLYWHEEL_VELOCITY;
            shooter.aim(B04_SHOT_PITCH);
        }
        // pickup group 4
        teamUtil.log("==================== Group 4 ================");
        if (useArms) { intake.getReadyToIntakeNoWait(); }
        if (!drive.mirroredMoveToXHoldingLine(B00_PICKUP_VELOCITY, B04_SETUP_X, B04_SETUP4_Y, B00_PICKUP_H, B02_SETUP_H, B00_PICKUP_VELOCITY, null, 0, 3000)) return false;
        if (!drive.mirroredMoveToXHoldingLine(B00_PICKUP_VELOCITY, B04_PICKUP_X, B00_PICKUP1_Y, B00_PICKUP_H, B02_SETUP_H, B00_CORNER_VELOCITY, null, 0, 3000)) return false;
        // Manually set what is loaded in intake in case detector fails
        if (useIntakeDetector) {
            intake.detectIntakeArtifactsV2();
        } else {
            intake.logDetectorOutput(); // for debugging purposes
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
        teamUtil.pause(B04_PICKUP_PAUSE);
        if (useArms) robot.autoTransferAndLoadNoWait(B04_PICKUP_INTAKE_PAUSE, false, 3000);
        // Drive back to shooting zone
        if (!mirroredDriveToShotPositionFast(B04_SHOT_X, B04_SHOT_Y, B04_SHOT_RH, B04_SHOT_END_VEL, B04_SHOT_STRAIGHT_PERCENT, B04_SHOT_DRIFT_PERCENT)) return false; // Try the new faster one
        //if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED, B08_SHOOT4_X-B08_SHOOT4_DRIFT,B08_SHOOT4_Y,B08_SHOOT4_DH, B08_SHOOT4_H, B08_SHOOT4_END_VEL, null, 0, 4000)) return;
        if(intake.failedOut.get()){
            teamUtil.log("Auto has FAILED OUT because of a jammed intake");
            stopRobot();
            return false;
        }
        // shoot 4th set of balls
        return robot.autoShootPattern(useArms,5000);
    }

    ///  //////////////////////////////////////////////////////////////// Group 5--"B05"
    public static double B05_SHOT_THRESHOLD = 3000;
    public static long B05_INTAKE_PAUSE = 300;
    public static double B05_FLYWHEEL_VELOCITY = IDEAL_FLYWHEEL;
    public static double B05_SHOT_PITCH = IDEAL_PITCH;
    public static double B05_SHOT_X = 600; // was 650
    public static double B05_SHOT_Y = 700 ; // was 750
    public static double B05_SHOT_RH = 35;
    public static double B05_SHOT_END_VEL = 1000;
    public static double B05_SHOT_STRAIGHT_PERCENT = .5;
    public static double B05_SHOT_DRIFT_PERCENT = .95;
    public boolean group5andPark(boolean useArms, boolean useIntakeDetecto, long startTime){
        teamUtil.log("==================== Group 5 ================");
        if (useArms) {
            shooter.setShootSpeed(B05_FLYWHEEL_VELOCITY);
            Shooter.VELOCITY_COMMANDED = B05_FLYWHEEL_VELOCITY;
            shooter.aim(B05_SHOT_PITCH);

        }
        if (getMoreBallsV2()) {
            if (useArms) robot.autoTransferAndLoadNoWait(B05_INTAKE_PAUSE, true,3000); // Rely on Loaded Detector for these
            // Drive back to shooting zone
            if (!mirroredDriveToShotPositionFast(B05_SHOT_X, B05_SHOT_Y, B05_SHOT_RH, B05_SHOT_END_VEL, B05_SHOT_STRAIGHT_PERCENT, B05_SHOT_DRIFT_PERCENT)) return false; // Try the new faster one
            //if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED, B08_SHOOT5_SETUP_X,B08_SHOOT5_SETUP_Y,B08_SHOOT5_DH, B08_SHOOT5_DH, B00_CORNER_VELOCITY, null, 0, 4000)) return;
            //if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED, B08_SHOOT5_X,B08_SHOOT5_Y,B08_SHOOT5_DH, B08_SHOOT5_RH, B08_SHOOT5_END_VEL, null, 0, 4000)) return;

            if(intake.failedOut.get()){
                teamUtil.log("Auto has FAILED OUT because of a jammed intake");
                stopRobot();
                return false;
            }
            // check for enough time to launch last 3 TODO: Make this incremental inside of driveWhileShootingPattern
            boolean enoughTime = System.currentTimeMillis() - startTime < 30000 - B05_SHOT_THRESHOLD; // check that there is enough time and we want to get more
            // shoot 5th set of balls
            if (enoughTime) {
                if (!robot.autoShootPattern(useArms, 5000))
                    //if (!driveWhileShootingPattern(useArms, teamUtil.alliance == teamUtil.Alliance.BLUE ? (B08_SHOOT4_H) : 360 - B08_SHOOT4_H, B00_SHOOT_VELOCITY, 5000))
                    return false;
            } else {
                teamUtil.log("Not enough time to launch 3. Stopping");
            }
        }

        if(System.currentTimeMillis() - startTime < 30000 - B09_PARK_TIME_CUTOFF){ // 2 seconds left
            if(!drive.mirroredMoveToYHoldingLine(B00_MAX_SPEED, B09_PARK_Y, B09_PARK_X, B09_PARK_DRIVE, 0, B09_PARK_END_VELOCITY, null, 0, 3000)) return false;
        }else{
            teamUtil.log("Not enough time to park. Stopping");
        }
        return true;
    }

    ///  ////////////////////////////////////////////////////////////////  GOAL SIDE V3
    public void goalSideV4(boolean useArms, boolean useIntakeDetector, long gateLeaveTime) {
        double nextGoalDistance = 0;
        long startTime = System.currentTimeMillis();
        double savedDeclination;
        teamUtil.log("##################################################################################");
        teamUtil.log("#########################  Starting GoalSideV3 Auto ##############################");
        shooter.flywheelStartup(); // set flywheel to fast start PIDF coefs
        intake.setLoadedArtifacts(PPG); // Assumes artifacts are preloaded in this order!!
        intake.setIntakeArtifacts(PPG);
        intake.signalArtifacts();
        // Prep Shooter
        nextGoalDistance = drive.getGoalDistance((int) B01_SHOT_X, (int) B01_SHOT_Y * (teamUtil.alliance== teamUtil.Alliance.RED ? -1 : 1));
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
            teamUtil.log("NOT using Detector. Running in hardcoded mode for patterns.");
        }


        /////////////////////////////Shoot Preloads (Group 1)
        if (!preloads(useArms)) return;

        /////////////////////////////Intake 2nd group and shoot
        if (!group2(useArms, useIntakeDetector)) return;


        /////////////////////////////Intake 3rd group, empty ramp and shoot
        if (!group3(useArms, useIntakeDetector, startTime, gateLeaveTime)) return;


        /////////////////////////////Intake 4th group and shoot
        if (!group4(useArms, useIntakeDetector)) return;

        ///////////////////////////// Intake 5th group and shoot
        if (!group5andPark(useArms, useIntakeDetector, startTime)) return;

        /////////////////////////////Wrap up
        stopRobot();
    }



    public double calculateAngleFromSecondToFirst(double x1, double y1, double x2, double y2) {
        // 1. Calculate the difference from the second point (origin) to the first
        double dx = x1 - x2;
        double dy = y1 - y2;

        // 2. Handle the edge case where both points are the same
        if (dx == 0 && dy == 0) {
            return 0.0;
        }

        // 3. Use standard atan2(dy, dx)
        // This natively treats positive x as 0° and positive y as 90°.
        double radians = Math.atan2(dy, dx);

        // 4. Convert radians to degrees
        double degrees = Math.toDegrees(radians);

        // 5. Normalize the result to a [0, 360) range.
        // atan2 returns values between -180 and 180.
        return (degrees < 0) ? (degrees + 360) : degrees;
    }

    public static double DRIVE_TO_SHOT_ROTATION_ADJUST_FACTOR = 20;
    public boolean mirroredDriveToShotPositionFast(double xTarget, double yTarget, double shotHeading, double endVelocity, double straightPercent, double driftPercent) {
        teamUtil.log("mirroredDriveToShotPositionFast");
        int currentX = drive.oQlocalizer.posX_mm;
        int currentY = drive.oQlocalizer.posY_mm;
        if (teamUtil.alliance== teamUtil.Alliance.RED) {
            currentY = currentY *-1;
        }
        double driveHeading = calculateAngleFromSecondToFirst(xTarget, yTarget, currentX, currentY);
        double straightX = (xTarget - currentX) * straightPercent + currentX;
        double straightY = (yTarget - currentY) * straightPercent + currentY;
        double driftX = (xTarget - currentX) * driftPercent + currentX;
        double driftY = (yTarget - currentY) * driftPercent + currentY;
        teamUtil.log(String.format("DH: %.1f Target: %.0f,%.0f Straight %.0f,%.0f Drift %.0f,%.0f", driveHeading, xTarget, yTarget, straightX, straightY, driftX, driftY));
        robot.blinkin.setSignal(Blinkin.Signals.NORMAL_WHITE);
        if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED, straightX, straightY, driveHeading, driveHeading,B00_CORNER_VELOCITY,null, 0, 3000 )) return false;
        robot.blinkin.setSignal(Blinkin.Signals.PURPLE);
        double store = drive.ROTATION_ADJUST_FACTOR;
        drive.ROTATION_ADJUST_FACTOR = DRIVE_TO_SHOT_ROTATION_ADJUST_FACTOR; // chill out rotation adjust for this large/quick spin/translation
        if (!drive.mirroredMoveToXHoldingLine(B00_MAX_SPEED, driftX, driftY, driveHeading, shotHeading,endVelocity,null, 0, 3000 )) return false;
        drive.ROTATION_ADJUST_FACTOR = store;
        return true;
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
        teamUtil.log("getMoreBallsV2");

        // get Intake Ready
        double stored = Intake.INTAKE_IN_POWER;
        Intake.INTAKE_IN_POWER = C02_GRAB_INTAKE_POWER; // adjust intake speed for this operation
        intake.getReadyToIntakeNoWait();

        // Drive towards wall fast
        if (!drive.mirroredMoveToXHoldingLine(C04_FAST_APPROACH_VELOCITY, C04_FAST_APPROACH_X,C04_FAST_APPROACH_Y,C04_FAST_APPROACH_DRIVE_HEADING, C04_FAST_APPROACH_ROBOT_HEADING, C04_FAST_APPROACH_END_VELOCITY, null, 0, 2100)) return false;
        intake.intakeNum = 0; // don't return instantly from grab3
        if (!grab3V2(C04_GRAB_VEL, C04_GRAB_Y_LIMIT, C04_GRAB_TIME)) return false;

        Intake.INTAKE_IN_POWER = stored; // restore intake speed default
        teamUtil.log("getMoreBallsV2 Finished");
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




}
