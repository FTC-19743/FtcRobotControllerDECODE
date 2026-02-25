package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Pattern.GPP;
import static org.firstinspires.ftc.teamcode.libs.teamUtil.Pattern.PGP;
import static org.firstinspires.ftc.teamcode.libs.teamUtil.Pattern.PPG;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
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
    public static int IDEAL_FLYWHEEL = 1570;
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

    public void stopRobot(){
        drive.stopMotors();
        intake.intakeStop();
        shooter.stopShooter();
    }

    public boolean farSideShoot3 (boolean useArms, long flyWheelPause, long timeOut) {
        teamUtil.log("farSideShoot3 Starting");
        long timeOutTime = System.currentTimeMillis() + timeOut;

        if (useArms) {
            // Make sure flywheels are spun up and heading is good
            while ((!robot.autoShooterHeadingReady() || !shooter.isLoaded() || robot.shooterFlyWheelsReady(drive.robotGoalDistance())) && teamUtil.keepGoing(timeOutTime)) {
                robot.autoHoldShotHeading();
            }
        }
        robot.drive.stopMotors();
        teamUtil.pause(flyWheelPause); // extra time to get flywheels running at ideal speed

        if (System.currentTimeMillis() >=timeOutTime) {
            teamUtil.log("farSideShoot3 TIMED OUT waiting for valid shooting conditions");
            return false;
        } else {
            return robot.autoShootSuperFast(useArms, false,5000); // Don't bother with pattern on preloads since we are going to empty the ramp
        }

    }
    ///  //////////////////////////////////////////////////////////////// Pre Loads (Group 1--"B01")
    public static double B01_SHOT_X = -1400;
    public static double B01_SHOT_Y = 400;
    public static double B01_SHOT_H = 21;
    public static double B01_SHOT_END_VEL = 300;
    public static double B01_FLYWHEEL_VEL = IDEAL_FLYWHEEL;
    public static float B01_SHOT_PITCH = IDEAL_PITCH;
    public static long B01_FLYWHEEL_PAUSE = 1000;
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

    public boolean getCornerBallsAndShoot(boolean useArms) {
        teamUtil.log("==================== getCornerBallsAndShoot ================");
        if (getMoreBalls()) {
            if (useArms) robot.autoTransferAndLoadSuperFastNoWait(B02_INTAKE_PAUSE,3000); // Rely on Loaded Detector for these
            // Drive back to shooting zone
            if (!drive.mirroredMoveToYHoldingLine(B02_OFF_WALL_VELOCITY, B02_OFF_WALL_Y,B02_OFF_WALL_X,B02_OFF_WALL_DRIVE_HEADING, B02_OFF_WALL_ROBOT_HEADING, B02_OFF_WALL_END_VELOCITY, null, 0, 2100)) return false;
            if (!drive.mirroredMoveToYHoldingLine(B02_FAST_APPROACH_VELOCITY, B02_FAST_APPROACH_Y,B02_FAST_APPROACH_X,B02_FAST_APPROACH_DRIVE_HEADING, B01_SHOT_H, B02_FAST_APPROACH_END_VELOCITY, null, 0, 2100)) return false;
            return farSideShoot3(useArms, 0, 5000);
        } else {
            return false;
        }
    }



    ///  ////////////////////////////////////////////////////////////////  GOAL SIDE V3
    public void farSideV1(boolean useArms, boolean useIntakeDetector) {
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
            teamUtil.log("NOT using Detector. Running in hardcoded mode for patterns.");
        }

        /////////////////////////////Shoot Preloads (Group 1)
        if (!preloads(useArms)) return;

        /////////////////////////////Get the corner ones and shoot (Group 2)
        getCornerBallsAndShoot(useArms);

        /////////////////////////////Park off the line


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


}
