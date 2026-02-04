package org.firstinspires.ftc.teamcode.assemblies;

import static androidx.core.math.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//import org.firstinspires.ftc.teamcode.libs.Blinkin;

import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

@Config // Makes Static data members available in Dashboard
public class Intake {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    static public boolean details = false;

    public Servo left_flipper;
    public Servo middle_flipper;
    public Servo right_flipper;

    public DcMotorEx elevator;
    public DcMotorEx intake;

    private Servo rgbLeft;
    private Servo rgbMiddle;
    private Servo rgbRight;

    private ColorSensor leftTopColorSensor;
    private ColorSensor middleTopColorSensor;
    private ColorSensor rightTopColorSensor;

    private ColorSensor leftLowerColorSensor;
    private ColorSensor middleLowerColorSensor;
    private ColorSensor rightLowerColorSensor;

    public AtomicBoolean elevatorMoving = new AtomicBoolean(false);
    public AtomicBoolean failedOut = new AtomicBoolean(false);
    public AtomicBoolean flipping = new AtomicBoolean(false);

    public AtomicBoolean detecting = new AtomicBoolean(false);
    public AtomicBoolean stopDetector = new AtomicBoolean(false);

    public enum Location{NONE, LEFT, CENTER, RIGHT}; // correspond to the flippers (reversed if facing the robot)
    public enum ARTIFACT {NONE, GREEN, PURPLE};
    public static ARTIFACT leftLoad = ARTIFACT.NONE,rightLoad = ARTIFACT.NONE,middleLoad = ARTIFACT.NONE, leftIntake = ARTIFACT.NONE, middleIntake = ARTIFACT.NONE, rightIntake = ARTIFACT.NONE;
    public int intakeNum = 0;


    static public double ELEVATOR_CALIBRATE_POWER = -0.1;
    static public int ELEVATOR_GROUND = 5;
    static public long ELEVATOR_PAUSE_1 = 300;
    static public long ELEVATOR_PAUSE_2 = 500;
    public static float ELEVATOR_UP_POWER = .5f;
    public static float ELEVATOR_DOWN_POWER = -.5f;
    public static int ELEVATOR_UP_TIMEOUT = 1500;
    public static int ELEVATOR_DOWN_TIMEOUT = 1500;
    public static int ELEVATOR_UP_VELOCITY_THRESHOLD = 100;
    public static int ELEVATOR_DOWN_VELOCITY_THRESHOLD = -100;

    public static int ELEVATOR_REVERSE_INTAKE_ENCODER = 85;
    public static int ELEVATOR_UP_ENCODER = 135;
    public static int ELEVATOR_DOWN_ENCODER = 50;
    public static int ELEVATOR_UNLOAD_ENCODER = 166;
    public static int ELEVATOR_HOLD_VELOCITY = 1500;
    public static long ELEVATOR_STARTUP_TIME = 250;

    static public double INTAKE_IN_POWER = 0.7;
    static public double INTAKE_OUT_POWER = -0.7;

    static public float FLIPPER_PRE_TRANSFER = 0.94f;
    static public float FLIPPER_CEILING = 0.81f;
    static public float FLIPPER_CEILING_MIDDLE = 0.77f;
    static public float FLIPPER_TRANSFER = 0.7f;
    static public float MIDDLE_FLIPPER_SHOOTER_TRANSFER = 0.05f;
    static public float EDGE_FLIPPER_SHOOTER_TRANSFER = 0.15f;
    public static long FLIPPER_UNLOAD_PAUSE = 400;
    public static Location pinned;

    public Intake() {
        teamUtil.log("Constructing Intake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initialize() {
        teamUtil.log("Initializing Intake");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        elevator = hardwareMap.get(DcMotorEx.class, "elevator");

        left_flipper = hardwareMap.get(Servo.class,"leftflipper");
        right_flipper = hardwareMap.get(Servo.class,"rightflipper");
        middle_flipper = hardwareMap.get(Servo.class,"middleflipper");

        rgbLeft = hardwareMap.get(Servo.class,"rgbleft");
        rgbMiddle = hardwareMap.get(Servo.class,"rgbmiddle");
        rgbRight = hardwareMap.get(Servo.class,"rgbright");

//        leftTopColorSensor = hardwareMap.get(RevColorSensorV3.class, "upperleftcolorsensor");
//        middleTopColorSensor = hardwareMap.get(RevColorSensorV3.class, "uppermiddlecolorsensor");
//        rightTopColorSensor = hardwareMap.get(RevColorSensorV3.class, "upperrightcolorsensor");
//
//        leftLowerColorSensor = hardwareMap.get(ColorSensor.class, "lowerleftcolorsensor");
//        middleLowerColorSensor = hardwareMap.get(ColorSensor.class, "lowermiddlecolorsensor");
//        rightLowerColorSensor = hardwareMap.get(ColorSensor.class, "lowerrightcolorsensor");

        teamUtil.log("Intake Initialized");
    }

    public void calibrate() {
        teamUtil.log("Calibrating Intake");
        calibrateElevators();

    }

    public void calibrateElevators(){
        teamUtil.log("Calibrating elevators Only");
        elevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevator.setPower(ELEVATOR_CALIBRATE_POWER);
        int lastelevatorPosition = elevator.getCurrentPosition();
        teamUtil.pause(250);
        while (elevator.getCurrentPosition() != lastelevatorPosition) { // TODO Add timeout and keepgoing check
            lastelevatorPosition = elevator.getCurrentPosition();
            if (details) teamUtil.log("Calibrate Intake: elevator: " + elevator.getCurrentPosition());
            teamUtil.pause(50);
        }
        elevator.setPower(0);
        teamUtil.pause(500); // let it "relax" just a bit
        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //elevator.setTargetPositionTolerance(ELEVATOR_TOLERANCE_RETRACT);// make that our zero position
        elevator.setTargetPosition(ELEVATOR_GROUND);
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setVelocity(0);

        teamUtil.log("Calibrate Intake Final: elevator: "+elevator.getCurrentPosition());
    }

    public void intakeTelemetry() {
        telemetry.addLine("Intake elevator Position: " + elevator.getCurrentPosition());
        //telemetry.addData("UpperSensors(A/R/G/B): ", "L%s M%s R%s",formatSensor(leftTopColorSensor), formatSensor(middleTopColorSensor), formatSensor(rightTopColorSensor));
        //telemetry.addData("LowerSensors(A/R/G/B): ", "L%s M%s R%s",formatSensor(leftLowerColorSensor), formatSensor(middleLowerColorSensor), formatSensor(rightLowerColorSensor));
        //detectLoadedArtifacts();
        //checkIntakeArtifacts();
        telemetry.addLine("Loaded: L:" + leftLoad.toString() + "  M:" + middleLoad.toString() + "  R:" + rightLoad.toString());
        telemetry.addLine("Intake: Num:" + intakeNum + " L:" + leftIntake.toString() + "  M:" + middleIntake.toString() + "  R:" + rightIntake.toString());
    }

    public boolean servoPositionIs(Servo servo, float pos) {
        return Math.abs(servo.getPosition()-pos)<.001;
    }

    private void intakeIn(){
        intake.setPower(INTAKE_IN_POWER);
    }

    public void intakeStart(){
        teamUtil.log("intakeStart");
        intakeIn();
        startDetector();
        detectorMode = DETECTION_MODE.INTAKE;
        flippersToCeiling();
    }

    public void intakeOut(){
        intake.setPower(INTAKE_OUT_POWER);
    }

    public void intakeStop(){
        intake.setPower(0);
    }

    public void flippersToCeiling(){
        left_flipper.setPosition(FLIPPER_CEILING);
        right_flipper.setPosition(FLIPPER_CEILING);
        middle_flipper.setPosition(FLIPPER_CEILING_MIDDLE);
    }
    public void flippersToTransfer() {
        left_flipper.setPosition(FLIPPER_TRANSFER);
        right_flipper.setPosition(FLIPPER_TRANSFER);
        middle_flipper.setPosition(FLIPPER_TRANSFER);
    }

    public void setLoadedArtifacts(teamUtil.Pattern pattern) {
        leftLoad = (pattern == teamUtil.Pattern.PPG || pattern == teamUtil.Pattern.PGP) ? ARTIFACT.PURPLE : ARTIFACT.GREEN;
        middleLoad = (pattern == teamUtil.Pattern.PPG || pattern == teamUtil.Pattern.GPP) ? ARTIFACT.PURPLE : ARTIFACT.GREEN;
        rightLoad = (pattern == teamUtil.Pattern.PGP || pattern == teamUtil.Pattern.GPP) ? ARTIFACT.PURPLE : ARTIFACT.GREEN;
    }

    public void setLoadedArtifacts(ARTIFACT left, ARTIFACT middle, ARTIFACT right) {
        leftLoad = left;
        middleLoad = middle;
        rightLoad = right;
    }

    public void setIntakeArtifacts(teamUtil.Pattern pattern) {
        leftIntake = (pattern == teamUtil.Pattern.PPG || pattern == teamUtil.Pattern.PGP) ? ARTIFACT.PURPLE : ARTIFACT.GREEN;
        middleIntake = (pattern == teamUtil.Pattern.PPG || pattern == teamUtil.Pattern.GPP) ? ARTIFACT.PURPLE : ARTIFACT.GREEN;
        rightIntake = (pattern == teamUtil.Pattern.PGP || pattern == teamUtil.Pattern.GPP) ? ARTIFACT.PURPLE : ARTIFACT.GREEN;
    }

    public void setIntakeArtifacts(ARTIFACT left, ARTIFACT middle, ARTIFACT right) {
        leftIntake = left;
        middleIntake = middle;
        rightIntake = right;
    }


    public int numBallsInFlippers(){
        int loadedBalls = 0;
        if(leftLoad != ARTIFACT.NONE){
            loadedBalls++;
        }
        if(middleLoad != ARTIFACT.NONE){
            loadedBalls++;
        }
        if(rightLoad != ARTIFACT.NONE){
            loadedBalls++;
        }
        return loadedBalls;
    }

    // Unload the specified servo and prepare next one (if specified) to unload .
    // If the ARTIFACT is still in the intake, it will flip the specified servo to unload, wait and then bring it back to ceiling
    // If the ARTIFACT was previously pinned in the shooter, it will release it and move the flipper back to ceiling
    // if nextLocation is not NONE, that servo will pin its ARTIFACT and leave it in that state to be released later
    // Load state is updated to NONE when ARTIFACT is fully released
    // Long running operation! Use in separate thread if you need control!
    public void unloadFlipper(Location location, Location nextLocation) {
        teamUtil.log("unloadFlipper location: " + location+ " nextLocation: "+nextLocation);
        switch (location) {
            case LEFT:
                if (nextLocation==Location.RIGHT) {
                    right_flipper.setPosition(EDGE_FLIPPER_SHOOTER_TRANSFER);
                    pinned = nextLocation;
                }
                if (servoPositionIs(left_flipper,FLIPPER_TRANSFER)) {
                    left_flipper.setPosition(EDGE_FLIPPER_SHOOTER_TRANSFER);
                    leftLoad = ARTIFACT.NONE;
                    teamUtil.pause(FLIPPER_UNLOAD_PAUSE);
                    pinned = Location.NONE;
                }
                left_flipper.setPosition(FLIPPER_CEILING);
                leftLoad = ARTIFACT.NONE;


                break;
            case RIGHT:
                if (nextLocation==Location.LEFT) {
                    left_flipper.setPosition(EDGE_FLIPPER_SHOOTER_TRANSFER);
                    pinned = nextLocation;
                }
                if (servoPositionIs(right_flipper,FLIPPER_TRANSFER)) {
                    right_flipper.setPosition(EDGE_FLIPPER_SHOOTER_TRANSFER);
                    rightLoad = ARTIFACT.NONE;
                    teamUtil.pause(FLIPPER_UNLOAD_PAUSE);
                    pinned = Location.NONE;
                }
                right_flipper.setPosition(FLIPPER_CEILING);
                rightLoad = ARTIFACT.NONE;


                break;
            case CENTER:
                if (nextLocation==Location.LEFT) {
                    left_flipper.setPosition(EDGE_FLIPPER_SHOOTER_TRANSFER);
                    pinned = nextLocation;
                }
                if (nextLocation==Location.RIGHT) {
                    right_flipper.setPosition(EDGE_FLIPPER_SHOOTER_TRANSFER);
                    pinned = nextLocation;
                }
                if (servoPositionIs(middle_flipper,FLIPPER_TRANSFER)) {
                    middle_flipper.setPosition(MIDDLE_FLIPPER_SHOOTER_TRANSFER);
                    middleLoad = ARTIFACT.NONE;
                    teamUtil.pause(FLIPPER_UNLOAD_PAUSE);
                    pinned = Location.NONE;
                }
                middle_flipper.setPosition(FLIPPER_CEILING_MIDDLE);
                break;
            default:
        }

    }


    public static long FAST3_UNLOAD_PAUSE = 400;
    public static long FAST3_LEFT_ROLL_PAUSE = 600; // use big number for timing based stuff.  Set to small number if relying on shooter detector
    public static long FAST3_RIGHT_ROLL_PAUSE = 600;

    public void superFastUnload(boolean leftLoaded, boolean middleLoaded, boolean rightLoaded) {
        flipping.set(true);
        teamUtil.robot.shooter.sidePushersStow(); // in case previous operation left them in the way
        middle_flipper.setPosition(MIDDLE_FLIPPER_SHOOTER_TRANSFER); // Flip middle
        left_flipper.setPosition(EDGE_FLIPPER_SHOOTER_TRANSFER); // flip and pin left
        right_flipper.setPosition(EDGE_FLIPPER_SHOOTER_TRANSFER); // flip and pin right
        teamUtil.pause(FAST3_UNLOAD_PAUSE);
        middle_flipper.setPosition(FLIPPER_CEILING_MIDDLE); // release middle
        if (!middleLoaded && rightLoaded) {
            right_flipper.setPosition(FLIPPER_CEILING); // release right and give it a head start on the left
            teamUtil.pause(FAST3_RIGHT_ROLL_PAUSE);
        }
        left_flipper.setPosition(FLIPPER_CEILING); // release left
        right_flipper.setPosition(FLIPPER_CEILING); // release right
        teamUtil.robot.shooter.sidePushersHold();

        flipping.set(false);
        teamUtil.log("superFastUnload Finished");
    }
    public void superFastUnloadNoWait(boolean leftLoaded, boolean middleLoaded, boolean rightLoaded) {
        teamUtil.log("Launching Thread to superFastUnload");
        if (flipping.get()) {
            teamUtil.log("WARNING: superFastUnload called while flipping--Ignored");
            return;
        }
        flipping.set(true);
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                superFastUnload( leftLoaded,  middleLoaded,  rightLoaded);
            }
        });
        thread.start();
    }

/* Older
    public void fastUnloadStep1() {
        flipping.set(true);
        middle_flipper.setPosition(MIDDLE_FLIPPER_SHOOTER_TRANSFER); // Flip middle
        left_flipper.setPosition(EDGE_FLIPPER_SHOOTER_TRANSFER); // flip and pin left
        right_flipper.setPosition(EDGE_FLIPPER_SHOOTER_TRANSFER); // flip and pin right
        teamUtil.pause(FAST3_UNLOAD_PAUSE);
        middle_flipper.setPosition(FLIPPER_CEILING_MIDDLE); // release middle
        left_flipper.setPosition(FLIPPER_CEILING); // release left
        middleLoad = ARTIFACT.NONE;
        leftLoad = ARTIFACT.NONE;
        flipping.set(false);
        teamUtil.log("fastUnloadStep1 Finished");
    }

    public void fastUnloadStep1NoWait() {
        teamUtil.log("Launching Thread to fastUnloadStep1");
        if (flipping.get()) {
            teamUtil.log("WARNING: fastUnloadStep1NoWait called while flipping--Ignored");
            return;
        }
        flipping.set(true);
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                fastUnloadStep1();
            }
        });
        thread.start();
    }

    public void fastUnloadStep2() {
        right_flipper.setPosition(FLIPPER_CEILING); // release right
        rightLoad = ARTIFACT.NONE;
    }

    public static long middleFlipToSensor = 0;
    public static long outsideFlipToSensor = 800;

    // Fast transfer from flipper to shooter without worrying about colors
    public long flipNextFastInternal(){
        if(middleLoad != ARTIFACT.NONE){
            middle_flipper.setPosition(MIDDLE_FLIPPER_SHOOTER_TRANSFER); // Flip middle
            if(leftLoad != ARTIFACT.NONE){
                left_flipper.setPosition(EDGE_FLIPPER_SHOOTER_TRANSFER); // flip and pin left
            }
            if(rightLoad != ARTIFACT.NONE) {
                right_flipper.setPosition(EDGE_FLIPPER_SHOOTER_TRANSFER); // flip and pin right
            }
            teamUtil.pause(FLIPPER_UNLOAD_PAUSE);
            middle_flipper.setPosition(FLIPPER_CEILING_MIDDLE); // release middle
            middleLoad = ARTIFACT.NONE;
            return middleFlipToSensor;
            // Should we release the left or right at this point?
        }else{
            if (servoPositionIs(left_flipper,EDGE_FLIPPER_SHOOTER_TRANSFER)) { // left already pinned
                left_flipper.setPosition(FLIPPER_CEILING); // release left
                leftLoad = ARTIFACT.NONE;
                return outsideFlipToSensor;
            }else if(servoPositionIs(right_flipper,EDGE_FLIPPER_SHOOTER_TRANSFER)){ // right already pinned
                right_flipper.setPosition(FLIPPER_CEILING); // release right
                rightLoad = ARTIFACT.NONE;
                return outsideFlipToSensor;

            }else{ // Nothing in middle and nothing pinned
                if(leftLoad != ARTIFACT.NONE){
                    left_flipper.setPosition(EDGE_FLIPPER_SHOOTER_TRANSFER); // flip left
                    if (rightLoad != ARTIFACT.NONE) {
                        right_flipper.setPosition(EDGE_FLIPPER_SHOOTER_TRANSFER); // pin right
                    }
                    teamUtil.pause(FLIPPER_UNLOAD_PAUSE);
                    left_flipper.setPosition(FLIPPER_CEILING); // release left
                    leftLoad = ARTIFACT.NONE;
                    return outsideFlipToSensor;

                }else if(rightLoad != ARTIFACT.NONE) {
                    right_flipper.setPosition(EDGE_FLIPPER_SHOOTER_TRANSFER); // flip right
                    rightLoad = ARTIFACT.NONE;
                    teamUtil.pause(FLIPPER_UNLOAD_PAUSE);
                    right_flipper.setPosition(FLIPPER_CEILING); // release right
                    rightLoad = ARTIFACT.NONE;
                    return outsideFlipToSensor;

                }else{
                    return middleFlipToSensor; //nothing so 0
                }
            }
        }
    }

    public void flipNextFast(){
        flipping.set(true);
        long timeOutTime = System.currentTimeMillis()+2500;
        while(ballsLeftToShoot() && teamUtil.keepGoing(timeOutTime)){
            teamUtil.log("Attempting to load shooter");
            long detectTime = System.currentTimeMillis() + flipNextFastInternal();
            while(!teamUtil.robot.shooter.isLoaded() && teamUtil.keepGoing(detectTime)){
                teamUtil.pause(20);
            }
            if(teamUtil.robot.shooter.isLoaded()){
                teamUtil.log("Shooter Successfully loaded");
                flipping.set(false);
                return;
            } else {
                teamUtil.log("WARNING: flipNextFast: Unable to load shooter, moving to next shot");
            }
        }
        if(!teamUtil.keepGoing(timeOutTime)){
            teamUtil.log("flipNextFast timed out");
        }else {
            teamUtil.log("WARNING: flipNextFast: Unable to load any shots, giving up");
        }
        flipping.set(false);
    }

    public void flipNextFastNoWait() {
        teamUtil.log("Launching Thread to flipNextFast.");
        if (flipping.get()) {
            teamUtil.log("WARNING: flipNextFastNoWait called while flipping--Ignored");
            return;
        }
        flipping.set(true);
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                flipNextFast();
            }
        });
        thread.start();
    }
*/

    public boolean ballsLeftToShoot(){
        //teamUtil.log("left: "+left_flipper.getPosition()+", middle: "+middle_flipper.getPosition()+", right: "+right_flipper.getPosition()+", balls: "+numBallsInFlippers());
        return servoPositionIs(left_flipper, EDGE_FLIPPER_SHOOTER_TRANSFER) ||
                servoPositionIs(middle_flipper, MIDDLE_FLIPPER_SHOOTER_TRANSFER) ||
                servoPositionIs(right_flipper, EDGE_FLIPPER_SHOOTER_TRANSFER) ||
                numBallsInFlippers() > 0;
    }

    public boolean elevatorToGroundV2() {
        teamUtil.log("elevatorToGroundV2.");
        failedOut.set(false);
        elevatorMoving.set(true);
        if (elevator.getCurrentPosition() <= 5) {
            teamUtil.log("WARNING: elevatorToGroundV2 called while elevator already at bottom--Ignored");
            elevatorMoving.set(false);
            return true;
        }
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setPower(ELEVATOR_DOWN_POWER);
        long timeOutTime = System.currentTimeMillis()+ELEVATOR_DOWN_TIMEOUT;
        teamUtil.pause(250); // allow time for elevator to get moving
        while(teamUtil.keepGoing(timeOutTime) && elevator.getVelocity() < ELEVATOR_DOWN_VELOCITY_THRESHOLD && elevator.getCurrentPosition()>ELEVATOR_DOWN_ENCODER){
            if (details) {
                teamUtil.log("Elevator Pos: "+ elevator.getCurrentPosition() + " Vel: "+ elevator.getVelocity());
            }
        }
        if (elevator.getCurrentPosition() > ELEVATOR_DOWN_ENCODER) {
            // We ran into an issue so fail out
            teamUtil.log("WARNING: elevatorToGroundV2 Failed Out due to stall or timeout");
            elevator.setPower(0);
            elevatorMoving.set(false);
            failedOut.set(true);
            return false;
        }

        // Run the last bit at low power and stall against mechanical block
        elevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevator.setPower(ELEVATOR_CALIBRATE_POWER);
        int lastelevatorPosition = elevator.getCurrentPosition();
        teamUtil.pause(250);
        while (teamUtil.keepGoing(timeOutTime) && elevator.getCurrentPosition() != lastelevatorPosition) {
            lastelevatorPosition = elevator.getCurrentPosition();
            if (details) teamUtil.log("dropping Intake: elevator: " + elevator.getCurrentPosition());
            teamUtil.pause(50);
        }
        elevator.setPower(0);
        elevatorMoving.set(false);

        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("elevatorToGroundV2 TIMED OUT!!!!!!!!!!!!");
            failedOut.set(true);
            return false;
        } else {
            teamUtil.log("elevatorToGroundV2 Finished");
            return true;
        }
    }

    public void elevatorToGroundV2NoWait () {
        if (elevatorMoving.get()) {
            teamUtil.log("WARNING: elevatorToGroundV2NoWait called while moving--Ignored");
            return;
        }
        elevatorMoving.set(true);
        failedOut.set(false);
        teamUtil.log("Launching Thread to elevatorToGroundV2NoWait.");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                elevatorToGroundV2();
            }
        });
        thread.start();
    }

    public boolean getReadyToIntake() {
        teamUtil.log("getReadyToIntake");

        // Move flippers into out of the way position to drop any artifacts we might be holding
        left_flipper.setPosition(FLIPPER_PRE_TRANSFER);
        right_flipper.setPosition(FLIPPER_PRE_TRANSFER);
        middle_flipper.setPosition(FLIPPER_PRE_TRANSFER);
        teamUtil.pause(500); // Allow time for drop

        // Move elevator to ground
        if( elevatorToGroundV2()) {
            // Move flippers into blocking position
            flippersToCeiling();

            intakeIn();
            startDetector();
            detectorMode = DETECTION_MODE.INTAKE;

            teamUtil.log("getReadyToIntake Finished");
            return true;
        } else {
            teamUtil.log("getReadyToIntake Finished But Elevator to Ground V2 Failed");
            return false;
        }
    }

    public void getReadyToIntakeNoWait(){
        if (elevatorMoving.get()) {
            teamUtil.log("WARNING: getReadyToIntake called while moving--Ignored");
            return;
        }
        elevatorMoving.set(true);
        failedOut.set(false);
        teamUtil.log("Launching Thread to getReadyToIntake.");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                getReadyToIntake();
            }
        });
        thread.start();
    }


    // Transfer from ground to flippers and preload for fast shots
    // Does not worry about colors
    public void elevatorToShooterFast(boolean detectLoaded){
        teamUtil.log("elevatorToShooterFast");
        if(leftIntake == ARTIFACT.NONE && middleIntake == ARTIFACT.NONE && rightIntake == ARTIFACT.NONE){
            teamUtil.log("WARNING: elevatorToShooterFast called without loaded artifacts");
            elevatorMoving.set(false);
            failedOut.set(true);
            return;
        }
        if(elevatorToFlippersV2(false, detectLoaded)){
            superFastUnloadNoWait(leftLoad!= ARTIFACT.NONE,middleLoad!= ARTIFACT.NONE,rightLoad!= ARTIFACT.NONE);
        }
        teamUtil.log("elevatorToShooterFast finished");
        elevatorMoving.set(false);
    }

    public void elevatorToShooterFastNoWait(boolean detectLoaded){
        if (elevatorMoving.get()) {
            teamUtil.log("WARNING: elevatorToShooterNoWait called while elevatorMoving is true--Ignored");
            return;
        }
        elevatorMoving.set(true);
        failedOut.set(false);
        teamUtil.log("Launching Thread to elevatorToShooterNoWait.");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                elevatorToShooterFast(detectLoaded);
            }
        });
        thread.start();
    }

    // Transfer from ground to flippers and optionally wait for elevator to get back to ground before returning
    // Returns false if the elevator stalls or times out
    public boolean elevatorToFlippersV2(boolean waitForGround, boolean detectLoaded){
        teamUtil.log("elevatorToFlippersV2NoWait");

        failedOut.set(false);
        elevatorMoving.set(true);
        if (elevator.getCurrentPosition() > ELEVATOR_DOWN_ENCODER) {
            teamUtil.log("WARNING: elevatorToFlippersV2 called while elevator not at bottom--Ignored");
            elevatorMoving.set(false);
            return true;
        }


        // "transfer" the sensor readings to the loaded level, this is a backup for the loaded detector
        setLoadedArtifacts(leftIntake, middleIntake, rightIntake);

        // Move flippers out of the way
        left_flipper.setPosition(FLIPPER_PRE_TRANSFER);
        right_flipper.setPosition(FLIPPER_PRE_TRANSFER);
        middle_flipper.setPosition(FLIPPER_PRE_TRANSFER);
        teamUtil.pause(ELEVATOR_PAUSE_1);

        // Move elevator up while checking for stall (jam) or timeout
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setPower(ELEVATOR_UP_POWER);
        long timeOutTime = System.currentTimeMillis()+ELEVATOR_UP_TIMEOUT;
        teamUtil.pause(ELEVATOR_STARTUP_TIME); // allow time for elevator to get moving
        while(teamUtil.keepGoing(timeOutTime) && elevator.getVelocity() > ELEVATOR_UP_VELOCITY_THRESHOLD && elevator.getCurrentPosition()<ELEVATOR_UP_ENCODER){
            if (details) {
                teamUtil.log("Elevator Pos: "+ elevator.getCurrentPosition() + " Vel: "+ elevator.getVelocity());
            }
            // If the balls are past the intake wheel, reverse the intake to avoid pulling in more
            if (elevator.getCurrentPosition() > ELEVATOR_REVERSE_INTAKE_ENCODER) {
                intakeOut();
            }
        }
        if (elevator.getCurrentPosition() < ELEVATOR_UP_ENCODER) {
            // We ran into an issue so fail out
            teamUtil.log("elevatorToFlippersV2 Failed Out due to time out or stall");
            setLoadedArtifacts(ARTIFACT.NONE, ARTIFACT.NONE, ARTIFACT.NONE);

            elevator.setPower(0);
            stopDetector();
            elevatorMoving.set(false);
            failedOut.set(true);
            return false;
        }

        // Hold at unload position
        elevator.setTargetPosition(ELEVATOR_UNLOAD_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setVelocity(ELEVATOR_HOLD_VELOCITY);
        elevatorMoving.set(false);

        // Load balls into flippers
        left_flipper.setPosition(FLIPPER_TRANSFER);
        right_flipper.setPosition(FLIPPER_TRANSFER);
        middle_flipper.setPosition(FLIPPER_TRANSFER);
        if(detectLoaded) detectorMode = DETECTION_MODE.LOADED;
        teamUtil.pause(ELEVATOR_PAUSE_2);

        if (waitForGround) {
            boolean success =  elevatorToGroundV2();
            if (detectLoaded) detectLoadedArtifactsV2();
            signalArtifacts();
            //stopDetector();
            teamUtil.log("elevatorToFlippersV2 Finished");
            return success;
        } else {
            elevatorToGroundV2NoWait();
            if (detectLoaded) detectLoadedArtifactsV2();
            signalArtifacts();
            //stopDetector();
            teamUtil.log("elevatorToFlippersV2 Finished while elevator returning to ground");
            return true;
        }
    }

    public void elevatorToFlippersV2NoWait(boolean detectLoaded){
        if (elevatorMoving.get()) {
            teamUtil.log("WARNING: elevatorToFlippersV2NoWait called while moving--Ignored");
            return;
        }
        elevatorMoving.set(true);
        failedOut.set(false);
        teamUtil.log("Launching Thread to elevatorToFlippersV2NoWait.");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                elevatorToFlippersV2(true, detectLoaded);
            }
        });
        thread.start();
    }


    /// ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    // RGB Signals

    public static float rgbGREEN = 0.5f;
    public static float rgbPURPLE = 0.7f;
    public static float rbgOFF = 0f;

    public void setRGBsOff() {
        rgbLeft.setPosition(rbgOFF);
        rgbMiddle.setPosition(rbgOFF);
        rgbRight.setPosition(rbgOFF);

    }
    public void setRGBSignal(Servo rgb, ARTIFACT artifact) {
        switch (artifact) {
            case GREEN: rgb.setPosition(rgbGREEN); break;
            case PURPLE: rgb.setPosition(rgbPURPLE); break;
            default: rgb.setPosition(rbgOFF); break;
        }
    }
    public void setRGBSignals(ARTIFACT left, ARTIFACT middle, ARTIFACT right) {
        setRGBSignal(rgbLeft, left);
        setRGBSignal(rgbMiddle, middle);
        setRGBSignal(rgbRight, right);
    }
    public void signalArtifacts () {
        if (detectorMode == DETECTION_MODE.INTAKE) {
            setRGBSignals(leftIntake, middleIntake, rightIntake);
        } else if (detectorMode == DETECTION_MODE.LOADED)  {
            setRGBSignals(leftLoad, middleLoad, rightLoad);
        } else {
            setRGBSignals(leftLoad, middleLoad, rightLoad); // Default to loaded status, which is currently only set by elevatorToFlippersV2()
            //setSignals(ARTIFACT.NONE, ARTIFACT.NONE, ARTIFACT.NONE);
        }
    }

    /// ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Limelight based detector code

    public enum DETECTION_MODE {NONE, INTAKE, LOADED};
    public DETECTION_MODE detectorMode = DETECTION_MODE.NONE;
    public static boolean DETECTOR_FAILSAFE = false;
    public static boolean KEEP_INTAKE_DETECTOR_SNAPSCRIPT_RUNNING = true;

    // Tell the limelight where to look for ARTIFACTS
    public boolean startDetector() {
        if (teamUtil.robot.limeLightActive()) {
            teamUtil.log("WARNING: startDetector called while limelight active.  Ignored.");
        } else {
            teamUtil.log("startDetector: Result: " + teamUtil.robot.startLimeLightPipeline(Robot.PIPELINE_INTAKE));
        }
        return true;
    }

    public void stopDetector() {
        teamUtil.log("stopDetector");
        detectorMode = DETECTION_MODE.NONE;
        if (KEEP_INTAKE_DETECTOR_SNAPSCRIPT_RUNNING) {
            teamUtil.log("stopDetector: Leaving Limelight Detector pipeline running");
        } else {
            teamUtil.log("stopDetector: Result: " + teamUtil.robot.stopLimeLight());
        }
    }

    public static long LL_RESET_PAUSE = 50;
    public boolean resetIntakeDetector() {
        teamUtil.log("resetIntakeDetector");
        boolean stored = KEEP_INTAKE_DETECTOR_SNAPSCRIPT_RUNNING;
        KEEP_INTAKE_DETECTOR_SNAPSCRIPT_RUNNING = false; // force stop of limelight
        stopDetector();
        teamUtil.pause(LL_RESET_PAUSE);
        startDetector();
        detectorMode = DETECTION_MODE.INTAKE;
        KEEP_INTAKE_DETECTOR_SNAPSCRIPT_RUNNING = stored; // restore previous value
        return true;
    }
    private long lastDetectorWarningTime = 0;
    private long lastDetectorWarningInterval = 100;

    private double[]  getDetectorOutput() {
        LLResult result = teamUtil.robot.limelight.getLatestResult();
        //teamUtil.log("result: " + result);

        if (result == null /*|| !result.isValid() */) { // isValid() returns *false* on data returned from SnapScript, so don't use it
            if (System.currentTimeMillis() > lastDetectorWarningTime+lastDetectorWarningInterval) {
                teamUtil.log("ERROR: getDetectorOutput Failed to get latest results from Limelight");
                lastDetectorWarningTime = System.currentTimeMillis();
            }
            return null;
        }
        double[] llOutput = result.getPythonOutput();
        if (llOutput == null || llOutput.length < 5) {
            if (System.currentTimeMillis() > lastDetectorWarningTime+lastDetectorWarningInterval) {
                teamUtil.log("ERROR: getDetectorOutput got Bad data back from SnapScript on Limelight");
                lastDetectorWarningTime = System.currentTimeMillis();
            }
            return null;
        }
        int mode = (int) Math.round( llOutput[0]);
        if (mode != 23) {
            if (System.currentTimeMillis() > lastDetectorWarningTime+lastDetectorWarningInterval) {
                teamUtil.log("WARNING: Limelight Detector Pipeline returned data not valid");
                lastDetectorWarningTime = System.currentTimeMillis();
            }
            if (DETECTOR_FAILSAFE) {
                teamUtil.log("Attempting to restart Limelight Intake Detector with result: " + teamUtil.robot.startLimeLightPipeline(Robot.PIPELINE_INTAKE));
            }
            return null;
        }
        return llOutput;
    }

    private ARTIFACT getArtifactColor(double code) {
        int codeInt = (int) Math.round(code);
        if (codeInt == 1) return ARTIFACT.GREEN;
        else if (codeInt == 2) return ARTIFACT.PURPLE;
        else return ARTIFACT.NONE;
    }

    public void logDetectorOutput() {
        double[] llOutput = getDetectorOutput();
        if (llOutput == null) {
            return;
        }
        teamUtil.log("Detector Intake " + getArtifactColor(llOutput[2]) + "/" + getArtifactColor(llOutput[3]) + "/" + getArtifactColor(llOutput[4]) +
                "  Loaded: " + getArtifactColor(llOutput[5]) + "/" + getArtifactColor(llOutput[6]) + "/" + getArtifactColor(llOutput[7]));
    }

    public boolean detectIntakeArtifactsV2() {
        double[] llOutput = getDetectorOutput();
        if (llOutput == null){
            return false;
        }
        //teamUtil.log("Detect Intake worked...Codes: " + llOutput[2] +", "+ llOutput[3]+", "+ llOutput[4]);
        leftIntake = getArtifactColor(llOutput[2]);
        middleIntake = getArtifactColor(llOutput[3]);
        rightIntake = getArtifactColor(llOutput[4]);
        intakeNum = (leftIntake == ARTIFACT.NONE ? 0 : 1) + (middleIntake == ARTIFACT.NONE ? 0 : 1) + (rightIntake == ARTIFACT.NONE ? 0 : 1);
        return true;
    }

    public boolean detectLoadedArtifactsV2() {
        double[] llOutput = getDetectorOutput();
        if (llOutput == null){
            return false;
        }
        //teamUtil.log("Detect Loaded worked...Codes: " + llOutput[5] +", "+ llOutput[6]+", "+ llOutput[7]);
        leftLoad = getArtifactColor(llOutput[5]);
        middleLoad = getArtifactColor(llOutput[6]);
        rightLoad = getArtifactColor(llOutput[7]);
        return true;
    }



    ///  ////////////////////////////////////////////////////////////////////////////////
    // Old Color Sensor detection code
    /*
    private String formatSensor (ColorSensor sensor) {
        return String.format ("(%d/%d/%d/%d)",sensor.alpha(), sensor.red(), sensor.green(), sensor.blue());
    }
    public static double FLIPPERS_UNLOAD = 0.5;
    public static int UNLOAD_PAUSE = 1000;
    public static int SHORT_UNLOAD_PAUSE = 1000;

    public Servo onTheBackburner;
    public void unloadToShooter(boolean ordered){ // need to check if the artifacts are loaded before sending them to the shooter
        unloadOrder()[0].setPosition(FLIPPERS_UNLOAD);
        if(unloadOrder()[0] == middle_flipper){teamUtil.pause(SHORT_UNLOAD_PAUSE);
        }else{teamUtil.pause(UNLOAD_PAUSE);}
        unloadOrder()[1].setPosition(FLIPPERS_UNLOAD);
        onTheBackburner = unloadOrder()[2];
    }

    public Servo[] unloadOrder(){
        //need to implement
        Servo[] arr = {left_flipper, middle_flipper, right_flipper};
        return arr;
    }

    public static int UPPER_ALPHA_THRESHOLD = 25;
    public static double GREEN_THRESHOLD = 1.25;
    public static int LEFT_ALPHA_THRESHOLD = 52;

    public ARTIFACT detectLoadedArtifact(ColorSensor sensor) {
        if(sensor.alpha() < UPPER_ALPHA_THRESHOLD){return ARTIFACT.NONE;}
        if(sensor == leftTopColorSensor && sensor.alpha() < LEFT_ALPHA_THRESHOLD){return ARTIFACT.NONE;}
        if((float) sensor.green()/ (float) sensor.alpha() > GREEN_THRESHOLD){return ARTIFACT.GREEN;}
        return ARTIFACT.PURPLE;
    }

    public static int LOWER_ALPHA_THRESHOLD = 75;

    public ARTIFACT checkIntakeArtifact(ColorSensor sensor) {
        int color = sensor.argb();
        int alpha = (color >> 24) & 0xFF;
        int red   = (color >> 16) & 0xFF;
        int green = (color >> 8)  & 0xFF;
        int blue  =  color        & 0xFF;
        if(sensor.alpha() < LOWER_ALPHA_THRESHOLD){return ARTIFACT.NONE;}
        if (sensor.red() > sensor.green() || sensor.blue() > sensor.green()) {return ARTIFACT.PURPLE;}
        return ARTIFACT.GREEN;
    }

    public void detectLoadedArtifacts() {
        leftLoad = detectLoadedArtifact(leftTopColorSensor);
        middleLoad = detectLoadedArtifact(middleTopColorSensor);
        rightLoad = detectLoadedArtifact(rightTopColorSensor);
    }

    public void checkIntakeArtifacts () {
        leftIntake = checkIntakeArtifact(leftLowerColorSensor);
        middleIntake = checkIntakeArtifact(middleLowerColorSensor);
        rightIntake = checkIntakeArtifact(rightLowerColorSensor);
        intakeNum = (leftIntake == ARTIFACT.NONE ? 0 : 1) + (middleIntake == ARTIFACT.NONE ? 0 : 1) + (rightIntake == ARTIFACT.NONE ? 0 : 1);
    }

    public void startDetector (boolean colors) {
        if (detecting.get()) {
            teamUtil.log ("WARNING----------startDetector called while already detecting. Ignored");
        } else {
            detecting.set(true);
            stopDetector.set(false);
            teamUtil.log("Launching Thread to startDetector.");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    detectIntakeArtifacts(colors);
                }
            });
            thread.start();
        }
    }

    public void stopDetector () {
        if (!detecting.get()) {
            teamUtil.log ("WARNING----------stopDetector called while not detecting. Ignored");
        } else {
            stopDetector.set(true);
            teamUtil.log("Stopping Detector Thread");
            teamUtil.robot.blinkin.setSignal(Blinkin.Signals.OFF);
        }
    }

    public void setBlinkinArtifact(ARTIFACT artifact){
        switch (artifact) {
            case NONE: teamUtil.robot.blinkin.setSignal(Blinkin.Signals.EMPTY); break;
            case PURPLE: teamUtil.robot.blinkin.setSignal(Blinkin.Signals.PURPLE); break;
            case GREEN: teamUtil.robot.blinkin.setSignal(Blinkin.Signals.GREEN); break;
        }
    }
    public static int FLASH_TIME = 100;
    public static int GAP_TIME = 100;
    public static int CYCLE_TIME = 500;
    public void detectIntakeArtifacts(boolean colors) {
        while (!stopDetector.get()) {
            //checkIntakeArtifacts();
            if (colors) {
                setBlinkinArtifact(leftIntake);
                teamUtil.pause(FLASH_TIME);
                teamUtil.robot.blinkin.setSignal(Blinkin.Signals.OFF);
                teamUtil.pause(GAP_TIME);
                setBlinkinArtifact(middleIntake);
                teamUtil.pause(FLASH_TIME);
                teamUtil.robot.blinkin.setSignal(Blinkin.Signals.OFF);
                teamUtil.pause(GAP_TIME);
                setBlinkinArtifact(rightIntake);
                teamUtil.pause(FLASH_TIME);
                teamUtil.robot.blinkin.setSignal(Blinkin.Signals.OFF);
                teamUtil.pause(CYCLE_TIME);
            } else {
                switch (intakeNum) {
                    case 0 : teamUtil.robot.blinkin.setSignal(Blinkin.Signals.ZERO);break;
                    case 1 : teamUtil.robot.blinkin.setSignal(Blinkin.Signals.ONE);break;
                    case 2 : teamUtil.robot.blinkin.setSignal(Blinkin.Signals.TWO);break;
                    case 3 : teamUtil.robot.blinkin.setSignal(Blinkin.Signals.THREE);break;
                }
            }
        }
        detecting.set(false);
        stopDetector.set(false);
    }

    */

}