package org.firstinspires.ftc.teamcode.assemblies;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.libs.OctoQuadFWv3;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

@Config // Makes Static data members available in Dashboard
public class BasicDrive{

////////////////////////////////////////////////////////////////////////////////////////////////
//
//    Drive class for mecanum wheels
//
    ////////////////////////////////////////////////////////////////////////////////////////////////

    // constants
    HardwareMap hardwareMap;
    Telemetry telemetry;

    // OctoQuad Setup and Calibration
    static public int DEADWHEEL_PORT_X = 0;
    static public int DEADWHEEL_PORT_Y = 1;
    static public OctoQuadFWv3.EncoderDirection DEADWHEEL_X_DIR = OctoQuadFWv3.EncoderDirection.FORWARD;
    static public OctoQuadFWv3.EncoderDirection DEADWHEEL_Y_DIR = OctoQuadFWv3.EncoderDirection.REVERSE;
    static public float X_TICKS_PER_MM = 19.9848f;
    static public float Y_TICKS_PER_MM = 20.0066f;
    static public float TCP_OFFSET_X_MM = 80f;
    static public float TCP_OFFSET_Y_MM = -155F;
    static public float IMU_SCALAR = 1.0354f;
    int oQbadPacketCount = 0;
    int oQgoodPacketCount = 0;

    // Localizer data read from the OctoQuad
    public OctoQuadFWv3.LocalizerDataBlock oQlocalizer = new OctoQuadFWv3.LocalizerDataBlock();
    public OctoQuadFWv3.EncoderDataBlock oQencoders = new OctoQuadFWv3.EncoderDataBlock();

    //public BNO055IMU imu; //This variable is the imu
    //public static double HEADING_OFFSET_IMU; // offset between IMU heading and field

    public static double HEADING_OFFSET_ODO; // offset between IMU heading and field
    public double lastVelocity;
    public boolean holdingHeading = false;
    public double heldHeading = 0;
    public AtomicBoolean movingAutonomously = new AtomicBoolean(false); // true under autonomous operation in teleop
    public AtomicBoolean manualInterrupt = new AtomicBoolean(true); // used to interrupt autonomous operations with manual driving control

    public DcMotorEx fl = null;
    public DcMotorEx fr = null;
    public DcMotorEx bl = null;
    public DcMotorEx br = null;

    static public double COUNTS_PER_CENTIMETER = 12.972; // 435s with 104mm wheels
    public double TILE_CENTER_TO_CENTER = 60.325; // tile width in Cms

    static public double MIN_START_VELOCITY = 300; //TODO (Current value works OK, but maybe could be more aggressive)
    static public double MIN_END_VELOCITY = 90; //TODO (Current value works OK, but maybe could be more aggressive)
    static public float MIN_END_POWER = .1f;
    static public double MAX_ACCELERATION = 50;
    static public double MAX_DECELERATION = 1.65;
    static public double POWER_BRAKING_STRAIGHT_FACTOR = .25f; // MMs per velocity unit
    static public double MAX_STRAIGHT_ACCELERATION = 20; //TODO
    static public double MAX_STRAIGHT_DECELERATION = 1.87; //TODO
    static public double MIN_STRAFE_START_VELOCITY = 500; //TODO
    static public double MIN_STRAFE_END_VELOCITY = 200; //TODO
    static public double MAX_STRAFE_DECELERATION = 2; //TODO
    static public double MAX_STRAFE_ACCELERATION = 20; // TODO

    static public double MAX_VELOCITY = 2200; // Calibrated 11/2/24
    static public double MAX_VELOCITY_STRAFE = 1750; // Calibrated 11/2/24
    static public double ROTATION_ADJUST_FACTOR = 40; //was .02
    static public double SIDE_VECTOR_COEFFICIENT = .92;
    static public double FORWARD_VECTOR_COEFFICIENT = 1.08;
    static public double SPIN_DECEL_THRESHOLD_DEGREES = 120; // Calibrated 11/2/24 (could be more aggresive?)
    static public double SPIN_DRIFT_DEGREES = 1; // Calibrated 11/2/24
    static public double SPIN_CRAWL_SPEED = 150; // Calibrated 11/2/24
    static public double SPIN_CRAWL_DEGREES = 10; // Calibrated 11/2/24 (could be more aggresive?)
    static public boolean details = false;
    static public float STRAIGHT_HEADING_DECLINATION = 1f; // convert strafe encoder error into heading declination
    static public float STRAIGHT_MAX_DECLINATION = 27f; // don't veer off of straight more than this number of degrees
    static public float STRAFE_HEADING_DECLINATION = .5f; // convert strafe encoder error into heading declination
    static public float STRAFE_MAX_DECLINATION = 27f; // don't veer off of straight more than this number of degrees

    static public double MOVE_TO_COEFFICIENT = 2;
    static public float MOVE_TO_POWER_COEFFICIENT = .001f;
    static public double MOVE_TO_THRESHOLD = 10;
    static public double MOVE_TO_DECCEL_COEFFICIENT = 0.5;

    public static float SPIN_DEADBAND = 0.3f;
    public static float DEADBAND = 0.1f;
    public static float SLOPE = 1.6f;
    public static float FASTSLOPE = 3.6f;
    public static float SLOWSPEED = .1f;
    public static float STRAFESLOWSPEED = 0.25f;
    public static float MAXROTATIONFACTOR = 0.8f;
    public static float ROTATION_ADJUST_HELD_HEADING = 0.025f;
    public static float SLOWSLOPE =0.22f;
    public static float SLOWSLOPESTRAFE =0.35f;

    public static double STOP_VEL_THRESHOLD = 1;
    public static double STOP_ANGLE_VEL_THRESHOLD = 0.5;

    public static double GOAL_X = 1575; // For Decode season, field coordinates are 0,0 at center of field
    public static double GOAL_Y = 1575;



    /************************************************************************************************************/
    // Initialization
    public BasicDrive() {
        teamUtil.log("Constructing BasicDrive");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public interface ActionCallback{
        public void action();
        //must return immediately or launch a thread
    }

    public void initialize() {
        teamUtil.log("Initializing BasicDrive");
        //Initialize the hardware variables.
        fl = hardwareMap.get(DcMotorEx.class, "flm");
        fr = hardwareMap.get(DcMotorEx.class, "frm");
        bl = hardwareMap.get(DcMotorEx.class, "blm");
        br = hardwareMap.get(DcMotorEx.class, "brm");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        teamUtil.robot.oq = hardwareMap.get(OctoQuadFWv3.class, "octo");
        teamUtil.robot.oq.setSingleEncoderDirection(DEADWHEEL_PORT_X, DEADWHEEL_X_DIR);
        teamUtil.robot.oq.setSingleEncoderDirection(DEADWHEEL_PORT_Y, DEADWHEEL_Y_DIR);
        teamUtil.robot.oq.setLocalizerPortX(DEADWHEEL_PORT_X);
        teamUtil.robot.oq.setLocalizerPortY(DEADWHEEL_PORT_Y);
        teamUtil.robot.oq.setLocalizerCountsPerMM_X(X_TICKS_PER_MM);
        teamUtil.robot.oq.setLocalizerCountsPerMM_Y(Y_TICKS_PER_MM);
        teamUtil.robot.oq.setLocalizerTcpOffsetMM_X(TCP_OFFSET_X_MM);
        teamUtil.robot.oq.setLocalizerTcpOffsetMM_Y(TCP_OFFSET_Y_MM);
        teamUtil.robot.oq.setLocalizerImuHeadingScalar(IMU_SCALAR);
        teamUtil.robot.oq.setLocalizerVelocityIntervalMS(25);
        teamUtil.robot.oq.setI2cRecoveryMode(OctoQuadFWv3.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);

        setMotorsBrake();
        teamUtil.log("Initializing Drive - FINISHED");
    }

    public void calibrate(){
        // Resetting the localizer will apply the parameters configured above.
        // This function will NOT block until calibration of the IMU is complete -
        // for that you need to look at the status returned by getLocalizerStatus()
        teamUtil.robot.oq.resetLocalizerAndCalibrateIMU();
    }

    public void loop() { // Call this frequently so that odometry data is up to date
        teamUtil.robot.oq.readLocalizerDataAndAllEncoderData(oQlocalizer, oQencoders);
        while (!oQlocalizer.crcOk && teamUtil.keepGoing(System.currentTimeMillis() + 10)) { // keep reading until we get good data
            teamUtil.log("ERROR!------------------------------------------  BAD CRC from OctoQuad!");
            teamUtil.robot.oq.readLocalizerDataAndAllEncoderData(oQlocalizer, oQencoders);
        }
    }

    /************************************************************************************************************/
    // Telemetry

    public void driveMotorTelemetry() {
        telemetry.addData("Encoders ", "FL:%d FR:%d BL:%d BR:%d X:%d Y%d",
                fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition(), oQencoders.positions[DEADWHEEL_PORT_X], oQencoders.positions[DEADWHEEL_PORT_Y]);
        String data = String.format(Locale.US, "Localizer X: %.0f, Y: %.0f, H: %.1f", (float) oQlocalizer.posX_mm, (float) oQlocalizer.posY_mm, Math.toDegrees(oQlocalizer.heading_rad));
        telemetry.addData("ODO Position ", data);
        String velocity = String.format(Locale.US, "Velocity X: %.0f, Y: %.0f, H: %.1f", (float) oQlocalizer.velX_mmS, (float) oQlocalizer.velY_mmS, Math.toDegrees(oQlocalizer.velHeading_radS));
        telemetry.addData("ODO Velocity ", velocity);
        telemetry.addData("Headings: ", "ODO: %.1f  To Goal: %.1f  Held: %.1f", getHeadingODO(), getGoalHeading(), heldHeading);
        String currents = String.format(Locale.US, "FR: %.0f, FL: %.0f, BR: %.0f, BL: %.0f" , fr.getCurrent(CurrentUnit.AMPS), fl.getCurrent(CurrentUnit.AMPS),br.getCurrent(CurrentUnit.AMPS), bl.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("CurrentDraw: ", currents);

        //teamUtil.log("FR: " + fr.getCurrent(CurrentUnit.AMPS) +" FL: " + fl.getCurrent(CurrentUnit.AMPS) + " BR: " + br.getCurrent(CurrentUnit.AMPS) + "BL" + bl.getCurrent(CurrentUnit.AMPS));
    }

    public void logMotorPositions() {
        teamUtil.log("fr: " + fr.getCurrentPosition());
        teamUtil.log("fl: " + fl.getCurrentPosition());
        teamUtil.log("br: " + br.getCurrentPosition());
        teamUtil.log("bl: " + bl.getCurrentPosition());
    }

    /************************************************************************************************************/
    //   Basic Motor Operations

    public void setBulkReadOff() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }
    }

    public void setBulkReadAuto() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void runMotors(double velocity) {
        lastVelocity = velocity;
        fl.setVelocity(velocity);
        fr.setVelocity(velocity);
        bl.setVelocity(velocity);
        br.setVelocity(velocity);
    }

    public void setMotorsBrake() {
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorsFloat() {
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setMotorPower(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    public void setMotorsActiveBrake() {
        // hold a position using setPosition
        int flPosition = fl.getCurrentPosition();
        int frPosition = fr.getCurrentPosition();
        int blPosition = bl.getCurrentPosition();
        int brPosition = br.getCurrentPosition();
        fl.setTargetPosition(flPosition);
        fr.setTargetPosition(frPosition);
        bl.setTargetPosition(blPosition);
        br.setTargetPosition(brPosition);

        setMotorsRunToPosition();
        setMotorPower(0.5);
    }

    public void stopMotors() {
        if (details) teamUtil.log("Stopping Motors");
        lastVelocity = 0;
        fl.setVelocity(0);
        fr.setVelocity(0);
        bl.setVelocity(0);
        br.setVelocity(0);
    }

    public void setMotorVelocities(double flV, double frV, double blV, double brV) {
        fl.setVelocity(flV);
        fr.setVelocity(frV);
        bl.setVelocity(blV);
        br.setVelocity(brV);
    }

    public void setMotorPowers(float flP, float frP, float blP, float brP) {
        fl.setPower(flP);
        fr.setPower(frP);
        bl.setPower(blP);
        br.setPower(brP);
    }

    public void setMotorsRunToPosition() {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void setMotorsRunWithoutEncoder() {
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void resetAllDriveEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setMotorsWithEncoder() {
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Uses localizer to wait until robot comes to a rest (assumes drifting from previous movement)
    public void waitForRobotToStop(long timeout){
        teamUtil.log("waitForRobotToStop");
        loop();
        long timeOutTime = System.currentTimeMillis()+timeout;

        //Get Velocities in mm/sec
        double xVelo = oQlocalizer.velX_mmS;
        double yVelo = oQlocalizer.velY_mmS;
        double hVelo = oQlocalizer.velHeading_radS;

        while((Math.abs(xVelo)> STOP_VEL_THRESHOLD || Math.abs(yVelo)> STOP_VEL_THRESHOLD/*||hVelo>ANGLE_VELO_THRESHOLD*/) && teamUtil.keepGoing(timeOutTime)){
            teamUtil.pause(10);
            if(details) teamUtil.log(String.format("xPos: %.0f xVel: %.2f yPos: %.0f yVel: %.2f hVel: %.2f", oQlocalizer.posX_mm, xVelo, oQlocalizer.posY_mm, yVelo , hVelo));
            loop();
            xVelo = oQlocalizer.velX_mmS;
            yVelo = oQlocalizer.velY_mmS;
            hVelo = oQlocalizer.velHeading_radS;
        }

        if(System.currentTimeMillis()>timeOutTime){
            teamUtil.log("waitForRobotToStop has TIMED OUT");
            return;
        }
        teamUtil.log("waitForRobotToStop has Finished");
    }

    /************************************************************************************************************/
    //   Heading Operations

    public double getHeadingError(double targetAngle) {
        // distance from target
        double robotError;

        // calculate heading error in -179 to +180 range  (
        robotError = targetAngle - getHeadingODO();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getRawHeadingODO() {
        return Math.toDegrees(oQlocalizer.heading_rad);
    }

    public double getHeadingODO() {
        // stows an offset to change the range and set the heading
        return adjustAngle(getRawHeadingODO() - HEADING_OFFSET_ODO);
    }

    //Make the current heading to specified number
    public void setHeading(int heading) {
        HEADING_OFFSET_ODO = getRawHeadingODO() - heading;
        heldHeading = getHeadingODO(); // Use pinpoint IMU
    }

    public double adjustAngle(double angle) {
        //assuming imu runs from [0, 360] and angle is added/substracted, adjust it to expected reading
        while (angle >= 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    // Uses the robots current position (localizer) to determine field relative heading to center of goal
    public double getGoalHeading(){
        double local_goal_y = GOAL_Y;
        if (teamUtil.alliance == teamUtil.Alliance.RED){
            local_goal_y = -GOAL_Y;
        }
        double offset_x = GOAL_X - oQlocalizer.posX_mm;
        double offset_y = local_goal_y - oQlocalizer.posY_mm;
        if(offset_x == 0){
            if(offset_y > 0){
                return 90;
            }else{
                return 270;
            }
        }
        double raw_Angle = Math.toDegrees(Math.atan(offset_y / offset_x));
        if(offset_x < 0){
            raw_Angle += 180;
        }
        raw_Angle = adjustAngle(raw_Angle);
        return raw_Angle;
    }


    /************************************************************************************************************/
    //   Holonomic Motor Operations

    // Set the robot position x,y and heading in Field Relative coordinates
    // X and Y are in mms, Heading is in degrees
    public void setRobotPosition(int x, int y, double heading) {
        teamUtil.log("setRobotPosition: x: "+ x + " y: " + y + " heading: " + heading);
        teamUtil.robot.oq.setLocalizerPose(x,y,(float)Math.toRadians(heading));
        loop();
        teamUtil.log("odoHeading after reset: " + getRawHeadingODO());
        setHeading((int) heading);
    }

    // drive the motors at the specified (robot relative) at the specified velocity while holding the (robot relative) robot heading
    // power is 0-1
    public static float ROTATION_ADJUST_FACTOR_POWER = 0.05f;
    public static boolean newDrivePowerAlgo = false;
    public void driveMotorsHeadingsPower(double driveHeading, double robotHeading, double power) {
        // move robot based on a heading to face and a heading to drive to
        float flV, frV, blV, brV;
        double x, y, scale;

        // Determine how much adjustment for rotational drift
        double headingError = getHeadingError(robotHeading); // Difference between desired and actual robot heading
        //double headingError = Math.max(-45.0, Math.min(getHeadingError(robotHeading), 45.0)); // clip this to 45 degrees in either direction to control rate of spin
        float rotationAdjust = (float)(ROTATION_ADJUST_FACTOR_POWER *  headingError * power); // scale based on amount of rotational error and power
        if(details) teamUtil.log("Params: DriveHeading: " +driveHeading + " RobotHeading: " + robotHeading + " Power: " + power + " RobotHeadingError: " + headingError + " ODOHeading: " + getHeadingODO()+ " RotAdjust: " + rotationAdjust);


        if (newDrivePowerAlgo) {
            // The new stuff
            x = Math.cos(Math.toRadians(driveHeading + 90)); // + 90 cause forward is 0...
            y = Math.sin(Math.toRadians(driveHeading + 90));
            // adjust so x+y and x-y are between -1 and 1 but retain their ratio
            scale = 1 / Math.max(Math.abs(x + y), Math.abs(y - x));
            flV = (float) ((x + y) * scale * power);
            brV = flV;
            frV = (float) ((y - x) * scale * power);
            blV = frV;
        } else {
            // The old stuff
            // Covert heading to cartesian on the unit circle and scale so largest value is 1
            // This is essentially creating joystick values from the heading
            // driveHeading is relative to robot at this point since the wheels are relative to robot!
            x = Math.cos(Math.toRadians(driveHeading + 90)); // + 90 cause forward is 0...
            y = Math.sin(Math.toRadians(driveHeading + 90));
            scale = 1 / Math.max(Math.abs(x), Math.abs(y));
            x = x * scale * power; // Then set proportional to commanded power
            y = y * scale * power;
            // Clip to motor power range
            flV = (float) Math.max(-1.0, Math.min(x + y, 1.0)) ;
            brV = flV;
            frV = (float) Math.max(-1.0, Math.min(y - x, 1.0)) ;
            blV = frV;
        }
        if(details) teamUtil.log("Powers before rot adjust: FLV/BRV: " + flV + " FRV/BLV: " + frV);

        // Adjust for rotational drift
        flV = flV - rotationAdjust;
        brV = brV + rotationAdjust;
        frV = frV + rotationAdjust;
        blV = blV - rotationAdjust;
        if(details) teamUtil.log("Powers AFTER rot adjust: FLV: " + flV + " FRV: " + frV + " BLV: " + blV + " BRV: " + brV);

        // Update the motors
        setMotorPowers(flV, frV, blV, brV);
    }

    // drive the motors at the specified heading (robot relative) at the specified velocity while holding the (robot relative) robot heading
    public void driveMotorsHeadings(double driveHeading, double robotHeading, double velocity) {
        // move robot based on a heading to face and a heading to drive to
        double flV, frV, blV, brV;
        double x, y, scale;

        // Determine how much adjustment for rotational drift
        double headingError = getHeadingError(robotHeading); // Difference between desired and actual robot heading
        //double headingError = Math.max(-45.0, Math.min(getHeadingError(robotHeading), 45.0)); // clip this to 45 degrees in either direction to control rate of spin
        double rotationAdjust = ROTATION_ADJUST_FACTOR * headingError; // scale based on amount of rotational error.....Took out velocity
        if(details) teamUtil.log("Params: DriveHeading: " +driveHeading + " RobotHeading: " + robotHeading + " Velocity: " + velocity+ " RobotHeadingError: " + headingError +  " ODOHeading: " + getHeadingODO()+ " RotAdjust: " + rotationAdjust);

        // Covert heading to cartesian on the unit circle and scale so largest value is 1
        // This is essentially creating joystick values from the heading
        // driveHeading is relative to robot at this point since the wheels are relative to robot!
        x = Math.cos(Math.toRadians(driveHeading + 90)); // + 90 cause forward is 0...
        y = Math.sin(Math.toRadians(driveHeading + 90));
        scale = 1 / Math.max(Math.abs(x), Math.abs(y));
        x = x * scale;
        y = y * scale;

        // Clip to motor power range
        flV = Math.max(-1.0, Math.min(x + y, 1.0)) * velocity;
        brV = flV;
        frV = Math.max(-1.0, Math.min(y - x, 1.0)) * velocity;
        blV = frV;
        if(details) teamUtil.log("Velocities before rot adjust: FLV/BRV: " + flV + " FRV/BLV: " + frV);

        // Adjust for rotational drift
        flV = flV - rotationAdjust;
        brV = brV + rotationAdjust;
        frV = frV + rotationAdjust;
        blV = blV - rotationAdjust;
        if(details) teamUtil.log("Velocities AFTER rot adjust: FLV: " + flV + " FRV: " + frV + " BLV: " + blV + " BRV: " + brV);

        // Update the motors
        setMotorVelocities(flV, frV, blV, brV);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set the velocity of all 4 motors based on a driveHeading RELATIVE TO FIELD and provided velocity
    // Will rotate robot as needed to achieve and hold robotHeading RELATIVE TO FIELD by moving to a set target
    public void driveMotorsHeadingsFR(double driveHeading, double robotHeading, double velocity) {
        double RRDriveHeading = getHeadingError(driveHeading);
        if(details)teamUtil.log("RRDriveHeading: " + RRDriveHeading + " RobotHeading: " + robotHeading + " DriveHeading: " + driveHeading);
        driveMotorsHeadings(RRDriveHeading, robotHeading, velocity);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set the velocity of all 4 motors based on a driveHeading RELATIVE TO FIELD and provided velocity
    // Will rotate robot as needed to achieve and hold robotHeading RELATIVE TO FIELD by moving to a set target
    public void driveMotorsHeadingsFRPower(double driveHeading, double robotHeading, float power) {
        double RRDriveHeading = getHeadingError(driveHeading);
        if(details)teamUtil.log("RRDriveHeading: " + RRDriveHeading + " RobotHeading: " + robotHeading + " DriveHeading: " + driveHeading);
        driveMotorsHeadingsPower(RRDriveHeading, robotHeading, power);
    }


    class MotorData { // a helper class to allow for faster access to hub data
        int eFL, eFR, eBL, eBR; // encoder values of each motor
    }

    public void getDriveMotorData(MotorData data) {
        // update current motor positions
        data.eFL = fl.getCurrentPosition();
        data.eFR = fr.getCurrentPosition();
        data.eBL = bl.getCurrentPosition();
        data.eBR = br.getCurrentPosition();
    }

    /************************************************************************************************************/
    // Methods to drive based on motor encoders

    public int getEncoderDistance(MotorData initialPositions) {
        // use trig to find the hypotenuse of the side and front distances
        MotorData currentPositions = new MotorData();
        getDriveMotorData(currentPositions);

        // Calculate the vector along the forward/backward axis
        int ForwardVector = (currentPositions.eFL - initialPositions.eFL)
                + (currentPositions.eFR - initialPositions.eFR)
                + (currentPositions.eBL - initialPositions.eBL)
                + (currentPositions.eBR - initialPositions.eBR);
        // Calculate the vector along the left/right axis
        int SideVector = (currentPositions.eFL - initialPositions.eFL)
                + (currentPositions.eBR - initialPositions.eBR)
                - (currentPositions.eFR - initialPositions.eFR)
                - (currentPositions.eBL - initialPositions.eBL);

        // Return the hypotenuse of the two vectors
        // divide by 4 to account for the math that adds all 4 motor encoders
        return (int) (Math.sqrt(Math.pow((ForwardVector * FORWARD_VECTOR_COEFFICIENT), 2) + (Math.pow((SideVector * SIDE_VECTOR_COEFFICIENT), 2))) / 4);

    }

    public void moveCm(double centimeters, double driveHeading) {
        // simplified parameters of moveCm
        moveCm(MAX_VELOCITY, centimeters, driveHeading, getHeadingODO(), MIN_END_VELOCITY);
    }

    public void moveCm(double centimeters, double driveHeading, double endVelocity) {
        // simplified parameters of moveCm
        moveCm(MAX_VELOCITY, centimeters, driveHeading, getHeadingODO(), endVelocity);
    }

    public void moveCm(double maxVelocity, double centimeters, double driveHeading, double endVelocity) {
        // simplified parameters of moveCm
        moveCm(maxVelocity, centimeters, driveHeading, getHeadingODO(), endVelocity);
    }

    public void moveCm(double maxVelocity, double centimeters, double driveHeading, double robotHeading, double endVelocity) {
        // move based on a cruise, end, and max velocity, distance, and headings
        teamUtil.log("MoveCM cms:" + centimeters + " driveH:" + driveHeading + " robotH:" + robotHeading + " MaxV:" + maxVelocity + " EndV:" + endVelocity);


        MotorData data = new MotorData();
        getDriveMotorData(data);

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        if (endVelocity < MIN_END_VELOCITY) {
            endVelocity = MIN_END_VELOCITY; // simplify by setting min end to 0
        }
        // tics^2/s
        if (lastVelocity == 0) { // at a stop
            velocityChangeNeededAccel = maxVelocity - MIN_START_VELOCITY;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        } else { // already moving
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }
        setMotorsWithEncoder();
        // all are measured in tics
        double totalTics = centimeters * COUNTS_PER_CENTIMETER;
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_DECELERATION);
        double postCruiseTargetDistance = totalTics - decelerationDistance;
        if (postCruiseTargetDistance < 0) { // need to cut off the curve
            double percentageToRemoveAccel = accelerationDistance / (accelerationDistance + decelerationDistance);
            accelerationDistance += postCruiseTargetDistance * percentageToRemoveAccel;
            decelerationDistance += postCruiseTargetDistance * percentageToRemoveAccel;
            postCruiseTargetDistance = 0;
        }
        double distance = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeadingODO());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
            teamUtil.log("Post Cruise distance: " + postCruiseTargetDistance);
        }
//acceleration
        while (distance < accelerationDistance) {
            loop();
            distance = getEncoderDistance(data);
            if (lastVelocity == 0) {
                driveMotorsHeadingsFR(driveHeading, robotHeading, MAX_ACCELERATION * distance + MIN_START_VELOCITY); // velocity moves by distance
            } else {
                driveMotorsHeadingsFR(driveHeading, robotHeading, MAX_ACCELERATION * distance + lastVelocity);
            }
        }
        if (details) {
            teamUtil.log("Heading:" + getHeadingODO());
            teamUtil.log("distance after acceleration: " + distance);
        }
//cruise
        while (distance < postCruiseTargetDistance) {
            loop();

            distance = getEncoderDistance(data);
            driveMotorsHeadingsFR(driveHeading, robotHeading, maxVelocity); // constant
        }
        if (details) {
            teamUtil.log("Heading:" + getHeadingODO());
            teamUtil.log("distance after cruise: " + distance);
        }


//deceleration
        double startDecelerationDistance = distance;
        double ticsUntilEnd = totalTics - distance;
        while (distance < totalTics) {
            loop();

            distance = getEncoderDistance(data);
            ticsUntilEnd = totalTics - distance;
            driveMotorsHeadingsFR(driveHeading, robotHeading, MAX_DECELERATION * ticsUntilEnd + endVelocity); // lowers through tics to end

        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distance);
        }
        if (endVelocity <= MIN_END_VELOCITY) {
            stopMotors();
            if (details) {
                teamUtil.log("Went below or was min end velocity");
            }
        }
        lastVelocity = Math.max(endVelocity,MIN_START_VELOCITY);
        teamUtil.log("MoveCM--Finished");

    }


    /************************************************************************************************************/
    // OLD Methods to drive based on odometry pods

/*
    public boolean strafeToEncoderWithDecel(double driveHeading, double robotHeading, double velocity, double targetEncoderValue, double endVelocity, double decelK, long timeout) {
        long timeOutTime = System.currentTimeMillis() + timeout;
        teamUtil.log("strafeToEncoder: Current: " + strafeEncoder.getCurrentPosition() + " Target: " + targetEncoderValue);
        float driftCms = 1;
        double realTarget = targetEncoderValue + (driveHeading < 180? -1:1)*driftCms*TICS_PER_CM_STRAFE_ENCODER;
        double strafeCmsToGo;
        double liveVelocity;
        if(endVelocity<MIN_END_VELOCITY){
            endVelocity = MIN_END_VELOCITY;
        }

        if (driveHeading<180) {
            while (strafeEncoder.getCurrentPosition() < realTarget && teamUtil.keepGoing(timeOutTime)) {
                strafeCmsToGo = Math.abs(targetEncoderValue-strafeEncoder.getCurrentPosition())/TICS_PER_CM_STRAFE_ENCODER;
                liveVelocity = Math.min(velocity, endVelocity+decelK* COUNTS_PER_CENTIMETER * strafeCmsToGo);
                driveMotorsHeadingsFR(driveHeading, robotHeading, liveVelocity);
            }
        }
        else{
            while (strafeEncoder.getCurrentPosition() > realTarget && teamUtil.keepGoing(timeOutTime)) {
                strafeCmsToGo = Math.abs(targetEncoderValue-strafeEncoder.getCurrentPosition())/TICS_PER_CM_STRAFE_ENCODER;
                liveVelocity = Math.min(velocity, endVelocity+decelK* COUNTS_PER_CENTIMETER * strafeCmsToGo);
                driveMotorsHeadingsFR(driveHeading, robotHeading, liveVelocity);
            }
        }
        lastVelocity=velocity;
        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("strafeToEncoder - TIMED OUT!");
            return false;
        } else {
            teamUtil.log("strafeToEncoder - FINISHED : Current: " + strafeEncoder.getCurrentPosition());
            return true;
        }
    }

    public void straightToTarget(double maxVelocity, double forwardTarget, double driveHeading, double robotHeading, double endVelocity, long timeout) {
        teamUtil.log("ERROR: straightToTarget not updated for PinPoint yet"); // TODO
        if (true) return;
        // same as movecm but reads distance from dead wheels
        teamUtil.log("driveToTarget target: " + forwardTarget + " driveH: " + driveHeading + " robotH: " + robotHeading + " MaxV: " + maxVelocity + " EndV: " + endVelocity);
        details = false;
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;

        if (details) teamUtil.log("Starting Forward Encoder: "+ forwardEncoder.getCurrentPosition());

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        if (endVelocity < MIN_END_VELOCITY) {
            endVelocity = MIN_END_VELOCITY;
        }
        // tics^2/s
        if (lastVelocity == 0) {
            velocityChangeNeededAccel = maxVelocity - MIN_START_VELOCITY;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        } else {
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }
        // all are measured in tics
        double startEncoder = forwardEncoder.getCurrentPosition();
        if (((driveHeading< 90 || driveHeading>270)&&forwardTarget-startEncoder >=0) || ((driveHeading> 90 && driveHeading<270) && startEncoder-forwardTarget >=0)){

            teamUtil.log("ALREADY PAST TARGET--Not Moving");
            stopMotors();
            return;
        }
        double totalTics = Math.abs(startEncoder-forwardTarget);
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_STRAIGHT_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_STRAIGHT_DECELERATION);
        if (accelerationDistance+decelerationDistance >= totalTics ) { // No room for cruise phase
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");
            double accelPercentage = accelerationDistance / (accelerationDistance + decelerationDistance);
            double decelPercentage = 1-accelPercentage;
            accelerationDistance = totalTics * accelPercentage;
            decelerationDistance = totalTics * decelPercentage;
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");

        }
        double distanceRemaining = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeadingODO());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
        }
        distanceRemaining = (driveHeading< 90 || driveHeading>270) ? forwardEncoder.getCurrentPosition()-forwardTarget : forwardTarget - forwardEncoder.getCurrentPosition();

        setBulkReadAuto();

//acceleration
        double currentVelocity = 0;
        while ((distanceRemaining > (totalTics-accelerationDistance))&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? forwardEncoder.getCurrentPosition()-forwardTarget : forwardTarget - forwardEncoder.getCurrentPosition();
            if (lastVelocity == 0) {
                currentVelocity = MAX_STRAIGHT_ACCELERATION * (totalTics-distanceRemaining) + MIN_START_VELOCITY;
            } else {
                currentVelocity = MAX_STRAIGHT_ACCELERATION * (totalTics-distanceRemaining) + lastVelocity;
            }
            if (details) teamUtil.log("Accelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, currentVelocity);

        }
        if (details) {
            teamUtil.log("Heading:" + getHeadingODO());
            teamUtil.log("distance after acceleration: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Acceleration Phase");
            stopMotors();
            return;


        }
//cruise
        while ((distanceRemaining > decelerationDistance)&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? forwardEncoder.getCurrentPosition()-forwardTarget : forwardTarget - forwardEncoder.getCurrentPosition();
            if (details) teamUtil.log("Cruising at Velocity: "+ maxVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, maxVelocity);
        }
        if (details) {
            teamUtil.log("Heading:" + getHeadingODO());
            teamUtil.log("distance after cruise: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Cruise Phase");
            stopMotors();
            return;


        }

//deceleration
        while ((distanceRemaining > 0)&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? forwardEncoder.getCurrentPosition()-forwardTarget : forwardTarget - forwardEncoder.getCurrentPosition();
            currentVelocity = MAX_STRAIGHT_DECELERATION * distanceRemaining + endVelocity;
            if (details) teamUtil.log("Decelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, currentVelocity);
        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distanceRemaining);
        }
        if (endVelocity <= MIN_END_VELOCITY) {
            stopMotors();
            if (details) {
                teamUtil.log("Went below or was min end velocity");
            }
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return;


        }
        setBulkReadOff();
        lastVelocity = endVelocity;
        teamUtil.log("driveToTarget--Finished.  Current Forward Encoder:" + forwardEncoder.getCurrentPosition());

    }

    public void strafeToTarget(double maxVelocity, double strafeTarget, double driveHeading, double robotHeading, double endVelocity, long timeout) {
        teamUtil.log("ERROR: strafeToTarget not updated for PinPoint yet"); // TODO
        if (true) return;
        teamUtil.log("strafeToTarget target: " + strafeTarget + " driveH: " + driveHeading + " robotH: " + robotHeading + " MaxV: " + maxVelocity + " EndV: " + endVelocity);
        details = false;
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;

        if (details) teamUtil.log("Starting Strafe Encoder: "+ strafeEncoder.getCurrentPosition());

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        if (endVelocity < MIN_STRAFE_END_VELOCITY) {
            endVelocity = MIN_STRAFE_END_VELOCITY;
        }
        // tics^2/s
        if (lastVelocity == 0) { // at stop
            velocityChangeNeededAccel = maxVelocity - MIN_STRAFE_START_VELOCITY;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        } else { // moving
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }
        // all are measured in tics
        double startEncoder = strafeEncoder.getCurrentPosition();
        if ((driveHeading< 180 && strafeTarget-startEncoder <=0) || (driveHeading > 180 && startEncoder-strafeTarget <=0))
        {
            teamUtil.log("ALREADY PAST TARGET--Not Strafing");
            stopMotors();
            return;
        }
        double totalTics = Math.abs(startEncoder-strafeTarget);
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_STRAFE_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_STRAFE_DECELERATION);
        if (accelerationDistance+decelerationDistance >= totalTics ) { // No room for cruise phase
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");
            double accelPercentage = accelerationDistance / (accelerationDistance + decelerationDistance);
            double decelPercentage = 1-accelPercentage;
            accelerationDistance = totalTics * accelPercentage;
            decelerationDistance = totalTics * decelPercentage;
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");

        }
        double distanceRemaining = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeadingODO());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
        }
        distanceRemaining = driveHeading<180 ? strafeTarget - strafeEncoder.getCurrentPosition() : strafeEncoder.getCurrentPosition()-strafeTarget;

        setBulkReadAuto();

//acceleration
        double currentVelocity = 0;
        while ((distanceRemaining > (totalTics-accelerationDistance))&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = driveHeading<180 ? strafeTarget - strafeEncoder.getCurrentPosition() : strafeEncoder.getCurrentPosition()-strafeTarget;
            if (lastVelocity == 0) {
                currentVelocity = MAX_STRAFE_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + MIN_STRAFE_START_VELOCITY; // increases velocity
            } else {
                currentVelocity = MAX_STRAFE_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + lastVelocity;
            }
            if (details) teamUtil.log("Accelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, currentVelocity);

        }
        if (details) {
            teamUtil.log("Heading:" + getHeadingODO());
            teamUtil.log("distance after acceleration: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Acceleration Phase");
            stopMotors();
            return;


        }
//cruise
        while ((distanceRemaining > decelerationDistance)&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = driveHeading<180 ? strafeTarget - strafeEncoder.getCurrentPosition() : strafeEncoder.getCurrentPosition()-strafeTarget;
            if (details) teamUtil.log("Cruising at Velocity: "+ maxVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, maxVelocity); // constant
        }
        if (details) {
            teamUtil.log("Heading:" + getHeadingODO());
            teamUtil.log("distance after cruise: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Cruise Phase");
            stopMotors();
            return;


        }

//deceleration
        while ((distanceRemaining > 0)&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = driveHeading<180 ? strafeTarget - strafeEncoder.getCurrentPosition() : strafeEncoder.getCurrentPosition()-strafeTarget;
            currentVelocity = MAX_STRAFE_DECELERATION * distanceRemaining + endVelocity;
            if (details) teamUtil.log("Decelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, currentVelocity);// decreases
        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distanceRemaining);
        }
        if (endVelocity <= MIN_STRAFE_END_VELOCITY) {
            stopMotors();
            if (details) {
                teamUtil.log("Went below or was min end velocity");
            }
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return;


        }
        setBulkReadOff();
        lastVelocity = endVelocity;
        teamUtil.log("strafeToTarget--Finished.  Current Strafe Encoder:" + strafeEncoder.getCurrentPosition());

    }
     */

    /************************************************************************************************************/
    /************************************************************************************************************/
    // New Methods to drive based on Odometry Computer

    public boolean strafeToTarget(double driveHeading, double robotHeading, double velocity, double strafeTarget, long timeout) {
        // strafe to a strafe encoder value
        long timeOutTime = System.currentTimeMillis() + timeout;
        loop();
        teamUtil.log("strafeToTarget: Current: " + oQlocalizer.posY_mm + " Target: " + strafeTarget);
        if (driveHeading<180) {
            while (oQlocalizer.posY_mm < strafeTarget && teamUtil.keepGoing(timeOutTime)) {
                loop();
                driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            }
        }
        else{
            while (oQlocalizer.posY_mm > strafeTarget && teamUtil.keepGoing(timeOutTime)) {
                loop();
                driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            }
        }
        lastVelocity=velocity;
        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("strafeToTarget - TIMED OUT!");
            return false;
        } else {
            teamUtil.log("strafeToTarget - FINISHED : Current: " + oQlocalizer.posY_mm);
            return true;
        }
    }

    // Drive straight forward or backward while attempting to hold the strafe encoder at a specific value
    // Robot heading should be 0,90,180, or 270.  Drive Heading will be determined by the target
    public boolean straightHoldingStrafeEncoder(double maxVelocity, double straightTarget, double strafeTarget, int robotHeading, double endVelocity, boolean powerBraking, ActionCallback action, double actionTarget, long timeout) {
        if(robotHeading != 90 && robotHeading != 270 && robotHeading != 0 && robotHeading != 180){
            teamUtil.log("straightHoldingStrafeEncoder - ERROR INCOMPATIBLE ROBOT HEADING");
            stopMotors();
            return false;
        }

        teamUtil.log("straightHoldingStrafeEncoder target: " + straightTarget +  " Strafe target: " + strafeTarget + " robotH: " + robotHeading + " MaxV: " + maxVelocity + " EndV: " + endVelocity);
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;

        loop();
        if (details) teamUtil.log("Starting Forward Pos: "+ oQlocalizer.posX_mm);
        if (details) teamUtil.log("Starting Strafe Pos: "+ oQlocalizer.posY_mm);
        // if (details) teamUtil.log("Starting Forward Encoder: "+ forwardEncoder.getCurrentPosition());
        //if (details) teamUtil.log("Starting Strafe Encoder: "+ strafeEncoder.getCurrentPosition());

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        double driveHeading;
        double startEncoder = oQlocalizer.posX_mm;
        //double startEncoder = forwardEncoder.getCurrentPosition();
        boolean goingUp;
        if(straightTarget-startEncoder >=0){
            driveHeading = robotHeading;
            goingUp = true;
        }else{
            driveHeading = adjustAngle(robotHeading+180);
            goingUp=false;
        }

        float headingFactor = goingUp? -1 : 1; // reverse correction for going backwards

        double requestedEndVelocity = endVelocity;
        if (endVelocity < MIN_END_VELOCITY) {
            endVelocity = MIN_END_VELOCITY;
        }

        if (lastVelocity == 0) {
            velocityChangeNeededAccel = maxVelocity - MIN_START_VELOCITY;
        } else {
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
        }
        velocityChangeNeededDecel = maxVelocity - endVelocity;
        // all are measured in tics

        double totalTics = Math.abs(startEncoder-straightTarget);
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_STRAIGHT_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_STRAIGHT_DECELERATION);
        if (accelerationDistance+decelerationDistance >= totalTics ) { // No room for cruise phase
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");
            double accelPercentage = accelerationDistance / (accelerationDistance + decelerationDistance);
            double decelPercentage = 1-accelPercentage;
            accelerationDistance = totalTics * accelPercentage;
            decelerationDistance = totalTics * decelPercentage;
        }
        double distanceRemaining = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeadingODO());
            teamUtil.log("Total MMs: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
        }
        distanceRemaining = Math.abs(straightTarget - oQlocalizer.posX_mm);
        //distanceRemaining = Math.abs(straightTarget - forwardEncoder.getCurrentPosition());

        setBulkReadAuto();
        boolean actionDone = false;
        double currentPos;

        //-------Acceleration Phase
        double currentVelocity;
        double adjustedDriveHeading;
        while ((distanceRemaining > (totalTics-accelerationDistance))&&teamUtil.keepGoing(timeoutTime)) {
            loop();
            currentPos = oQlocalizer.posX_mm;
            distanceRemaining = (!goingUp) ? currentPos-straightTarget : straightTarget - currentPos;
            if (lastVelocity == 0) {
                currentVelocity = MAX_STRAIGHT_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + MIN_START_VELOCITY;
            } else {
                currentVelocity = MAX_STRAIGHT_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + lastVelocity;
            }
            adjustedDriveHeading = driveHeading + MathUtils.clamp((oQlocalizer.posY_mm - strafeTarget)* STRAIGHT_HEADING_DECLINATION, -STRAIGHT_MAX_DECLINATION, STRAIGHT_MAX_DECLINATION) * headingFactor;
            //adjustedDriveHeading = driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)* STRAIGHT_HEADING_DECLINATION, -STRAIGHT_MAX_DECLINATION, STRAIGHT_MAX_DECLINATION) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }

            if (details) teamUtil.log("Accelerating at Velocity: "+ currentVelocity + " MMs Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, currentVelocity);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }
        }
        if (details) {
            teamUtil.log("Heading:" + getHeadingODO());
            teamUtil.log("distance after acceleration: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered in Acceleration Phase");
            stopMotors();
            return false;
        }

        //-------Cruise Phase
        while ((distanceRemaining > decelerationDistance)&&teamUtil.keepGoing(timeoutTime)) {
            loop();
            currentPos = oQlocalizer.posX_mm;
            distanceRemaining = (!goingUp) ? currentPos-straightTarget : straightTarget - currentPos;
            adjustedDriveHeading = driveHeading + MathUtils.clamp((oQlocalizer.posY_mm - strafeTarget)* STRAIGHT_HEADING_DECLINATION, -STRAIGHT_MAX_DECLINATION, STRAIGHT_MAX_DECLINATION) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }
            if (details) teamUtil.log("Cruising at Velocity: "+ maxVelocity + " MMs Remaining: " + distanceRemaining);

            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, maxVelocity);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }
        }
        if (details) {
            teamUtil.log("Heading:" + getHeadingODO());
            teamUtil.log("distance after cruise: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered in Cruise Phase");
            stopMotors();
            return false;
        }

        if (!powerBraking) {  //-------Normal Deceleration Phase
            //-------Normal Deceleration Phase
            while ((distanceRemaining > 0)&&teamUtil.keepGoing(timeoutTime)) {
                loop();
                currentPos = oQlocalizer.posX_mm;
                distanceRemaining = (!goingUp) ? currentPos-straightTarget : straightTarget - currentPos;
                currentVelocity = MAX_STRAIGHT_DECELERATION * distanceRemaining + endVelocity;
                adjustedDriveHeading = driveHeading + MathUtils.clamp((oQlocalizer.posY_mm - strafeTarget)* STRAIGHT_HEADING_DECLINATION, -STRAIGHT_MAX_DECLINATION, STRAIGHT_MAX_DECLINATION) * headingFactor;
                if (details) {
                    teamUtil.log("dh: " + adjustedDriveHeading);
                }

                if (details) teamUtil.log("Decelerating at Velocity: "+ currentVelocity + " MMs Remaining: " + distanceRemaining);
                driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, currentVelocity);
                if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                    action.action();
                    actionDone=true;
                }
            }
            if (details) {
                teamUtil.log("distance after deceleration: " + distanceRemaining);
            }
        } else {             // Power Braking: Use the built in braking of the motors to slow down fast
            currentVelocity = Math.abs(oQlocalizer.velX_mmS); // find the actual current velocity of the robot
            if (endVelocity > MIN_END_VELOCITY) {  // Assume that end position does not need to be precise and let the robot drift to a stop (or crash into something...)
                double powerBrakingDistance = (currentVelocity-endVelocity) * POWER_BRAKING_STRAIGHT_FACTOR;
                if (details) teamUtil.log("Preparing to PowerBrake at "+ powerBrakingDistance);
                while ((distanceRemaining > powerBrakingDistance)&&teamUtil.keepGoing(timeoutTime)) {
                    loop();
                    currentPos = oQlocalizer.posX_mm;
                    distanceRemaining = (!goingUp) ? currentPos-straightTarget : straightTarget - currentPos;
                    adjustedDriveHeading = driveHeading + MathUtils.clamp((oQlocalizer.posY_mm - strafeTarget)* STRAIGHT_HEADING_DECLINATION, -STRAIGHT_MAX_DECLINATION, STRAIGHT_MAX_DECLINATION) * headingFactor;
                    if (details) {
                        teamUtil.log("dh: " + adjustedDriveHeading);
                    }
                    if (details) teamUtil.log("Extended Cruise at Velocity: "+ maxVelocity + " MMs Remaining: " + distanceRemaining);

                    driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, maxVelocity);
                    if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                        action.action();
                        actionDone=true;
                    }
                }
                if(System.currentTimeMillis()>timeoutTime){
                    teamUtil.log("TIMEOUT Triggered in Extended Cruise Phase");
                    stopMotors();
                    return false;
                }
                setMotorsBrake(); // hit the brakes hard--robot is drifting at this point
                stopMotors();
                while ((distanceRemaining > 0) && Math.abs(oQlocalizer.velX_mmS)>endVelocity && teamUtil.keepGoing(timeoutTime)) {  // wait for distance or speed to achieve goal
                    loop();
                    currentPos = oQlocalizer.posX_mm;
                    distanceRemaining = (!goingUp) ? currentPos-straightTarget : straightTarget - currentPos;
                    if (details) teamUtil.log("Power Braking: Velocity: "+ oQlocalizer.velX_mmS + " MMs Remaining: " + distanceRemaining);
                    if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                        action.action();
                        actionDone=true;
                    }
                }
                if(System.currentTimeMillis()>timeoutTime){
                    teamUtil.log("TIMEOUT Triggered while power braking");
                    stopMotors();
                    return false;
                }
                while ((distanceRemaining > 0) && teamUtil.keepGoing(timeoutTime)) { // if there is still distance to go, cruise at endVelocity
                    loop();
                    currentPos = oQlocalizer.posX_mm;
                    distanceRemaining = (!goingUp) ? currentPos-straightTarget : straightTarget - currentPos;
                    currentVelocity = endVelocity;
                    adjustedDriveHeading = driveHeading + MathUtils.clamp((oQlocalizer.posY_mm - strafeTarget)* STRAIGHT_HEADING_DECLINATION, -STRAIGHT_MAX_DECLINATION, STRAIGHT_MAX_DECLINATION) * headingFactor;
                    if (details) {
                        teamUtil.log("dh: " + adjustedDriveHeading);
                    }
                    if (details) teamUtil.log("Cruising after power braking: "+ currentVelocity + " MMs Remaining: " + distanceRemaining);
                    driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, currentVelocity);
                    if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                        action.action();
                        actionDone=true;
                    }
                }
                if (details) {
                    teamUtil.log("distance after deceleration: " + distanceRemaining);
                }
                if (requestedEndVelocity>0) { // power braking may have left the motors off. Turn them back on if requested.
                    driveMotorsHeadingsFR(driveHeading, robotHeading, requestedEndVelocity);
                }
            } else {
                teamUtil.log("POWER BRAKING with zero end velocity not implemented yet!");
                stopMotors();
                return false;
            }
        }
        if (requestedEndVelocity < 1) { // leave motors running if they didn't ask for a full stop
            if (details) teamUtil.log("Stopping motors due to requested end velocity of 0");
            stopMotors();
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return false;
        }
        setBulkReadOff();
        lastVelocity = Math.max(endVelocity,MIN_START_VELOCITY);
        teamUtil.log("driveStraightToTargetWithStrafeEncoderValue--Finished.  Current Forward Pos:" + oQlocalizer.posX_mm);
        return true;
    }

    // Strafe straight left or right while attempting to hold the forward encoder at a specific value
    // Robot heading should be 0,90,180, or 270.  Drive Heading will be determined by the target
    public boolean strafeHoldingStraightEncoder(double maxVelocity, double strafeTarget, double straightTarget, int robotHeading, double endVelocity, ActionCallback action, double actionTarget, long timeout) {
        if(robotHeading!=90&&robotHeading!=270&&robotHeading!=0&&robotHeading!=180){
            teamUtil.log("strafeHoldingStraightEncoder - ERROR INCOMPATIBLE ROBOT HEADING");
            stopMotors();
            return false;
        }

        teamUtil.log("strafeHoldingStraightEncoder target: " + strafeTarget +  " Straight target: " + straightTarget + " robotH: " + robotHeading + " MaxV: " + maxVelocity + " EndV: " + endVelocity);

        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;
        loop();
        if (details) teamUtil.log("Starting Forward Pos: "+ oQlocalizer.posY_mm);
        if (details) teamUtil.log("Starting Strafe Pos: "+ oQlocalizer.posX_mm);

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        double driveHeading;
        loop();

        double startEncoder = oQlocalizer.posY_mm;
        boolean goingUp;
        if(strafeTarget-startEncoder >=0){
            driveHeading = adjustAngle(robotHeading+90);
            goingUp = true;
        }else{
            driveHeading = adjustAngle(robotHeading-90);
            goingUp=false;
        }

        float headingFactor = goingUp? 1 : -1; // reverse correction for going backwards

        double requestedEndVelocity = endVelocity;
        if (endVelocity < MIN_STRAFE_END_VELOCITY) {
            endVelocity = MIN_STRAFE_END_VELOCITY;
        }



        if (lastVelocity == 0) {
            velocityChangeNeededAccel = maxVelocity - MIN_STRAFE_START_VELOCITY;
        } else {
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
        }
        velocityChangeNeededDecel = maxVelocity - endVelocity;
        // all are measured in tics

        double totalTics = Math.abs(startEncoder-strafeTarget);
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_STRAFE_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_STRAFE_DECELERATION);
        if (accelerationDistance+decelerationDistance >= totalTics ) { // No room for cruise phase
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");
            double accelPercentage = accelerationDistance / (accelerationDistance + decelerationDistance);
            double decelPercentage = 1-accelPercentage;
            accelerationDistance = totalTics * accelPercentage;
            decelerationDistance = totalTics * decelPercentage;
        }
        double distanceRemaining = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeadingODO());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
        }
        loop();

        distanceRemaining = Math.abs(strafeTarget - oQlocalizer.posY_mm);

        setBulkReadAuto();
        boolean actionDone = false;
        double currentPos;

        //-------Acceleration Phase
        double currentVelocity;
        double adjustedDriveHeading;
        while ((distanceRemaining > (totalTics-accelerationDistance))&&teamUtil.keepGoing(timeoutTime)) {
            loop();
            currentPos = oQlocalizer.posY_mm;
            distanceRemaining = (!goingUp) ? currentPos-strafeTarget : strafeTarget - currentPos;
            if (lastVelocity == 0) {
                currentVelocity = MAX_STRAFE_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + MIN_STRAFE_START_VELOCITY;
            } else {
                currentVelocity = MAX_STRAFE_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + lastVelocity;
            }
            adjustedDriveHeading = driveHeading + MathUtils.clamp((oQlocalizer.posX_mm - straightTarget)* STRAFE_HEADING_DECLINATION, -STRAFE_MAX_DECLINATION, STRAFE_MAX_DECLINATION) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }

            if (details) teamUtil.log("Accelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, currentVelocity);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }


        }
        if (details) {
            teamUtil.log("Heading:" + getHeadingODO());
            teamUtil.log("distance after acceleration: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Acceleration Phase");
            stopMotors();
            return false;
        }

        //-------Cruise Phase
        while ((distanceRemaining > decelerationDistance)&&teamUtil.keepGoing(timeoutTime)) {
            loop();

            currentPos = oQlocalizer.posY_mm;
            distanceRemaining = (!goingUp) ? currentPos-strafeTarget : strafeTarget - currentPos;
            adjustedDriveHeading = driveHeading + MathUtils.clamp((oQlocalizer.posX_mm - straightTarget)* STRAFE_HEADING_DECLINATION, -STRAFE_MAX_DECLINATION, STRAFE_MAX_DECLINATION) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }
            if (details) teamUtil.log("Cruising at Velocity: "+ maxVelocity + " Tics Remaining: " + distanceRemaining);

            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, maxVelocity);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }
        }
        if (details) {
            teamUtil.log("Heading:" + getHeadingODO());
            teamUtil.log("distance after cruise: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Cruise Phase");
            stopMotors();
            return false;
        }

        //-------Deceleration Phase
        while ((distanceRemaining > 0)&&teamUtil.keepGoing(timeoutTime)) {
            loop();

            currentPos = oQlocalizer.posY_mm;
            distanceRemaining = (!goingUp) ? currentPos-strafeTarget : strafeTarget - currentPos;
            currentVelocity = MAX_STRAIGHT_DECELERATION * distanceRemaining + endVelocity;
            adjustedDriveHeading = driveHeading + MathUtils.clamp((oQlocalizer.posX_mm - straightTarget)* STRAFE_HEADING_DECLINATION, -STRAFE_MAX_DECLINATION, STRAFE_MAX_DECLINATION) * headingFactor;            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }

            if (details) teamUtil.log("Decelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, currentVelocity);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }

        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distanceRemaining);
        }
        if (requestedEndVelocity < 1) {
            stopMotors();
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return false;


        }
        setBulkReadOff();
        lastVelocity = Math.max(endVelocity,MIN_STRAFE_START_VELOCITY);
        teamUtil.log("strafeHoldingStraightEncoder--Finished.  Current Strafe Encoder:" + oQlocalizer.posY_mm);
        return true;
    }

    // Drive straight forward or backward while attempting to hold the strafe encoder at a specific value
    // Robot heading should be 0,90,180, or 270.  Drive Heading will be determined by the target
    public boolean straightHoldingStrafePower(float power, double straightTarget, double strafeTarget, int robotHeading) {
        return straightHoldingStrafePower(power, straightTarget, strafeTarget, robotHeading,null, 0, 4000);
    }
    public boolean straightHoldingStrafePower(float power, double straightTarget, double strafeTarget, int robotHeading, ActionCallback action, double actionTarget, long timeout) {
        if(robotHeading != 90 && robotHeading != 270 && robotHeading != 0 && robotHeading != 180){
            teamUtil.log("straightHoldingStrafePower - ERROR INCOMPATIBLE ROBOT HEADING");
            stopMotors();
            return false;
        }
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;
        loop();
        double driveHeading;
        double startEncoder = oQlocalizer.posX_mm;
        boolean goingUp;
        if(straightTarget-startEncoder >=0){
            driveHeading = robotHeading;
            goingUp = true;
        }else{
            driveHeading = adjustAngle(robotHeading+180);
            goingUp=false;
        }
        float headingFactor = goingUp? -1 : 1; // reverse correction for going backwards

        double totalTics = Math.abs(startEncoder-straightTarget);
        teamUtil.log("straightHoldingStrafePower target: " + straightTarget +  " Strafe target: " + strafeTarget + " robotH: " + robotHeading + " Power: " + power + " TotalMMss: " + totalTics + " Starting Forward Pos: "+ oQlocalizer.posX_mm + " Starting Strafe Pos: "+ oQlocalizer.posY_mm + " Starting Heading:" + getHeadingODO());
        double distanceRemaining = Math.abs(straightTarget - oQlocalizer.posX_mm);
        setMotorsWithEncoder();
        boolean actionDone = false;
        double currentPos;
        double adjustedDriveHeading;

        //-------ONLY a CRUISE PHASE
        while ((distanceRemaining > 0) && teamUtil.keepGoing(timeoutTime)) {
            loop();
            currentPos = oQlocalizer.posX_mm;
            distanceRemaining = (!goingUp) ? currentPos-straightTarget : straightTarget - currentPos;
            adjustedDriveHeading = driveHeading + MathUtils.clamp((oQlocalizer.posY_mm - strafeTarget)* STRAIGHT_HEADING_DECLINATION, -STRAIGHT_MAX_DECLINATION, STRAIGHT_MAX_DECLINATION) * headingFactor;
            if(details)teamUtil.log("Cruising at Power: "+ power + " Adjusted Drive Heading: " + adjustedDriveHeading + " MMs Remaining: " + distanceRemaining);
            driveMotorsHeadingsFRPower(adjustedDriveHeading, robotHeading, power);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return false;
        }
        teamUtil.log("straightHoldingStrafePower--Finished.  Current Forward Pos:" + oQlocalizer.posX_mm);
        return true;
    }

    public boolean moveToX (float power, int x, int driveHeading, int robotHeading) {
        return moveToX(power, x, driveHeading, robotHeading,null, 0, 3000);
    }

    public boolean moveToX (float power, int x, int driveHeading, int robotHeading, ActionCallback action, double actionTarget, int timeout) {
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;
        loop();
        double startEncoder = oQlocalizer.posX_mm;
        boolean goingUp;
        if (x-startEncoder >=0){
            goingUp = true;
        }else{
            goingUp=false;
        }
        float headingFactor = goingUp? -1 : 1; // reverse correction for going backwards
        double totalTics = Math.abs(startEncoder-x);
        teamUtil.log("moveToX target: " + x + " driveH: " + driveHeading + " robotH: " + robotHeading + " Power: " + power + " TotalMMss: " + totalTics + " Starting Forward Pos: "+ oQlocalizer.posX_mm + " Starting Strafe Pos: "+ oQlocalizer.posY_mm + " Starting Heading:" + getHeadingODO());
        double distanceRemaining = Math.abs(x - oQlocalizer.posX_mm);
        setMotorsWithEncoder();
        boolean actionDone = false;
        double currentPos;
        while ((distanceRemaining > 0) && teamUtil.keepGoing(timeoutTime)) {
            loop();
            currentPos = oQlocalizer.posX_mm;
            distanceRemaining = (!goingUp) ? currentPos-x : x - currentPos;
            if(details)teamUtil.log("Cruising at Power: "+ power + " MMs Remaining: " + distanceRemaining);
            driveMotorsHeadingsFRPower(driveHeading, robotHeading, power);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return false;
        }
        teamUtil.log("moveToX--Finished.  Current Forward Pos:" + oQlocalizer.posX_mm);
        return true;
    }

    public boolean moveToY (float power, int y, int driveHeading, int robotHeading) {
        return moveToY(power, y, driveHeading, robotHeading,null, 0, 3000);
    }

    public boolean moveToY (float power, int y, int driveHeading, int robotHeading, ActionCallback action, double actionTarget, int timeout) {
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;
        loop();
        double startEncoder = oQlocalizer.posY_mm;
        boolean goingUp;
        if (driveHeading > 0 && driveHeading < 180){
            goingUp = true;
        }else{
            goingUp=false;
        }
        /*
        if (y-startEncoder >=0){
            goingUp = true;
        }else{
            goingUp=false;
        }
         */
        double totalTics = Math.abs(startEncoder-y);
        teamUtil.log("moveToY target: " + y + " driveH: " + driveHeading + " robotH: " + robotHeading + " Power: " + power + " TotalMMss: " + totalTics + " Starting Forward Pos: "+ oQlocalizer.posX_mm + " Starting Strafe Pos: "+ oQlocalizer.posY_mm + " Starting Heading:" + getHeadingODO());
        double distanceRemaining = Math.abs(y - oQlocalizer.posY_mm);
        setMotorsWithEncoder();
        boolean actionDone = false;
        double currentPos;
        while ((distanceRemaining > 0) && teamUtil.keepGoing(timeoutTime)) {
            loop();
            currentPos = oQlocalizer.posY_mm;
            distanceRemaining = (!goingUp) ? currentPos-y : y - currentPos;
            if(details)teamUtil.log("Cruising at Power: "+ power + " MMs Remaining: " + distanceRemaining);
            driveMotorsHeadingsFRPower(driveHeading, robotHeading, power);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return false;
        }
        teamUtil.log("moveToY--Finished.  Current Forward Pos:" + oQlocalizer.posY_mm);
        return true;
    }


    // Strafe straight left or right while attempting to hold the forward encoder at a specific value
    // Robot heading should be 0,90,180, or 270.  Drive Heading will be determined by the target
    public boolean strafeHoldingStraightPower(float power, double strafeTarget, double straightTarget, int robotHeading) {
        return strafeHoldingStraightPower(power, strafeTarget, straightTarget, robotHeading,null, 0, 4000);
    }
    public boolean strafeHoldingStraightPower(float power, double strafeTarget, double straightTarget, int robotHeading, ActionCallback action, double actionTarget, long timeout) {
        if(robotHeading!=90&&robotHeading!=270&&robotHeading!=0&&robotHeading!=180){
            teamUtil.log("strafeHoldingStraightPower - ERROR INCOMPATIBLE ROBOT HEADING");
            stopMotors();
            return false;
        }
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;
        loop();
        double driveHeading;
        double startEncoder = oQlocalizer.posY_mm;
        boolean goingUp;
        if(strafeTarget-startEncoder >=0){
            driveHeading = adjustAngle(robotHeading+90);
            goingUp = true;
        }else{
            driveHeading = adjustAngle(robotHeading-90);
            goingUp=false;
        }
        float headingFactor = goingUp? 1 : -1; // reverse correction for going backwards


        double totalTics = Math.abs(startEncoder-strafeTarget);
        teamUtil.log("strafeHoldingStraightPower target: " + strafeTarget +  " Straight target: " + straightTarget + " robotH: " + robotHeading + " Power: " + power + " TotalMMss: " + totalTics + " Starting Forward Pos: "+ oQlocalizer.posX_mm + " Starting Strafe Pos: "+ oQlocalizer.posY_mm + " Starting Heading:" + getHeadingODO());
        loop();
        double distanceRemaining = Math.abs(strafeTarget - oQlocalizer.posY_mm);
        setMotorsWithEncoder();
        boolean actionDone = false;
        double currentPos;
        double adjustedDriveHeading;

        //-------ONLY a Cruise Phase
        while ((distanceRemaining > 0) && teamUtil.keepGoing(timeoutTime)) {
            loop();
            currentPos = oQlocalizer.posY_mm;
            distanceRemaining = (!goingUp) ? currentPos-strafeTarget : strafeTarget - currentPos;
            adjustedDriveHeading = driveHeading + MathUtils.clamp((oQlocalizer.posX_mm - straightTarget)* STRAFE_HEADING_DECLINATION, -STRAFE_MAX_DECLINATION, STRAFE_MAX_DECLINATION) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }
            if (details) teamUtil.log("Cruising at Power: "+ power + " Adjusted Drive Heading: " + adjustedDriveHeading + " MMs Remaining: " + distanceRemaining);
            driveMotorsHeadingsFRPower(adjustedDriveHeading, robotHeading, power);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return false;
        }
        teamUtil.log("strafeHoldingStraightPower--Finished.  Current Strafe Encoder:" + oQlocalizer.posY_mm);
        return true;
    }

    public static double distanceToLine(double targetX, double targetY, double driveHeading, double posX, double posY) {
        // Convert angle to radians
        double angleRadians = Math.toRadians(driveHeading);
        // Compute A, B, and C for the line equation Ax + By + C = 0
        double A = -Math.sin(angleRadians);
        double B = Math.cos(angleRadians);
        double C = -A * targetX - B * targetY;
        // Compute signed perpendicular distance
        double distance = (A * posX + B * posY + C) / Math.sqrt(A * A + B * B);
        return distance;
    }

    public static double POWER_DECEL_COEFF = .001;
    public boolean moveToXHoldingStrafe(float power, double xTarget, double yTarget, int driveHeading, int robotHeading, float endPower, ActionCallback action, double actionTarget, long timeout) {
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;
        loop();
        double startEncoder = oQlocalizer.posX_mm;
        boolean goingUp;
        if(xTarget-startEncoder >=0){
            goingUp = true;
        }else{
            goingUp=false;
        }
        if (endPower == 0) {
            endPower = MIN_END_POWER;
        }
        double totalTics = Math.abs(startEncoder-xTarget);
        teamUtil.log("moveToXHoldingStrafe xTarget: " + xTarget +  " yTarget: " + yTarget + " robotH: " + robotHeading + " driveH: " + driveHeading + " Power: " + power + " TotalMMss: " + totalTics + " Starting Forward Pos: "+ oQlocalizer.posX_mm + " Starting Strafe Pos: "+ oQlocalizer.posY_mm + " Starting Heading:" + getHeadingODO());
        double distanceRemaining = Math.abs(xTarget - oQlocalizer.posX_mm);
        setMotorsWithEncoder();
        boolean actionDone = false;
        double currentPos;
        double adjustedDriveHeading;
        float adjustedPower;

        //-------ONLY a CRUISE PHASE (but with deceleration)
        while ((distanceRemaining > 0) && teamUtil.keepGoing(timeoutTime)) {
            loop();
            currentPos = oQlocalizer.posX_mm;
            distanceRemaining = (!goingUp) ? currentPos-xTarget : xTarget - currentPos;
            adjustedPower = (float) MathUtils.clamp(distanceRemaining* +POWER_DECEL_COEFF + endPower,endPower, power);
            adjustedDriveHeading = driveHeading - MathUtils.clamp(distanceToLine(xTarget, yTarget, driveHeading, oQlocalizer.posX_mm, oQlocalizer.posY_mm)* STRAIGHT_HEADING_DECLINATION, -STRAIGHT_MAX_DECLINATION, STRAIGHT_MAX_DECLINATION);
            if(details)teamUtil.log("Driving at Power: "+ adjustedPower + " Adjusted Drive Heading: " + adjustedDriveHeading + " X MMs Remaining: " + distanceRemaining);
            driveMotorsHeadingsFRPower(adjustedDriveHeading, robotHeading, adjustedPower);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }
        }

        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return false;
        }
        teamUtil.log("moveToXHoldingStrafe--Finished.  Current xPos:" + oQlocalizer.posX_mm + " Current yPos: "+ oQlocalizer.posY_mm);
        return true;
    }
    public boolean moveToYHoldingLine(float power, double yTarget, double xTarget, int driveHeading, int robotHeading, float endPower, ActionCallback action, double actionTarget, long timeout) {
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;
        loop();
        double startEncoder = oQlocalizer.posY_mm;
        boolean goingUp;
        if(yTarget-startEncoder >=0){
            goingUp = true;
        }else{
            goingUp=false;
        }
        if (endPower == 0) {
            endPower = MIN_END_POWER;
        }
        double totalTics = Math.abs(startEncoder-yTarget);
        teamUtil.log("moveToYHoldingLine yTarget: " + yTarget +  " xTarget: " + xTarget + " robotH: " + robotHeading + " driveH: " + driveHeading + " Power: " + power + " TotalMMss: " + totalTics + " Starting Forward Pos: "+ oQlocalizer.posX_mm + " Starting Strafe Pos: "+ oQlocalizer.posY_mm + " Starting Heading:" + getHeadingODO());
        double distanceRemaining = Math.abs(yTarget - oQlocalizer.posY_mm);
        setMotorsWithEncoder();
        boolean actionDone = false;
        double currentPos;
        double adjustedDriveHeading;
        float adjustedPower;

        //-------ONLY a CRUISE PHASE (but with deceleration)
        while ((distanceRemaining > 0) && teamUtil.keepGoing(timeoutTime)) {
            loop();
            currentPos = oQlocalizer.posY_mm;
            distanceRemaining = (!goingUp) ? currentPos-yTarget : yTarget - currentPos;
            adjustedPower = (float) MathUtils.clamp(distanceRemaining* +POWER_DECEL_COEFF + endPower,endPower, power);
            adjustedDriveHeading = driveHeading - MathUtils.clamp(distanceToLine(xTarget, yTarget, driveHeading, oQlocalizer.posX_mm, oQlocalizer.posY_mm)* STRAIGHT_HEADING_DECLINATION, -STRAIGHT_MAX_DECLINATION, STRAIGHT_MAX_DECLINATION);
            if(details)teamUtil.log("Driving at Power: "+ adjustedPower + " Adjusted Drive Heading: " + adjustedDriveHeading + " X MMs Remaining: " + distanceRemaining);
            driveMotorsHeadingsFRPower(adjustedDriveHeading, robotHeading, adjustedPower);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }
        }

        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return false;
        }
        teamUtil.log("moveToYHoldingLine--Finished.  Current yPos:" + oQlocalizer.posY_mm + " Current xPos: "+ oQlocalizer.posX_mm);
        return true;
    }




    public void moveToV2(double maxVelocity, double strafeTarget, double straightTarget, double robotHeading, double endVelocity,ActionCallback action, double actionTarget, boolean endInDeadband, long timeout){
        teamUtil.log("MoveTo StrafeTarget: " + strafeTarget +  " Straight target: " + straightTarget + " robotH: " + robotHeading + " MaxV: " + maxVelocity + " EndV: " + endVelocity + " EndInDeadband: " + endInDeadband);

        //TODO THIS CODE SHALL NOT BE USED UNTIL THE ANGLE PROBLEM IS FIXED

        teamUtil.log("moveTo");
        loop();
        long timeoutTime = System.currentTimeMillis()+timeout;
        boolean withinThreshold = false;
        boolean strafeTargetAchieved = false;
        boolean straightTargetAchieved = false;
        boolean strafeIncreasing = (strafeTarget - oQlocalizer.posY_mm > 0);
        boolean straightIncreasing = (straightTarget - oQlocalizer.posX_mm > 0);
        if(endVelocity<MIN_END_VELOCITY){
            endVelocity=MIN_END_VELOCITY;
            lastVelocity=0;
        }
        else{
            lastVelocity=endVelocity;
        }
        double angle = 0;
        double driveHeading;
        double velocity;

        while(!withinThreshold&&teamUtil.keepGoing(timeoutTime)) {
            loop();
            double straightChange = straightTarget - oQlocalizer.posX_mm;
            double strafeChange = strafeTarget - oQlocalizer.posY_mm;

            if(strafeChange==0){
                driveHeading = straightChange>0? 0:180;
            } else if (straightChange==0) {
                driveHeading = strafeChange>0? 90:270;
            }else {
                angle = Math.toDegrees(Math.atan(straightChange / strafeChange));
                if (straightChange > 0) {
                    if (strafeChange > 0) {
                        driveHeading=90-angle;
                    } else {
                        driveHeading=270-angle;
                    }
                } else {
                    if (strafeChange > 0) {
                        driveHeading=90-angle;
                    } else {
                        driveHeading=270-angle;
                    }
                }
            }

            double remainingDistance = Math.sqrt(straightChange * straightChange + strafeChange * strafeChange);
            if(remainingDistance<=MOVE_TO_THRESHOLD+15){
                velocity = Math.max(MOVE_TO_DECCEL_COEFFICIENT * remainingDistance+endVelocity, endVelocity);
            } else{
                velocity = Math.min(MOVE_TO_COEFFICIENT * remainingDistance+endVelocity, maxVelocity);
            }

            if(endInDeadband){ // We must be at or near the target
                if (remainingDistance <= MOVE_TO_THRESHOLD) {
                    withinThreshold = true;
                }
            }else{ // We just need to exceed the targets in both dimensions once
                strafeTargetAchieved = strafeTargetAchieved || strafeIncreasing ? oQlocalizer.posY_mm > strafeTarget - MOVE_TO_THRESHOLD : oQlocalizer.posY_mm < strafeTarget + MOVE_TO_THRESHOLD;
                straightTargetAchieved = straightTargetAchieved || straightIncreasing ? oQlocalizer.posX_mm > straightTarget - MOVE_TO_THRESHOLD : oQlocalizer.posX_mm < straightTarget + MOVE_TO_THRESHOLD;
                withinThreshold = strafeTargetAchieved && straightTargetAchieved;
                /*
                if(driveHeading>=0&&driveHeading<90){
                    if((strafeChange<MOVE_TO_THRESHOLD)&&(straightChange<MOVE_TO_THRESHOLD)){
                        withinThreshold=true;
                    }
                }else if(driveHeading<270&&driveHeading>=180){
                    if((strafeChange>-MOVE_TO_THRESHOLD)&&(straightChange>-MOVE_TO_THRESHOLD)){
                        withinThreshold=true;
                    }
                }else if(driveHeading<=360&&driveHeading>=270){
                    if((strafeChange>-MOVE_TO_THRESHOLD)&&(straightChange<MOVE_TO_THRESHOLD)){
                        withinThreshold=true;
                    }
                }else{
                    if((strafeChange<MOVE_TO_THRESHOLD)&&(straightChange>-MOVE_TO_THRESHOLD)){
                        withinThreshold=true;
                    }
                }
                 */
            }
            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);


            if (details) {
                teamUtil.log(String.format("Raw Angle %.1f DH: %.1f RH: %.1f Vel: %.0f Distance: %.0f X: %.0f Y %.0f", angle, driveHeading, robotHeading, velocity, remainingDistance, strafeChange, straightChange));
                // teamUtil.log("Raw Angle: " + angle + " DH: " + driveHeading + " RH: " + robotHeading + " Vel: " + velocity + " Distance " + remainingDistance + " X: " + strafeChange + " Y: " + straightChange);
            }
        }
        // TODO Make lastVelocity accurate
        if(lastVelocity<=MIN_END_VELOCITY){
            stopMotors();
        }

        teamUtil.log("MoveTo FINISHED");
    }



    public void moveTo(double maxVelocity, double strafeTarget, double straightTarget, double robotHeading, double endVelocity,ActionCallback action, double actionTarget, long timeout){
        moveTo(maxVelocity,strafeTarget,straightTarget,robotHeading,endVelocity,action,actionTarget,true,timeout);
    }

    public void moveTo(double maxVelocity, double strafeTarget, double straightTarget, double robotHeading, double endVelocity,ActionCallback action, double actionTarget, boolean endInDeadband, long timeout){
        teamUtil.log("MoveTo StrafeTarget: " + strafeTarget +  " Straight target: " + straightTarget + " robotH: " + robotHeading + " MaxV: " + maxVelocity + " EndV: " + endVelocity + " EndInDeadband: " + endInDeadband);
        loop();
        long timeoutTime = System.currentTimeMillis()+timeout;
        boolean withinThreshold = false;
        boolean strafeTargetAchieved = false;
        boolean straightTargetAchieved = false;
        boolean strafeIncreasing = (strafeTarget - oQlocalizer.posY_mm > 0);
        boolean straightIncreasing = (straightTarget - oQlocalizer.posX_mm > 0);
        if(endVelocity<MIN_END_VELOCITY){
            endVelocity=MIN_END_VELOCITY;
            lastVelocity=0;
        }
        else{
            lastVelocity=endVelocity;
        }
        double angle = 0;
        double driveHeading;
        setMotorsWithEncoder();

        while(!withinThreshold&&teamUtil.keepGoing(timeoutTime)) {
            loop();
            double straightChange = straightTarget - oQlocalizer.posX_mm;
            double strafeChange = strafeTarget - oQlocalizer.posY_mm;

            if(strafeChange==0){
                driveHeading = straightChange>0? 0:180;
            } else if (straightChange==0) {
                driveHeading = strafeChange>0? 90:270;
            }else {
                angle = Math.toDegrees(Math.atan(straightChange / strafeChange));
                if (straightChange > 0) {
                    if (strafeChange > 0) {
                        driveHeading=90-angle;
                    } else {
                        driveHeading=270-angle;
                    }
                } else {
                    if (strafeChange > 0) {
                        driveHeading=90-angle;
                    } else {
                        driveHeading=270-angle;
                    }
                }
            }

            double remainingDistance = Math.sqrt(straightChange * straightChange + strafeChange * strafeChange);

            double velocity = Math.min(MOVE_TO_COEFFICIENT * remainingDistance+endVelocity, maxVelocity);

            if(endInDeadband){ // We must be at or near the target
                if (remainingDistance <= MOVE_TO_THRESHOLD) {
                    withinThreshold = true;
                }
            }else{ // We just need to exceed the targets in both dimensions once
                strafeTargetAchieved = strafeTargetAchieved || strafeIncreasing ? oQlocalizer.posY_mm > strafeTarget - MOVE_TO_THRESHOLD : oQlocalizer.posY_mm < strafeTarget + MOVE_TO_THRESHOLD;
                straightTargetAchieved = straightTargetAchieved || straightIncreasing ? oQlocalizer.posX_mm > straightTarget - MOVE_TO_THRESHOLD : oQlocalizer.posX_mm < straightTarget + MOVE_TO_THRESHOLD;
                withinThreshold = strafeTargetAchieved && straightTargetAchieved;
           }
            driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            if (details) {
                teamUtil.log(String.format("Raw Angle %.1f DH: %.1f RH: %.1f Vel: %.0f Distance: %.0f X: %.0f Y %.0f", angle, driveHeading, robotHeading, velocity, remainingDistance, strafeChange, straightChange));
                // teamUtil.log("Raw Angle: " + angle + " DH: " + driveHeading + " RH: " + robotHeading + " Vel: " + velocity + " Distance " + remainingDistance + " X: " + strafeChange + " Y: " + straightChange);
            }
        }
        // TODO Make lastVelocity accurate
        if(lastVelocity<=MIN_END_VELOCITY){
            stopMotors();
        }

        teamUtil.log("MoveTo FINISHED Heading: " + Math.toDegrees(oQlocalizer.heading_rad) + " Xpos: " + oQlocalizer.posX_mm + " Ypos: " + oQlocalizer.posY_mm);
    }

    public void moveToPower(float maxPower, double strafeTarget, double straightTarget, double robotHeading, long timeout){
        moveToPower(maxPower,strafeTarget,straightTarget,robotHeading,0,null,0,true,timeout);
    }
    public void moveToPower(float maxPower, double strafeTarget, double straightTarget, double robotHeading, boolean endInDeadband, long timeout){
        moveToPower(maxPower,strafeTarget,straightTarget,robotHeading,0,null,0,endInDeadband,timeout);
    }
    public void moveToPower(float maxPower, double strafeTarget, double straightTarget, double robotHeading, float endPower,ActionCallback action, double actionTarget, long timeout){
        moveToPower(maxPower,strafeTarget,straightTarget,robotHeading,endPower,action,actionTarget,true,timeout);
    }

    public double computeDriveHeading(double strafeChange, double straightChange) {
        if(strafeChange==0){
            return straightChange>0? 0:180;
        } else if (straightChange==0) {
            return strafeChange>0? 90:270;
        }else {
            double angle = Math.toDegrees(Math.atan(straightChange / strafeChange));
            if (straightChange > 0) {
                if (strafeChange > 0) {
                    return 90-angle;
                } else {
                    return 270-angle;
                }
            } else {
                if (strafeChange > 0) {
                    return 90-angle;
                } else {
                    return 270-angle;
                }
            }
        }
    }

    public void moveToPower(float maxPower, double strafeTarget, double straightTarget, double robotHeading, float  endPower,ActionCallback action, double actionTarget, boolean endInDeadband, long timeout){
        teamUtil.log("moveToPower StrafeTarget: " + strafeTarget +  " Straight target: " + straightTarget + " robotH: " + robotHeading + " MaxV: " + maxPower + " EndV: " + endPower + " EndInDeadband: " + endInDeadband);
        loop();
        long timeoutTime = System.currentTimeMillis()+timeout;
        boolean withinThreshold = false;
        boolean strafeTargetAchieved = false;
        boolean straightTargetAchieved = false;
        boolean strafeIncreasing = (strafeTarget - oQlocalizer.posY_mm > 0);
        boolean straightIncreasing = (straightTarget - oQlocalizer.posX_mm > 0);
        endPower = Math.max(endPower, MIN_END_POWER);
        double driveHeading;
        setMotorsRunWithoutEncoder();
        while(!withinThreshold&&teamUtil.keepGoing(timeoutTime)) {
            loop();
            double straightChange = straightTarget - oQlocalizer.posX_mm;
            double strafeChange = strafeTarget - oQlocalizer.posY_mm;

            driveHeading = computeDriveHeading(strafeChange, straightChange);
            double remainingDistance = Math.sqrt(straightChange * straightChange + strafeChange * strafeChange);
            float power = Math.min(MOVE_TO_POWER_COEFFICIENT * (float) remainingDistance + endPower, maxPower);

            if(endInDeadband){ // We must be at or near the target
                if (remainingDistance <= MOVE_TO_THRESHOLD) {
                    withinThreshold = true;
                }
            }else{ // We just need to exceed the targets in both dimensions once
                strafeTargetAchieved = strafeTargetAchieved || strafeIncreasing ? oQlocalizer.posY_mm > strafeTarget - MOVE_TO_THRESHOLD : oQlocalizer.posY_mm < strafeTarget + MOVE_TO_THRESHOLD;
                straightTargetAchieved = straightTargetAchieved || straightIncreasing ? oQlocalizer.posX_mm > straightTarget - MOVE_TO_THRESHOLD : oQlocalizer.posX_mm < straightTarget + MOVE_TO_THRESHOLD;
                withinThreshold = strafeTargetAchieved && straightTargetAchieved;
            }
            driveMotorsHeadingsFRPower(driveHeading, robotHeading, power);

            if (details) {
                teamUtil.log(String.format("DH: %.1f RH: %.1f Power: %.2f Distance: %.0f X: %.0f Y %.0f", driveHeading, robotHeading, power, remainingDistance, strafeChange, straightChange));
            }
        }
        if(endPower<=MIN_END_POWER){
            stopMotors();
            teamUtil.log("Stopping Motors");
        }

        teamUtil.log("MoveToPower FINISHED Heading: " + Math.toDegrees(oQlocalizer.heading_rad) + " Xpos: " + oQlocalizer.posX_mm + " Ypos: " + oQlocalizer.posY_mm);
    }


    // This was developed for CS April Tag Localization.  Might be some stuff here useful for the new "moveTo" odometry pod method
    public void backToPoint(double robotHeading, double x, double y, double endVelocity) { // assumes robot heading is 180
        // x positive means to the robots right
        // y positive means robot move backwards (not tested for anything else!)
        double heading, distance;
        teamUtil.log("Move to Point: x/y " + x + "/"+ y);
        distance = Math.sqrt(x*x+y*y);
        if (y == 0) {
            heading = x < 0 ? 270 : 90;
        } else if (y > 0) { // Using vertical (y-axis) to compute reference angles since 0 is at top
            heading = adjustAngle(Math.toDegrees(Math.atan(x / y)));
        } else {
            heading = 180 + Math.toDegrees(Math.atan(x / y));
        }
        moveCm(MAX_VELOCITY,distance,heading,180,endVelocity);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Stall detection using encoders
/*
    public boolean waitForStall(long timeout){
        // wait for the robot to slow down on the wall
        // expects setPower
        teamUtil.log("Waiting For Stall");
        long timeoutTime = System.currentTimeMillis()+timeout;
        int lastEncoder = forwardEncoder.getCurrentPosition();
        double startEncoderVelocity = forwardEncoder.getVelocity();
        while(teamUtil.keepGoing(timeoutTime)){
            teamUtil.pause(25);
            if(details){teamUtil.log("Forward Encoder Velocity: " + forwardEncoder.getVelocity());}
            int currentEncoder = forwardEncoder.getCurrentPosition();
            if (details) teamUtil.log("last: " + lastEncoder + " current: "+ currentEncoder);
            if(forwardEncoder.getVelocity()<startEncoderVelocity*0.5){
                teamUtil.log("Stalled");
                return true;
            }
            lastEncoder = currentEncoder;
        }
        teamUtil.log("Didn't Stall");
        return false; // didn't get a stall
    }
*/

    /************************************************************************************************************/
    // Methods to turn the robot in place

    public void spinToHeading(double heading) {
        // moves at full speed then decelerates to spin
        double velocity = MAX_VELOCITY;
        boolean turningLeft;
        double startHeading = getHeadingODO();
        double currentHeading = getHeadingODO();
        double leftCoefficient = 1;
        double rightCoefficient = 1;
        setMotorsWithEncoder();
        if (heading > currentHeading) { // fix direction
            if (heading - currentHeading < 180) {
                leftCoefficient = -1;
            } else {
                rightCoefficient = -1;
            }
        } else {
            if (currentHeading - heading < 180) {
                rightCoefficient = -1;
            } else {
                leftCoefficient = -1;
            }
        }
        if (details) {
            teamUtil.log("turning left: " + rightCoefficient);
            teamUtil.log("current heading: " + currentHeading);
            teamUtil.log("heading goal: " + (heading + SPIN_DRIFT_DEGREES));
        }
        if (details) {
            teamUtil.log("crossing 0/360 barrier");
        }
        while (Math.abs(currentHeading - heading) > SPIN_DECEL_THRESHOLD_DEGREES) {
            setMotorVelocities(leftCoefficient * velocity, rightCoefficient * velocity, leftCoefficient * velocity, rightCoefficient * velocity);
            loop();
            //TODO FIND OUT HOW TO UPDATE LOCALIZERS HEADING INFO ONLY
            //odo.update(Pinpoint.readData.ONLY_UPDATE_HEADING);
            currentHeading = getHeadingODO();
        }
        if (details) {
            teamUtil.log("current heading: " + currentHeading);
            teamUtil.log("heading cutoff (greater): " + adjustAngle(heading - SPIN_CRAWL_DEGREES));
            teamUtil.log("done with max velocity phase");
            teamUtil.log("heading: " + currentHeading);
        }
        while (Math.abs(currentHeading - heading) > SPIN_CRAWL_DEGREES) {
            loop();
            //TODO FIND OUT HOW TO UPDATE LOCALIZERS HEADING INFO ONLY

            //odo.update(Pinpoint.readData.ONLY_UPDATE_HEADING);
            currentHeading = getHeadingODO();
            velocity = ((MAX_VELOCITY - SPIN_CRAWL_SPEED) / (SPIN_DECEL_THRESHOLD_DEGREES - SPIN_CRAWL_DEGREES)) * (Math.abs(currentHeading - heading) - SPIN_DECEL_THRESHOLD_DEGREES) + MAX_VELOCITY; // wrote an equasion
            if (velocity < SPIN_CRAWL_SPEED) {
                velocity = SPIN_CRAWL_SPEED;
            }
            setMotorVelocities(leftCoefficient * velocity, rightCoefficient * velocity, leftCoefficient * velocity, rightCoefficient * velocity);
        }

        if (details) {
            teamUtil.log("done with deceleration phase");
            teamUtil.log("heading: " + currentHeading);
        }
        while (Math.abs(currentHeading - heading) > SPIN_DRIFT_DEGREES) {
            loop();
            //TODO FIND OUT HOW TO UPDATE LOCALIZERS HEADING INFO ONLY

            //odo.update(Pinpoint.readData.ONLY_UPDATE_HEADING);
            currentHeading = getHeadingODO();
            velocity = SPIN_CRAWL_SPEED;
            setMotorVelocities(leftCoefficient * velocity, rightCoefficient * velocity, leftCoefficient * velocity, rightCoefficient * velocity);
        }

        if (details) {
            teamUtil.log("done with crawl phase");
            teamUtil.log("heading: " + currentHeading);
        }

        setMotorsBrake();
        setMotorPower(0);
    }



    /************************************************************************************************************/
    //Methods to drive based on joystick values

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Drive robot based on two joystick values
    // Implements a deadband where joystick position will be ignored (translation and rotation)
    // Uses a linear scale that starts at the edge of the dead band
    // Attempts to hold the last heading that was commanded via a turn
    public void driveJoyStickV2(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, boolean isFast, boolean isSlow) {

        float SLOPE = 0.55f;
        float FASTSLOPE = 1f;
        float SLOWSPEED = .1f;
        //float STRAFESLOWSPEED = 0.25f;
        //float SLOPE = 1 / (1 - DEADBAND); // linear from edge of dead band to full power  TODO: Should this be a curve?

        float POWERFACTOR = 1; // MAKE LESS THAN 0 to cap upperlimit of power
        float leftX;
        float leftY;
        float rotationAdjustment;

        float scaleAmount = isFast ? 1f : 0.5f; // full power is "fast", half power is "slow"

        float frontLeft, frontRight, backRight, backLeft;

        // Ensure nothing happens if we are inside the deadband
        if (Math.abs(leftJoyStickX) < DEADBAND) {
            leftJoyStickX = 0;
        }
        if (Math.abs(leftJoyStickY) < DEADBAND) {
            leftJoyStickY = 0;
        }
        if (Math.abs(rightJoyStickX) < SPIN_DEADBAND) {
            rightJoyStickX = 0;
        }

        if (movingAutonomously.get() && (leftJoyStickX != 0 || rightJoyStickX != 0 || leftJoyStickY != 0)) { // Do we need to interrupt an autonomous operation?
            manualInterrupt.set(true);
        }
        if(leftJoyStickX == 0){ // NEW Power Curves
            leftX = 0;
        } else if(isSlow){
            leftX = leftJoyStickX*SLOWSLOPESTRAFE+(leftJoyStickX>0? -0.078f:0.078f);
        } else if(isFast){
            leftX = leftJoyStickX * FASTSLOPE ;
        }
        //medium speed
        else{
            leftX = leftJoyStickX*SLOPE+(leftJoyStickX>0? -0.055f:0.055f);

        }

        if(leftJoyStickY == 0){
            leftY = 0;
        } else if(isSlow){
            leftY = leftJoyStickY*SLOWSLOPE+(leftJoyStickY>0? -0.078f:0.078f);
        } else if(isFast){
            leftY = leftJoyStickY * FASTSLOPE ;
        }
        //medium speed
        else{
            leftY = leftJoyStickY*SLOPE+(leftJoyStickY>0? -0.055f:0.055f);

        }

        final float MAXROTATIONFACTOR = 0.8f;
        if (Math.abs(rightJoyStickX) > DEADBAND) { // driver is turning the robot
            //old code
            //rotationAdjustment = (float) (rightJoyStickX * 0.525 * scaleAmount);
            rotationAdjustment = (float) (rightJoyStickX * 1 * scaleAmount);

            holdingHeading = false;
        } else { // Need to automatically hold the current heading
            if (!holdingHeading) { // start to hold current heading
                heldHeading = getHeadingODO();
                holdingHeading = true;
                if(details){
                    teamUtil.log("New heldHeading from turn: "+heldHeading);
                }
            }
            // old code
            //rotationAdjustment = (float) getHeadingError(heldHeading) * -1f * .05f; // auto rotate to held heading
            rotationAdjustment = (float) getHeadingError(heldHeading) * -1f * ROTATION_ADJUST_HELD_HEADING; // auto rotate to held heading
            rotationAdjustment = rotationAdjustment * Math.min(Math.max(Math.abs(leftX), Math.abs(leftY)), 0.7f); // make it proportional to speed
            rotationAdjustment = MathUtils.clamp(rotationAdjustment, -MAXROTATIONFACTOR,MAXROTATIONFACTOR ); // clip rotation so it doesn't obliterate translation
        }
        frontLeft = -(leftY - leftX - rotationAdjustment);
        frontRight = (-leftY - leftX - rotationAdjustment);
        backRight = (-leftY + leftX - rotationAdjustment);
        backLeft = -(leftY + leftX - rotationAdjustment); // TODO: fix powers being above 1

        if (details) {
            teamUtil.telemetry.addLine("Joy X/Y: "+ leftJoyStickX+ "/"+ leftJoyStickY+ " X/Y: "+ leftX+ "/"+leftY);
            if (holdingHeading) {
                teamUtil.telemetry.addData("HOLDING:", heldHeading);
            }
        }
        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        br.setPower(backRight);
        bl.setPower(backLeft);

        String currentSpeedState;
        if(isSlow){
            currentSpeedState="Is Slow";
        }else if(isFast){
            currentSpeedState="Is Fast";
        }else{
            currentSpeedState="Is Medium";
        }
        //telemetry.addLine("Current Speed State " + currentSpeedState);

        //TODO: take out soon just for testing purposes
        //telemetry.addLine("Left Joystick Y: " + leftJoyStickY);
        //telemetry.addLine("Left Joystick X: " + leftJoyStickX);
        //telemetry.addLine("fl power: " + frontLeft);
        //telemetry.addLine("fr power: " + frontRight);
        //telemetry.addLine("bl power: " + backLeft);
        //telemetry.addLine("br power: " + backRight);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Adds Field Relative driving to driveJoyStick
    public void universalDriveJoystickV2(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, boolean isFast,boolean isSlow, double robotHeading) {
        double angleInRadians = robotHeading * Math.PI / 180;
        float leftX = leftJoyStickX;
        float leftY = leftJoyStickY;
        float rightX = rightJoyStickX;

        //rotate to obtain new coordinates
        float rotatedLeftX = (float) (Math.cos(angleInRadians) * leftX - Math.sin(angleInRadians) * leftY);
        float rotatedLeftY = (float) (Math.sin(angleInRadians) * leftX + Math.cos(angleInRadians) * leftY);

        driveJoyStickV2(rotatedLeftX, rotatedLeftY, rightX, isFast,isSlow);
    }

    public void setHeldHeading(double heading){
        holdingHeading = true;
        heldHeading = heading;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Testing Code

    public void findMaxVelocity(int cmDistance) {
        // set motors to 3000 (theoretical max) then see how fast they actually go
        long startTime = System.currentTimeMillis();
        teamUtil.log("Finding Forward Max Velocities...");
        resetAllDriveEncoders();
        double travelTics = COUNTS_PER_CENTIMETER * cmDistance;
        setMotorVelocities(3000, 3000, 3000, 3000);
        double flmax = 0, frmax = 0, blmax = 0, brmax = 0, v;
        double ticStartPosition = fr.getCurrentPosition();
        while (fr.getCurrentPosition() < travelTics) {
            flmax = (v = fl.getVelocity()) > flmax ? v : flmax;
            frmax = (v = fr.getVelocity()) > frmax ? v : frmax;
            blmax = (v = bl.getVelocity()) > blmax ? v : blmax;
            brmax = (v = br.getVelocity()) > brmax ? v : brmax;
            teamUtil.log("Looping FL:" + flmax + " FR:" + frmax + " BL:" + blmax + " BR:" + brmax);
        }
        stopMotors();
        long elapsedTime = System.currentTimeMillis()-startTime;
        double ticsTraveled = fr.getCurrentPosition()-ticStartPosition;
        double cmsTraveled = ticsTraveled/COUNTS_PER_CENTIMETER;
        double timeS = elapsedTime/1000.0;
        teamUtil.log("Elapsed Time: " + elapsedTime);
        teamUtil.log("Tics Traveled: " + ticsTraveled);
        teamUtil.log("Cms Traveled: " + cmsTraveled);
        teamUtil.log("Cms Per Second: " + cmsTraveled/timeS);

        teamUtil.log("Forward Max Velocities FL:" + flmax + " FR:" + frmax + " BL:" + blmax + " BR:" + brmax);
    }

    public void findMaxStrafeVelocity(double distance){
        // same as above
        setHeading(180);
        long startTime = System.currentTimeMillis();
        teamUtil.log("Finding Strafing Max Velocities...");
        resetAllDriveEncoders();
        double travelTics = distance*COUNTS_PER_CENTIMETER;
        teamUtil.log("Travel Tics: " + travelTics);
        double ticStartPosition = fr.getCurrentPosition();
        driveMotorsHeadingsFR(270,180,3000);
        double flmax = 0, frmax = 0, blmax = 0, brmax = 0, v;

        while(fr.getCurrentPosition()<travelTics){

            flmax = (v = fl.getVelocity()) > flmax ? v : flmax;
            frmax = (v = fr.getVelocity()) > frmax ? v : frmax;
            blmax = (v = bl.getVelocity()) > blmax ? v : blmax;
            brmax = (v = br.getVelocity()) > brmax ? v : brmax;
            teamUtil.log("Looping FL:" + flmax + " FR:" + frmax + " BL:" + blmax + " BR:" + brmax);


        }
        stopMotors();
        long elapsedTime = System.currentTimeMillis()-startTime;
        double ticsTraveled = fr.getCurrentPosition()-ticStartPosition;
        double cmsTraveled = ticsTraveled/COUNTS_PER_CENTIMETER;
        double timeS = elapsedTime/1000.0;
        teamUtil.log("Elapsed Time: " + elapsedTime);
        teamUtil.log("Tics Traveled: " + ticsTraveled);

        teamUtil.log("Cms Per Second: " + cmsTraveled/timeS);
        teamUtil.log("Cms Traveled: " + cmsTraveled);

        teamUtil.log("Tics Per Second: " + ticsTraveled/(elapsedTime/1000));

        teamUtil.log("Strafing Max Velocities FL:" + flmax + " FR:" + frmax + " BL:" + blmax + " BR:" + brmax);
    }
}

