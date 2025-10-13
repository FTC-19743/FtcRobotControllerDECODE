package org.firstinspires.ftc.teamcode.slappyCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.libs.teamUtil;


@TeleOp(name="CandyBot", group="Robot")
public class CandyBot extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftDrive;
    public DcMotor  rightDrive;
    public DcMotorEx arm;





    // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public void calbibrate(){
        boolean details=false;
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setPower(-.32);
        int lastExtenderPosition = arm.getCurrentPosition();
        teamUtil.pause(250);
        while (arm.getCurrentPosition() != lastExtenderPosition) {
            lastExtenderPosition = arm.getCurrentPosition();
            if (details) teamUtil.log("Calibrate Intake: Extender: " + arm.getCurrentPosition());
            teamUtil.pause(50);
        }
        arm.setPower(0);
        teamUtil.pause(500); // let it "relax" just a bit
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPositionTolerance(15);// make that our zero position
        //arm.setTargetPosition(EXTENDER_UNLOAD);
        //arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //arm.setVelocity(EXTENDER_HOLD_RETRACT_VELOCITY);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        teamUtil.log("Calibrate arm Final: arm: "+arm.getCurrentPosition());
    }
    @Override
    public void runOpMode() {
        teamUtil.init(this);
        double left;
        double right;
        double drive;
        double turn;
        double max;

        // Define and Initialize Motors
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        arm = hardwareMap.get(DcMotorEx.class, "left_arm");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "CandyBot Ready.  Press START.");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        calbibrate();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            leftDrive.setPower(left);
            rightDrive.setPower(right);

            // Use gamepad left & right Bumpers to open and close the claw

            // Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad1.right_trigger>0.5){
                arm.setTargetPosition(190);
                arm.setVelocity(300);
                //while(arm.getCurrentPosition())
            }
            if (gamepad1.left_trigger>0.5){
                arm.setTargetPosition(30);
                arm.setVelocity(300);
                //while(arm.getCurrentPosition())
            }


            // Send telemetry message to signify robot running;
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addLine("arm " + arm.getCurrentPosition());
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
