package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Main", group="Training")
//@Disabled
public class DriveBasic extends OpMode {

    Init robot   = new Init();

    private int outtakePos = 0;
    private int outtakeRotatorPos = 0;
    private double intakePos = 0.0;
    private boolean rand1 = false;

    @Override /* * */
    public void init() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");    //

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakePos = robot.outtake1.getCurrentPosition();

        robot.outtakeRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outtakeRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeRotatorPos = robot.outtakeRotator.getCurrentPosition();

    }

    @Override /* * */
    public void init_loop()
    {

    }


    @Override /* * */
    public void start()
    {

    }

    public boolean dpadLeftPressed = false;
    public boolean isControlsActive = false;

    @Override
    public void loop() {

        //controls();
        drive();
        grasper();
        arms();
        telemetry();
    }
    public void controls()
    {
        // Gamepad 1 Controls
        telemetry.addLine("GAMEPAD 1 CONTROLS:");
        telemetry.addLine("LEFT STICK (x, y): Controls movement");
        telemetry.addLine(" - LEFT STICK X: Strafing");
        telemetry.addLine(" - LEFT STICK Y: Forward/backward movement");
        telemetry.addLine("RIGHT STICK (x): Controls turning");
        telemetry.addLine("DPAD_UP: Raises kickstands (left and right)");
        telemetry.addLine("DPAD_LEFT: toggles the control menu");
        telemetry.addLine("DPAD_DOWN: Lowers kickstands (left and right)");
        telemetry.addLine("RIGHT_TRIGGER: Drops the outtake mechanism");
        telemetry.addLine("X: Also drops the outtake mechanism");
        telemetry.addLine("A: Toggles drive speed between full (1.0) and reduced (0.4)");

        telemetry.addLine("");

        // Gamepad 2 Controls
        telemetry.addLine("GAMEPAD 2 CONTROLS:");
        telemetry.addLine("LEFT STICK (y): Controls outtake arm movement");
        telemetry.addLine(" - Positive for up, negative for down");
        telemetry.addLine("RIGHT STICK (y): Controls intake arm movement");
        telemetry.addLine(" - Positive for out, negative for in");
        telemetry.addLine("LEFT_TRIGGER: Opens the grasper");
        telemetry.addLine("RIGHT_TRIGGER: Closes the grasper");
        telemetry.addLine("DPAD_UP: Initiates intake routine");
        telemetry.addLine("A: Sets intake arm to maximum position");
        telemetry.addLine("B: Sets intake arm to minimum position");
        telemetry.addLine("X: Sets outtake arm to maximum position");
        telemetry.addLine("Y: Sets outtake arm to minimum position and moves outtake to intake position");
        telemetry.addLine("LEFT_BUMPER: Rotates the grasper to the left");
        telemetry.addLine("RIGHT_BUMPER: Rotates the grasper to the right");

    }

    public void telemetry()
    {
        telemetry.addLine("ADD INFORMATION HERE");
    }

    public void inTake()
    {

        if (gamepad2.left_stick_y > 0.4)
        {
            intakePos += 0.01;
        }
        if (gamepad2.left_stick_y < -0.4)
        {
            intakePos -= 0.01;
        }

        robot.intake1.setPosition(1.0 - intakePos);
        robot.intake2.setPosition(intakePos);
    }

    public void outTake() {

        if (gamepad2.a)
        {
            outtakePos = 0;
        }

        if (gamepad2.y)
        {
            outtakePos = 1000;
        }

        if (gamepad2.right_stick_y > 0.4)
        {
            OUTTAKE_ARM_SPEED = 1.0;
            outtakePos += (int) (gamepad2.right_stick_y * 15.0);
        }

        if (gamepad2.right_stick_y < -0.4)
        {
            OUTTAKE_ARM_SPEED = 1.0;
            outtakePos -= (int) (Math.abs(gamepad2.right_stick_y) * 15.0);
        }

        robot.outtake1.setPower(OUTTAKE_ARM_SPEED);
        robot.outtake1.setTargetPosition(outtakePos);
        robot.outtake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outtake1.setPower(OUTTAKE_ARM_SPEED);

        robot.outtake2.setPower(OUTTAKE_ARM_SPEED);
        robot.outtake2.setTargetPosition(outtakePos);
        robot.outtake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outtake2.setPower(OUTTAKE_ARM_SPEED);
    }


    public void arms() {
        outTake();
        inTake();
    }

    public void grasper()
    {

    }

    public void drive()
    {
        //drive
        double leftX;
        double leftY;
        double rightX;

        if (DRIVE_SPEED == 1.0 && gamepad1.a && !rand1)
        {
            DRIVE_SPEED = 0.4;
            rand1 = true;
        }
        if (DRIVE_SPEED == 0.4 && gamepad1.a && !rand1)
        {
            DRIVE_SPEED = 1.0;
            rand1 = true;
        }

        if (!gamepad1.a)
        {
            rand1 = false;
        }

        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftX = gamepad1.left_stick_x * DRIVE_SPEED;
        leftY = gamepad1.left_stick_y * DRIVE_SPEED;
        rightX = gamepad1.right_stick_x * DRIVE_SPEED;

        double leftRearPower = leftY + leftX - rightX;
        double leftFrontPower = leftY - leftX - rightX;
        double rightRearPower = leftY - leftX + rightX;
        double rightFrontPower = leftY + leftX + rightX;

        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);

    }

    @Override
    public void stop()
    {
        //hello
    }

}
