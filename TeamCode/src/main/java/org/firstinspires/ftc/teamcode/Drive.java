package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="DRIVE (and win hopefully)", group="Training")
//@Disabled
public class Drive extends OpMode {


    Init robot = new Init();

    private int outPos;
    private int armRotatorPos;
    private boolean toggle;
    private double intakePos;
    private double intakeGrasper;
    private double intakeSwivel;
    private double intakeRotate;

    private double outtakeGrasper;
    private double outtakeWrist;
    private double outtakeSwivel;

    private double unSheath;
    private double unSheath2;
    private boolean turnedOff;
    private boolean toggle2 = true;

    private double DRIVE_SPEED = 1.0;

    @Override /* * */
    public void init() {

        robot.init(hardwareMap);

        telemetry.addLine("INIT????????, Yeah");

        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outPos = robot.outtake1.getCurrentPosition();


        robot.outtakeRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outtakeRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotatorPos = robot.outtakeRotator.getCurrentPosition();

        intakePos = robot.intake1.getPosition();
        intakeGrasper = robot.intakeGrasper.getPosition();
        intakeSwivel = robot.intakeSwivel.getPosition();
        intakeRotate = robot.intakeRotator.getPosition();

        outtakeGrasper = robot.outtakeGrasper.getPosition();
        outtakeWrist = robot.outtakeSwivelLower.getPosition();
        outtakeSwivel = robot.outtakeSwivel.getPosition();

        unSheath = robot.unSheath.getPosition();
        unSheath2 = robot.unSheath2.getPosition();
    }

    @Override
    public void loop() {
        driveWToggle();
        intakeSystem();
        outtakeSystem();

        sheathManagment();

        controller1Telemetry();
        controller2Telemetry();


        //THREAD I, ARE YOU HAVING FUN? (EMPTY)
        new Thread(() -> {
            try {
                autoHang();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }).start();

        //THREAD II, ELECTRIC BOOGALOO (EMPTY)
        new Thread(() -> {
            try {
                autoPickUp();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }).start();

        //THREAD III, YIPPEE (READY FOR TESTING)
        new Thread(() -> {
            try {
                autoTransfer();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }).start();
    }

    public void controller1Telemetry() {
        telemetry.addLine("CONTROLLER NUMBER 1");
        telemetry.addLine("");
        telemetry.addLine("left and right sticks to drive");
        telemetry.addLine("LeftFront power: " + robot.leftFront.getPower());
        telemetry.addLine("LeftRear power: " + robot.leftRear.getPower());
        telemetry.addLine("RightFront power: " + robot.rightFront.getPower());
        telemetry.addLine("RightRear power: " + robot.rightRear.getPower());
        telemetry.addLine("");
        telemetry.addLine("Drive Speed: " + DRIVE_SPEED);
        telemetry.addLine("");
        telemetry.addLine("outtake1: " + robot.outtake1.getTargetPosition());
        telemetry.addLine("outtake2: " + robot.outtake2.getTargetPosition());
        telemetry.addLine("");
        telemetry.addLine("Outtake1 Power: " + robot.outtake1.getPower());
        telemetry.addLine("Outtake2 Power " + robot.outtake2.getPower());
        telemetry.addLine("");
        telemetry.addLine("Outtake Grasper position: " + outtakeGrasper);
        telemetry.addLine("Outtake Wrist position: " + outtakeWrist);
        telemetry.addLine("Outtake Swivel position: " + outtakeSwivel);
        telemetry.addLine("");
        telemetry.addLine("OuttakeRotator Power: " + robot.outtakeRotator.getPower());
        telemetry.addLine("OuttakeRotator position: " + robot.outtakeRotator.getCurrentPosition());
        telemetry.addLine("");
    }

    public void controller2Telemetry() {
        telemetry.addLine("");
        telemetry.addLine("CONTROLLER NUMBER 2");
        telemetry.addLine("");
        telemetry.addLine("Right stick Y controls intake slide");
        telemetry.addLine("Intake Slide position: " + robot.intake1.getPosition());
        telemetry.addLine("");
        telemetry.addLine("Left and Right Triggers controlled grasper");
        telemetry.addLine("Intake Grasper position: " + intakeGrasper);
        telemetry.addLine("");
        telemetry.addLine("Left stick Y controls swivel");
        telemetry.addLine("Intake Swivel position: " + intakeSwivel);
        telemetry.addLine("");
        telemetry.addLine("bumpers control rotator");
        telemetry.addLine("Intake Rotator position: " + intakeRotate);

    }

    public void sheathManagment()
    {
        if (!gamepad1.y && !turnedOff)
        {
            robot.unSheath.setPosition(UNSHEATH_SHEATH);
            robot.unSheath2.setPosition(UNSHEATH2_SHEATH);
        }

        if (gamepad1.y || turnedOff)
        {
            robot.unSheath.setPosition(UNSHEATH_UNSHEATH);
            robot.unSheath2.setPosition(UNSHEATH2_UNSHEATH);
            turnedOff = true;
        }
    }


    public void autoHang() throws InterruptedException {
        Thread.sleep(10);
    }

    public void autoPickUp() throws InterruptedException {
        Thread.sleep(10);
    }

    public void autoTransfer() throws InterruptedException {

        if (gamepad2.a)
        {
            outPos = OUTTAKE_ARM_BOTTOM;
            intakeGrasper = INTAKE_GRASPER_CLOSE;
            intakeRotate = INTAKE_ROTATOR_STRAIGHT;
            intakeSwivel = INTAKE_SWIVEL_TRANSFER;
            outtakeGrasper = OUTTAKE_GRASPER_OPEN;
            outtakeWrist = OUTTAKE_LOWER_SWIVEL_TRANSFER;

            Thread.sleep(100);

            intakePos = INTAKE_IN;

            Thread.sleep(800);

            outtakeSwivel = OUTTAKE_SWIVEL_TRANSFER;

            Thread.sleep(400);

            outtakeGrasper = OUTTAKE_GRASPER_CLOSE;

            Thread.sleep(400);

            intakeGrasper = INTAKE_GRASPER_OPEN;

            Thread.sleep(100);

            outtakeWrist = OUTTAKE_LOWER_SWIVEL_OUTTAKE_BUCKET;
            outtakeSwivel = OUTTAKE_SWIVEL_OUTTAKE_BUCKET;
            outPos = OUTTAKE_ARM_BUCKET;
        }
    }

    public void intakeSystem()
    {

        //intake slides
        if (gamepad2.right_stick_y > 0.4)
        {
            intakePos = Math.min(intakePos + 0.008, INTAKE_IN);
        }

        if (gamepad2.right_stick_y < -0.4)
        {
            intakePos = Math.max(intakePos - 0.008, INTAKE_OUT);
        }

        robot.intake1.setPosition(intakePos);


        //grasper control
        if (gamepad2.right_trigger > 0.4)
        {
            intakeGrasper = INTAKE_GRASPER_CLOSE;
        }

        if (gamepad2.left_trigger > 0.4)
        {
            intakeGrasper = INTAKE_GRASPER_OPEN;
        }

        robot.intakeGrasper.setPosition(intakeGrasper);


        //swivel control
        if (gamepad2.left_stick_y < -0.4)
        {
            intakeSwivel = Math.min(intakeSwivel + 0.008, INTAKE_SWIVEL_TRANSFER);
        }

        if (gamepad2.left_stick_y > 0.4)
        {
            intakeSwivel = Math.max(intakeSwivel - 0.008, INTAKE_SWIVEL_DOWN);
        }

        robot.intakeSwivel.setPosition(intakeSwivel);

        //intake rotator
        if (gamepad2.right_bumper)
        {
            intakeRotate = Math.min(intakeRotate + 0.008, INTAKE_ROTATOR_LEFT);
        }

        if (gamepad2.left_bumper)
        {
            intakeRotate = Math.max(intakeRotate - 0.008, INTAKE_ROTATOR_RIGHT);
        }

        robot.intakeRotator.setPosition(intakeRotate);

        //HOTKEYS
        if (gamepad2.x)
        {
            //pre-req
            intakePos = INTAKE_OUT;
            intakeRotate = INTAKE_ROTATOR_STRAIGHT;
            intakeSwivel = INTAKE_SWIVEL_SCAN;
            intakeGrasper = INTAKE_GRASPER_OPEN;

            //this will be replaced hopefully by autoPickUp later
        }

    }

    public void outtakeSystem()
    {
        //ARMS
        if (gamepad1.dpad_up)
        {
            outPos += OUTTAKE_ARM_SPEED;
        }
        if (gamepad1.dpad_down)
        {
            outPos -= OUTTAKE_ARM_SPEED;
        }

        if (outPos > OUTTAKE_ARM_BUCKET)
        {
            outPos = OUTTAKE_ARM_BUCKET;
        }

        //ARM ROTATOR
        if (gamepad1.x)
        {
            //armRotatorPos = Math.min(armRotatorPos * OUTTAKE_ROTATOR_SPEED, OUTTAKE_ROTATOR_BACK);
            armRotatorPos += OUTTAKE_ROTATOR_SPEED;
        }
        if (gamepad1.b)
        {
            //armRotatorPos = Math.max(armRotatorPos - OUTTAKE_ROTATOR_SPEED, OUTTAKE_ROTATOR_FORWARD);
            armRotatorPos -= OUTTAKE_ROTATOR_SPEED;
        }


        //grasper control
        if (gamepad1.right_trigger > 0.4)
        {
            outtakeGrasper = INTAKE_GRASPER_CLOSE;
        }

        if (gamepad1.left_trigger > 0.4)
        {
            outtakeGrasper = INTAKE_GRASPER_OPEN;
        }

        robot.outtakeGrasper.setPosition(outtakeGrasper);

        if (robot.vertArms.isPressed() && toggle2) {
            robot.outtake1.setPower(0);
            robot.outtake2.setPower(0);
            robot.outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outPos = 0;
            toggle2 = false;
        }

        if(!robot.vertArms.isPressed() || gamepad1.dpad_up)
        {
            robot.outtake1.setTargetPosition(outPos);
            robot.outtake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.outtake1.setPower(OUTTAKE_ARM_POWER);

            robot.outtake2.setTargetPosition(outPos);
            robot.outtake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.outtake2.setPower(OUTTAKE_ARM_POWER);
        }

        if (outPos > 50)
        {
            toggle2 = true;
        }

        if (outPos < -10)
        {
            outPos = -10;
        }

        if (turnedOff)
        {
            robot.outtakeRotator.setTargetPosition(armRotatorPos);
            robot.outtakeRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.outtakeRotator.setPower(OUTTAKE_ROTATOR_POWER);
        }
        else {
            robot.outtakeRotator.setPower(0.0);
        }

        robot.outtakeSwivel.setPosition(outtakeSwivel);
        robot.outtakeSwivelLower.setPosition(outtakeWrist);
    }


    public void driveWToggle()
    {
        double leftX;
        double leftY;
        double rightX;

        if (DRIVE_SPEED == HIGH_DRIVE_SPEED && gamepad1.a && !toggle)
        {
            DRIVE_SPEED = LOW_DRIVE_SPEED;
            toggle = true;
        }
        if (DRIVE_SPEED == LOW_DRIVE_SPEED && gamepad1.a && !toggle)
        {
            DRIVE_SPEED = HIGH_DRIVE_SPEED;
            toggle = true;
        }

        if (!gamepad1.a)
        {
            toggle = false;
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
}
