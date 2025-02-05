package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Init;


@TeleOp(name="Tune her, Hunter", group="Training")
//@Disabled
public class Tuning extends OpMode {

    Init robot = new Init();

    public double OUT_ARM_SPEED = 0.0;
    public double IN_ARM_SPEED = 0.0;// sets rate to move servo

    public double rotatePos;
    public double rotatePos1;
    public double rotatePos2;
    public double rotatePos3;
    public double rotatePos4;
    public double rotatePos5;
    public double rotatePos6;
    public double rotatePos7;
    public double rotatePos8;
    public double rotatePos9;


    public double inPos1;
    public int outPos1;




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");

        //

        // Set to Run without Encoder for Tele Operated


    }


    public void outTake(int amp)
    {
        robot.outtake1.setPower(1.0);
        robot.outtake1.setTargetPosition(amp);
        robot.outtake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outtake1.setPower(OUT_ARM_SPEED);

        robot.outtake2.setPower(1.0);
        robot.outtake2.setTargetPosition(amp);
        robot.outtake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outtake2.setPower(OUT_ARM_SPEED);
    }

    public void inTake(double amp)
    {
        //armMover Action
        robot.intake1.setPosition(1.0 - amp);
        robot.intake2.setPosition(amp);
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        rotatePos = robot.outtakeSwivel.getPosition();
        rotatePos1 = robot.outtakeSwivelLower.getPosition();
        rotatePos2 = robot.outtakeGrasper.getPosition();

        rotatePos3 = robot.intakeGrasper.getPosition();
        rotatePos4 = robot.intakeRotator.getPosition();
        rotatePos5 = robot.intakeSwivel.getPosition();

        rotatePos6 = robot.intake1.getPosition();
        rotatePos7 = robot.intake2.getPosition();

        inPos1 = robot.intake1.getPosition();
        outPos1 = robot.outtake1.getCurrentPosition();

        //armPos = 50; // Lifts Arm for Driving
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        if (gamepad2.left_bumper)
        {
            rotatePos = Math.min(rotatePos + 0.01, 1.0);
        }
        if (gamepad2.right_bumper)
        {
            rotatePos = Math.max(rotatePos - 0.01, 0.0);
        }

        robot.outtakeSwivel.setPosition(rotatePos);


        if (gamepad2.y)
        {
            rotatePos1 = Math.min(rotatePos1 + 0.01, 1.0);
        }
        if (gamepad2.a)
        {
            rotatePos1 = Math.max(rotatePos1 - 0.01, 0.0);
        }

        robot.outtakeSwivelLower.setPosition(rotatePos1);

        if (gamepad2.b)
        {
            rotatePos2 = Math.min(rotatePos2 + 0.01, 1.0);
        }
        if (gamepad2.x)
        {
            rotatePos2 = Math.max(rotatePos2 - 0.01, 0.0);
        }

        robot.outtakeGrasper.setPosition(rotatePos2);

        if (gamepad1.dpad_up)
        {
            rotatePos3 = Math.min(rotatePos3 + 0.01, 1.0);
        }
        if (gamepad1.dpad_down)
        {
            rotatePos3 = Math.max(rotatePos3 - 0.01, 0.0);
        }

        robot.intakeGrasper.setPosition(rotatePos3);

        if (gamepad1.a)
        {
            rotatePos4 = Math.min(rotatePos4 + 0.01, 1.0);
        }
        if (gamepad1.y)
        {
            rotatePos4 = Math.max(rotatePos4 - 0.01, 0.0);
        }

        robot.intakeRotator.setPosition(rotatePos4);

        if (gamepad1.right_bumper)
        {
            rotatePos5 = Math.min(rotatePos5 + 0.01, 1.0);
        }
        if (gamepad1.left_bumper)
        {
            rotatePos5 = Math.max(rotatePos5 - 0.01, 0.0);
        }

        robot.intakeSwivel.setPosition(rotatePos5);

        if (gamepad1.right_trigger > 0.4)
        {
            rotatePos6 = Math.min(rotatePos6 + 0.01, 1.0);
        }
        if (gamepad1.left_trigger > 0.4)
        {
            rotatePos6 = Math.max(rotatePos6 - 0.01, 0.0);
        }

        robot.intake1.setPosition(rotatePos6);

        if (gamepad2.right_trigger > 0.4)
        {
            rotatePos7 = Math.min(rotatePos7 + 0.01, 1.0);
        }
        if (gamepad2.left_trigger > 0.4)
        {
            rotatePos7 = Math.max(rotatePos7 - 0.01, 0.0);
        }

        if (gamepad1.dpad_left)
        {
            rotatePos8 = Math.min(rotatePos8 + 0.01, 1.0);
        }
        if (gamepad1.dpad_right)
        {
            rotatePos8 = Math.max(rotatePos8 - 0.01, 0.0);
        }

        robot.unSheath.setPosition(rotatePos8);

        if (gamepad1.left_stick_y > 0.4)
        {
            rotatePos9 = Math.min(rotatePos9 + 0.005, 1.0);
        }
        if (gamepad1.left_stick_y < -0.4)
        {
            rotatePos9 = Math.max(rotatePos9 - 0.005, 0.0);
        }

        robot.unSheath2.setPosition(rotatePos9);

        //robot.intake2.setPosition(rotatePos7);


//        if (gamepad2.left_stick_y > 0.4)
//        {
//            inPos1 += Math.min(inPos1 + 0.01, 1.0);
//            inTake(inPos1);
//        }
//
//        if (gamepad2.left_stick_y < -0.4)
//        {
//            inPos1 -= Math.max(inPos1 + 0.01, 0.0);
//            inTake(inPos1);
//        }

        //auto intake
//        if (gamepad2.right_stick_y > 0.4)
//        {
//            OUT_ARM_SPEED = 1.0;
//            outPos1 += (int) (gamepad2.right_stick_y * 15.0);
//        }
//
//        if (gamepad2.right_stick_y < -0.4)
//        {
//            OUT_ARM_SPEED = 1.0;
//            outPos1 -= (int) (Math.abs(gamepad2.right_stick_y) * 15.0);
//        }
//
//        outTake(outPos1);


        telemetry.addLine("Servo Tuning (controller 2");
        telemetry.addLine("");
        telemetry.addLine("outtakeSwivel (left and right bumper) " + rotatePos + "  " + robot.outtakeSwivel.getPosition());
        telemetry.addLine("outtakeSwivelLower (y and a) " + rotatePos1 + "  " + robot.outtakeSwivelLower.getPosition());
        telemetry.addLine("outtakeGrasper (b and x) " + rotatePos2 + "  " + robot.outtakeGrasper.getPosition());
        telemetry.addLine("intake2 (left and right triggers) " + rotatePos7 + "  " + robot.intake2.getPosition());
        telemetry.addLine("");
        telemetry.addLine("");
        telemetry.addLine("Arm Tuning (controller 2");
        telemetry.addLine("");
        telemetry.addLine("outtake (dpad_up and dpad_down) " + outPos1);
        //telemetry.addLine("intake (LeftStickY) " + inPos1);
        telemetry.addLine("");
        telemetry.addLine("");
        telemetry.addLine("Servo Tuning (controller 1");
        telemetry.addLine("");
        telemetry.addLine("intake1 (left and right triggers) " + rotatePos6 + "  " + robot.intake1.getPosition());
        telemetry.addLine("intakeGrasper (dpad_up and dpad_down) " + rotatePos3 + "  " + robot.intakeGrasper.getPosition());
        telemetry.addLine("intakeRotator (a and y) " + rotatePos4 + "  " + robot.intakeRotator.getPosition());
        telemetry.addLine("intakeSwivel (lb and rb) " + rotatePos5 + "  " + robot.intakeSwivel.getPosition());
        telemetry.addLine("unSheath (dpad_left and dpad_right) " + rotatePos8 + "  " + robot.unSheath.getPosition());


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
