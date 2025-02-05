package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Init;


@TeleOp(name=":(", group="Training")
//@Disabled
public class MotorTuning extends OpMode {

    Init robot = new Init();

    public double OUT_ARM_SPEED = 0.0;
    public double IN_ARM_SPEED = 0.0;// sets rate to move servo

    public int rotatePos;


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

    public void outRotate(int amp)
    {
        robot.outtakeRotator.setPower(0.5);
        robot.outtakeRotator.setTargetPosition(amp);
        robot.outtakeRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outtakeRotator.setPower(0.5);
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
        rotatePos = robot.outtakeRotator.getCurrentPosition();

        inPos1 = robot.intake1.getPosition();
        outPos1 = robot.outtake1.getCurrentPosition();

        //armPos = 50; // Lifts Arm for Driving
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        if (gamepad2.left_stick_y > 0.4)
        {
            rotatePos += OUTTAKE_ROTATOR_SPEED;
        }
        if (gamepad2.left_stick_y < -0.4)
        {
            rotatePos -= OUTTAKE_ROTATOR_SPEED;
        }

        outRotate(rotatePos);

        //robot.intake2.setPosition(rotatePos7);


        //outtake sliders
        if (gamepad2.right_stick_y > 0.4)
        {
            OUT_ARM_SPEED = 1.0;
            outPos1 += (int) (gamepad2.right_stick_y * 15.0);
        }

        if (gamepad2.right_stick_y < -0.4)
        {
            OUT_ARM_SPEED = 1.0;
            outPos1 -= (int) (Math.abs(gamepad2.right_stick_y) * 15.0);
        }

        outTake(outPos1);


        telemetry.addLine("Arm Tuning (controller 2");
        telemetry.addLine("");
        telemetry.addLine("outtake slides (right_stick_y) " + outPos1);
        telemetry.addLine("");
        telemetry.addLine("");

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
