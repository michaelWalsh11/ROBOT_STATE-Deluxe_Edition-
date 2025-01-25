package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Init {

    //Drive Motors
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;
    public DcMotor outtake1 = null;
    public DcMotor outtake2 = null;
    public DcMotor outtakeRotator = null;

    public Servo outtakeSwivel = null;
    public Servo outtakeSwivelLower = null;
    public Servo outtakeGrasper = null;

    public Servo intakeGrasper = null;
    public Servo intakeRotator = null;
    public Servo intakeSwivel = null;

    public Servo intake1 = null;
    public Servo intake2 = null;

    public IMU imu;
    public double offset = 0;//in degrees

    HardwareMap hwMap = null;

    //todo
    // - write down control hub orientation
    // - write motor stuff on phone
    // - see if we have servos and initialize them

    public void init(HardwareMap ahwMap)
    {
        hwMap = ahwMap;

        //MOTOR INIT
        leftFront = hwMap.dcMotor.get("leftfront");
        rightFront = hwMap.dcMotor.get("rightfront");
        leftRear = hwMap.dcMotor.get("leftrear");
        rightRear = hwMap.dcMotor.get("rightrear");
        outtake1 = hwMap.dcMotor.get("outtake1");
        outtake2 = hwMap.dcMotor.get("outtake2");
        outtakeRotator = hwMap.dcMotor.get("outtakeRotator");

        //MOTOR DIRECTION
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        outtake1.setDirection(DcMotor.Direction.FORWARD);
        outtake2.setDirection(DcMotor.Direction.FORWARD);
        outtakeRotator.setDirection(DcMotor.Direction.FORWARD);

        //DRIVE MOTOR POWER
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        //DRIVE MOTOR RUNMODE
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //SERVO INIT
        outtakeGrasper = hwMap.servo.get("outGrasper");
        outtakeSwivel = hwMap.servo.get("outSwivel");
        outtakeSwivelLower = hwMap.servo.get("outSwivelLow");

        intakeGrasper = hwMap.servo.get("inGrasper");
        intakeRotator = hwMap.servo.get("inRotator");
        intakeSwivel = hwMap.servo.get("inSwivel");

        intake1 = hwMap.servo.get("intake1");
        intake2 = hwMap.servo.get("intake2");

        //IMU
        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    //MIGHT NEED IDK THO
    public double getOrientation() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES) - offset;
    }

    public double resetImu() {
        offset = imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
        return offset;
    }
}
