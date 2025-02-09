package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Constants.*;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="My beatiful, precious, Hunter Nguyen bless us in the auto that shall transpire.", group="Autonomous")
public class BucketAuto extends LinearOpMode {

    public class Outtake {

        private DcMotorEx outTake1;
        private DcMotorEx outTake2;

        public Outtake(HardwareMap hardwareMap) {
            outTake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
            outTake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outTake1.setDirection(DcMotorSimple.Direction.FORWARD);

            outTake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
            outTake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outTake2.setDirection(DcMotorSimple.Direction.REVERSE);

        }

        public class OuttakeToPos implements Action {
            private final int targetPosition;
            private final double holdingPower;
            private final double direction;
            private boolean initialized = false;

            public OuttakeToPos(int targetPosition, double holdingPower, double direction) {
                this.targetPosition = targetPosition;
                this.holdingPower = holdingPower;
                this.direction = direction;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized)
                {

                    initialized = true;
                }

                outTake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                outTake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                outTake1.setPower(direction);
                outTake2.setPower(direction);

                packet.put("outtake1Pos", outTake1.getCurrentPosition());
                packet.put("outtake2Pos", outTake2.getCurrentPosition());


                int pos = outTake1.getCurrentPosition();
                packet.put("liftPos", pos);
                if (Math.abs(targetPosition - pos) > 20) {
                    return true;
                }
                else
                {
                    outTake1.setTargetPosition(pos);
                    outTake2.setTargetPosition(pos);
                    outTake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outTake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outTake1.setPower(holdingPower);  // Apply a small power to hold the position
                    outTake2.setPower(0.0);
                    return false; // Stop running, but maintain hold
                }
            }

            // Method to create actions with holding power
        }

        public Action bottom() {
            return new OuttakeToPos(OUTTAKE_ARM_BOTTOM, 0.2, -1.0);
        }

        public Action bucket() {
            return new OuttakeToPos(OUTTAKE_ARM_BUCKET, 0.2, 1.0);
        }

    }

    //rotator
    public class IntakeRotator
    {
        private Servo rotator;

        public IntakeRotator(HardwareMap hardwareMap) {
            rotator = hardwareMap.get(Servo.class, "inRotator");
        }

        public class RotateStraight implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(INTAKE_ROTATOR_STRAIGHT);
                return false;
            }
        }

        public class RotateLeft implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(INTAKE_ROTATOR_LEFT);
                return false;
            }
        }

        public class RotateRight implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(INTAKE_ROTATOR_RIGHT);
                return false;
            }
        }

        public class RotateGrab implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(INTAKE_ROTATOR_GRAB);
                return false;
            }
        }

        public class RotateRights implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(INTAKE_BLAH_BLAH_BOO_I_SCARED_YOU);
                return false;
            }
        }

        public Action straight() {
            return new RotateStraight();
        }

        public Action left() {
            return new RotateLeft();
        }

        public Action right() {
            return new RotateRight();
        }

        public Action grab() {
            return new RotateGrab();
        }

        public Action rot() {return new RotateRights();}
    }



    public class IntakeSwivel
    {
        private Servo inSwivel;

        public IntakeSwivel(HardwareMap hardwareMap) {
            inSwivel = hardwareMap.get(Servo.class, "inSwivel");
        }

        public class IntakeTransfer implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inSwivel.setPosition(INTAKE_SWIVEL_TRANSFER);
                return false;
            }
        }

        public class IntakeDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inSwivel.setPosition(INTAKE_SWIVEL_DOWN);
                return false;
            }
        }

        public class IntakeScan implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inSwivel.setPosition(INTAKE_SWIVEL_SCAN);
                return false;
            }
        }

        public Action down() { return new IntakeDown(); }

        public Action scan() {
            return new IntakeScan();
        }

        public Action transfer() {
            return new IntakeTransfer();
        }
    }


    public class IntakeGrasper
    {
        private Servo inGrasper;

        public IntakeGrasper(HardwareMap hardwareMap) {
            inGrasper = hardwareMap.get(Servo.class, "inGrasper");
        }

        public class IntakeClose implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inGrasper.setPosition(INTAKE_GRASPER_CLOSE);
                return false;
            }
        }

        public class IntakeOpen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inGrasper.setPosition(INTAKE_GRASPER_OPEN);
                return false;
            }
        }

        public Action open() { return new IntakeOpen(); }

        public Action close() {
            return new IntakeClose();
        }
    }


    public class IntakeSlider
    {
        private Servo inSlider;
        private double intakePos;

        public IntakeSlider(HardwareMap hardwareMap) {
            inSlider = hardwareMap.get(Servo.class, "intake1");
            intakePos = inSlider.getPosition();

        }

        public class IntakeIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Gradually move the servo towards INTAKE_IN, if not already there
                if (intakePos < INTAKE_IN) {
                    intakePos = Math.min(intakePos + 0.2, INTAKE_IN); // Gradually move in
                    inSlider.setPosition(intakePos); // Set the new position

                    return true;
                }
                return false;
            }
        }

        // Action to move the intake slider out
        public class IntakeOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Gradually move the servo towards INTAKE_OUT, if not already there
                if (intakePos > INTAKE_OUT) {
                    intakePos = Math.max(intakePos - 0.2, INTAKE_OUT); // Gradually move out
                    inSlider.setPosition(intakePos); // Set the new position
                    return true;
                }
                return false;
            }
        }

        public class IntakeTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Gradually move the servo towards INTAKE_OUT, if not already there
                if (intakePos < INTAKE_WEE_BIT_OUT) {
                    intakePos = Math.max(intakePos + 0.2, INTAKE_WEE_BIT_OUT); // Gradually move out
                    inSlider.setPosition(intakePos); // Set the new position
                    return true;
                }
                return false;
            }
        }


        public Action in() { return new IntakeIn(); }

        public Action out() {
            return new IntakeOut();
        }

        public Action tadOut() {
            return new IntakeTransfer();
        }
    }


    public class OuttakeGrasper
    {
        private Servo outGrasper;

        public OuttakeGrasper(HardwareMap hardwareMap) {
            outGrasper = hardwareMap.get(Servo.class, "outGrasper");
        }

        public class OuttakeOpen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outGrasper.setPosition(OUTTAKE_GRASPER_OPEN);
                return false;
            }
        }

        public class OuttakeClose implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outGrasper.setPosition(OUTTAKE_GRASPER_CLOSE);
                return false;
            }
        }

        public Action close() {
            return new OuttakeClose();
        }

        public Action open() {
            return new OuttakeOpen();
        }

    }


    public class OuttakeLowerSwivel
    {
        private Servo outSwivelLower;

        public OuttakeLowerSwivel(HardwareMap hardwareMap) {
            outSwivelLower = hardwareMap.get(Servo.class, "outSwivelLow");
        }

        public class OuttakeLowerSwivelIntake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outSwivelLower.setPosition(OUTTAKE_LOWER_SWIVEL_INTAKE);
                return false;
            }
        }

        public class OuttakeLowerSwivelTransfer implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outSwivelLower.setPosition(OUTTAKE_LOWER_SWIVEL_TRANSFER);
                return false;
            }
        }

        public class OuttakeLowerSwivelBucket implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outSwivelLower.setPosition(OUTTAKE_LOWER_SWIVEL_OUTTAKE_BUCKET);
                return false;
            }
        }

        public class OuttakeLowerSwivelSpecimen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outSwivelLower.setPosition(OUTTAKE_LOWER_SWIVEL_OUTTAKE_SPECIMEN);
                return false;
            }
        }

        public class OuttakeLowerSwivelSpecimenEnd implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outSwivelLower.setPosition(OUTTAKE_LOWER_SWIVEL_OUTTAKE_SPECIMEN_END);
                return false;
            }
        }


        public Action intake() { return new OuttakeLowerSwivelIntake(); }

        public Action specimen() {
            return new OuttakeLowerSwivelSpecimen();
        }

        public Action bucket() {
            return new OuttakeLowerSwivelBucket();
        }

        public Action transfer() {
            return new OuttakeLowerSwivelTransfer();
        }

        public Action specimenEnd()
        {
            return new OuttakeLowerSwivelSpecimenEnd();
        }
    }


    public class OuttakeSwivel
    {
        private Servo outSwivel;

        public OuttakeSwivel(HardwareMap hardwareMap) {
            outSwivel = hardwareMap.get(Servo.class, "outSwivel");
        }

        public class OuttakeSwivelIntake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outSwivel.setPosition(OUTTAKE_SWIVEL_INTAKE);
                return false;
            }
        }

        public class OuttakeSwivelTransfer implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outSwivel.setPosition(OUTTAKE_SWIVEL_TRANSFER);
                return false;
            }
        }

        public class OuttakeSwivelBucket implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outSwivel.setPosition(OUTTAKE_SWIVEL_OUTTAKE_BUCKET);
                return false;
            }
        }

        public class OuttakeSwivelSpecimenStart implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outSwivel.setPosition(OUTTAKE_SWIVEL_OUTTAKE_SPECIMEN_START);
                return false;
            }
        }

        public class OuttakeSwivelSpecimenEnd implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outSwivel.setPosition(OUTTAKE_SWIVEL_OUTTAKE_SPECIMEN_END);
                return false;
            }
        }

        public class NOOOOOOOOOOOOOOOO implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outSwivel.setPosition(NOOOOOOOOOOOOO);
                return false;
            }
        }

        public class OUTTAKE_SWIVEL_INTAKE_2 implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outSwivel.setPosition(OUTTAKE_SWIVEL_INTAKE_2);
                return false;
            }
        }


        public Action bucket() { return new OuttakeSwivelBucket(); }

        public Action specimenStart() {
            return new OuttakeSwivelSpecimenStart();
        }

        public Action specimenEnd() {
            return new OuttakeSwivelSpecimenEnd();
        }

        public Action transfer() {
            return new OuttakeSwivelTransfer();
        }

        public Action intake() { return new OuttakeSwivelIntake(); }

        public Action NO() { return new NOOOOOOOOOOOOOOOO(); }

        public Action r2() { return new OUTTAKE_SWIVEL_INTAKE_2(); }

    }

    public class Sleep
    {
        public Sleep()
        {

        }

        public class p500 implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                try {
                    Thread.sleep(5000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }

        }

        public class p150 implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                try {
                    Thread.sleep(1500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }

        }

        public class p100 implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }

        }

        public class p050 implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }

        }

        public class p025 implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }

        }

        public class p015 implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                try {
                    Thread.sleep(150);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }

        }

        public class p075 implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                try {
                    Thread.sleep(75);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }

        }

        public class p0 implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                try {
                    Thread.sleep(0);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }

        }

        public Action oneSec()
        {
            return new p100();
        }

        public Action half()
        {
            return new p050();
        }

        public Action quarter()
        {
            return new p025();
        }

        public Action small()
        {
            return new p015();
        }

        public Action oneAndHalf() { return new p150();}

        public Action five() { return new p500();}

        public Action eighth() {return new p015();}

        public Action none() {return new p0();}
    }




    @Override
    public void runOpMode() {

        Pose2d pose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);

        Outtake outtake = new Outtake(hardwareMap);

        OuttakeGrasper outGrasper = new OuttakeGrasper(hardwareMap);
        OuttakeLowerSwivel outLowerSwivel = new OuttakeLowerSwivel(hardwareMap);
        OuttakeSwivel outSwivel = new OuttakeSwivel(hardwareMap);

        IntakeSlider inSlider = new IntakeSlider(hardwareMap);
        IntakeRotator inRotator = new IntakeRotator(hardwareMap);
        IntakeSwivel inSwivel = new IntakeSwivel(hardwareMap);
        IntakeGrasper inGrasper = new IntakeGrasper(hardwareMap);

        Sleep sleep = new Sleep();


        TrajectoryActionBuilder putInOne = drive.actionBuilder(pose)
                .strafeToLinearHeading(new Vector2d(-17.5, 4), Math.toRadians(45));

        TrajectoryActionBuilder pickUpOne = drive.actionBuilder(new Pose2d(-17.5, 4, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-13.5, 17), Math.toRadians(90));

        TrajectoryActionBuilder putInTwo = drive.actionBuilder(new Pose2d(-15.5, 13, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-17.5, 4), Math.toRadians(45));

        TrajectoryActionBuilder pickUpTwo = drive.actionBuilder(new Pose2d(-17.5, 4, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-22.5, 16), Math.toRadians(100));

        TrajectoryActionBuilder putInThree = drive.actionBuilder(new Pose2d(-22.5, 16, Math.toRadians(100)))
                .strafeToLinearHeading(new Vector2d(-17.5, 4), Math.toRadians(45));

        TrajectoryActionBuilder pickUpThree = drive.actionBuilder(new Pose2d(-17.5, 4, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-21.5, 18), Math.toRadians(130));

        TrajectoryActionBuilder putInFour = drive.actionBuilder(new Pose2d(-21.5, 18, Math.toRadians(130)))
                .strafeToLinearHeading(new Vector2d(-17.5, 4), Math.toRadians(45));

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(-17.5, 4, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(0, 55), Math.toRadians(180));

        TrajectoryActionBuilder parker = drive.actionBuilder(new Pose2d(0, 55, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(1, 55), Math.toRadians(60));

        TrajectoryActionBuilder parkers = drive.actionBuilder(new Pose2d(1, 55, Math.toRadians(60)))
                .strafeToLinearHeading(new Vector2d(2, 55), Math.toRadians(-60));

        TrajectoryActionBuilder boomShaqalaqacka = drive.actionBuilder(new Pose2d(2, 55, Math.toRadians(-60)))
                .strafeToLinearHeading(new Vector2d(3, 55), Math.toRadians(180));

        TrajectoryActionBuilder parkersss = drive.actionBuilder(new Pose2d(3, 55, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(25, 55), Math.toRadians(180));



        SequentialAction transfer = new SequentialAction(
                inGrasper.close(),
                new ParallelAction(
                        inRotator.straight(),
                        inSwivel.transfer(),
                        inSlider.in(),
                        outGrasper.open(),
                        outLowerSwivel.transfer()
                ),
                outSwivel.transfer(),
                outGrasper.close(),
                inGrasper.open(),
                new ParallelAction(
                        outSwivel.bucket(),
                        outLowerSwivel.bucket()
                )
        );

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                outGrasper.close(),
                                outSwivel.bucket(),
                                outLowerSwivel.transfer(),
                                outtake.bucket(),
                                putInOne.build(),
                                inSwivel.transfer()
                        ),
                        sleep.quarter(),
                        outLowerSwivel.bucket(),
                        sleep.half(),
                        outGrasper.open(),
                        sleep.small(),
                        outLowerSwivel.transfer(),
                        pickUpOne.build(),
                        new ParallelAction(
                                outtake.bottom(),
                                inSwivel.scan(),
                                inGrasper.open(),
                                inSlider.out()
                        ),
                        sleep.half(),
                        inSwivel.down(),
                        sleep.quarter(),
                        inGrasper.close(),
                        sleep.quarter(),
                        new ParallelAction(
                                inRotator.straight(),
                                inSwivel.transfer(),
                                inSlider.tadOut(),
                                outLowerSwivel.transfer(),
                                outtake.bottom(),
                                outGrasper.open()
                        ),
                        sleep.half(),
                        outSwivel.transfer(),
                        sleep.quarter(),
                        outGrasper.close(),
                        sleep.quarter(),
                        inGrasper.open(),
                        sleep.quarter(),
                        new ParallelAction(
                                outSwivel.bucket(),
                                outtake.bucket(),
                                putInTwo.build()
                        ),
                        sleep.half(),
                        outLowerSwivel.bucket(),
                        sleep.half(),
                        outGrasper.open(),
                        sleep.small(),
                        outLowerSwivel.transfer(),
                        new ParallelAction(
                                pickUpTwo.build(),
                                outtake.bottom(),
                                inSlider.out(),
                                inSwivel.scan(),
                                inGrasper.open()
                        ),
                        sleep.half(),
                        inSwivel.down(),
                        sleep.quarter(),
                        inGrasper.close(),
                        sleep.quarter(),
                        new ParallelAction(
                                inRotator.straight(),
                                inSwivel.transfer(),
                                inSlider.tadOut(),
                                outLowerSwivel.transfer(),
                                outtake.bottom(),
                                outGrasper.open()
                        ),
                        sleep.half(),
                        outSwivel.transfer(),
                        sleep.quarter(),
                        outGrasper.close(),
                        sleep.quarter(),
                        inGrasper.open(),
                        sleep.quarter(),
                        new ParallelAction(
                                outSwivel.bucket(),
                                outtake.bucket(),
                                putInThree.build()
                        ),
                        sleep.half(),
                        outLowerSwivel.bucket(),
                        sleep.half(),
                        outGrasper.open(),
                        sleep.small(),
                        outLowerSwivel.transfer(),
                        new ParallelAction(
                                pickUpThree.build(),
                                outtake.bottom(),
                                inSlider.out(),
                                inSwivel.scan(),
                                inRotator.rot(),
                                inGrasper.open()
                        ),
                        sleep.half(),
                        inSwivel.down(),
                        sleep.quarter(),
                        inGrasper.close(),
                        sleep.quarter(),
                        inSwivel.transfer(),
                        sleep.quarter(),
                        new ParallelAction(
                                inRotator.straight(),
                                inSwivel.transfer(),
                                inSlider.tadOut(),
                                outLowerSwivel.transfer(),
                                outtake.bottom(),
                                outGrasper.open()
                        ),
                        sleep.half(),
                        outSwivel.transfer(),
                        sleep.quarter(),
                        outGrasper.close(),
                        sleep.quarter(),
                        inGrasper.open(),
                        sleep.quarter(),
                        new ParallelAction(
                                outSwivel.bucket(),
                                outtake.bucket(),
                                putInFour.build()
                        ),
                        sleep.half(),
                        outLowerSwivel.bucket(),
                        sleep.half(),
                        outGrasper.open(),
                        sleep.small(),
                        outLowerSwivel.transfer(),
                        new ParallelAction(
                                park.build(),
                                outtake.bottom(),
                                inSwivel.scan(),
                                inGrasper.open(),
                                inSlider.in(),
                                inRotator.straight(),
                                inSwivel.transfer()
                        ),
                        parker.build(),
                        parkers.build(),
                        boomShaqalaqacka.build(),
                        parkersss.build(),
                        outSwivel.intake()
                )
        );


        while (opModeIsActive())
        {

        }



    }
}

