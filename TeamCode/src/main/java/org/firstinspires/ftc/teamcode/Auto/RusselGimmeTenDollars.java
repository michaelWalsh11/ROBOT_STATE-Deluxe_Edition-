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

@Autonomous(name="Pay Up Marcus Walter (Dinero edition)", group="Autonomous")
public class RusselGimmeTenDollars extends LinearOpMode {

    public class Outtake {

        private DcMotorEx outTake1;
        private DcMotorEx outTake2;

        public Outtake(HardwareMap hardwareMap) {
            outTake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
            outTake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outTake1.setDirection(DcMotorSimple.Direction.REVERSE);

            outTake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
            outTake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outTake2.setDirection(DcMotorSimple.Direction.FORWARD);

        }

        public class OuttakeToPos implements Action {
            private final double targetPosition;
            private final double holdingPower;
            private boolean initialized = false;

            public OuttakeToPos(double targetPosition, double holdingPower) {
                this.targetPosition = targetPosition;
                this.holdingPower = holdingPower;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized)
                {
                    outTake1.setPower(0.8);
                    outTake2.setPower(0.8);
                    initialized = true;
                }

                double pos = outTake1.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < targetPosition) {
                    return true; // Continue running the action
                }
                else
                {
                    outTake1.setPower(holdingPower);  // Apply a small power to hold the position
                    outTake2.setPower(holdingPower);
                    return false; // Stop running, but maintain hold
                }
            }

            // Method to create actions with holding power
        }

        public Action bottom() {
            return new OuttakeToPos(OUTTAKE_ARM_BOTTOM, 0.2);
        }

        public Action bucket() {
            return new OuttakeToPos(OUTTAKE_ARM_BUCKET, 0.2);
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

        public Action in() { return new IntakeIn(); }

        public Action out() {
            return new IntakeOut();
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
    public void runOpMode() throws InterruptedException {


        //TODO
        // WHY IS THE DRIVING SO INCONSISTENT AND RANDOMLY VEERING
        //  -Tuning (tried and retuned it still sucks)
        //  -Floor = dirty (HUNTER DO IT)
        //  -Wheel problems? (HUNTER ANALYSIS MODE)
        // - trying to go too fast (SLOWED AND STILL TURNED)

        Pose2d pose = new Pose2d(0, 0, Math.toRadians(90));
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


        TrajectoryActionBuilder goToWallFirst = drive.actionBuilder(pose)
                .strafeToLinearHeading(new Vector2d(0, 33), Math.toRadians(90));

        TrajectoryActionBuilder backupALittle = drive.actionBuilder(new Pose2d(0, 33, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0, 27), Math.toRadians(90));

        TrajectoryActionBuilder GO = drive.actionBuilder(new Pose2d(0, 27, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(33, 21), Math.toRadians(45));

        TrajectoryActionBuilder Swing1 = drive.actionBuilder(new Pose2d(33, 21.5, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(39, 13), Math.toRadians(-90));

        TrajectoryActionBuilder SwingBack = drive.actionBuilder(new Pose2d(39, 13, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(40, 21), Math.toRadians(45));

        TrajectoryActionBuilder Swing2 = drive.actionBuilder(new Pose2d(40, 21.5, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(39, 13), Math.toRadians(-90));

        TrajectoryActionBuilder SwingBack2 = drive.actionBuilder(new Pose2d(39, 13, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(50.5, 20.5), Math.toRadians(45));

        TrajectoryActionBuilder Swing3 = drive.actionBuilder(new Pose2d(50.5,20.5, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(39, 13), Math.toRadians(-90));

        TrajectoryActionBuilder WallGrabSpot = drive.actionBuilder(new Pose2d(39, 13, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(31, 13), Math.toRadians(90));

        TrajectoryActionBuilder WallGrabSpot2 = drive.actionBuilder(new Pose2d(31, 13, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(31, 10), Math.toRadians(90));

        TrajectoryActionBuilder PutItOn = drive.actionBuilder(new Pose2d(31, 10, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-11, 34), Math.toRadians(90));

        TrajectoryActionBuilder backItUp = drive.actionBuilder(new Pose2d(-11, 34, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-11, 26), Math.toRadians(90));


//jacob scherrer smells like moldy cheese
        //jacob scherrer is not sigma
        //michael walsh is the goat of robotics
        //michael walsh should get girls but hunter win is to rizzy and took all the girls
        //hunter win is the greatest
        SequentialAction grab1 = new SequentialAction(
                WallGrabSpot.build(),
                sleep.half(),
                WallGrabSpot2.build(),
                sleep.quarter(),
                outGrasper.close(),
                sleep.quarter(),
                new ParallelAction(
                        inSlider.in(),
                        inRotator.straight(),
                        PutItOn.build(),
                        outGrasper.close(),
                        outLowerSwivel.specimen(),
                        outSwivel.NO()
                ),
                sleep.quarter(),
                outSwivel.specimenEnd(),
                outLowerSwivel.specimenEnd(),
                sleep.none(),
                backItUp.build(),
                sleep.none(),
                //hello michael walsh 
                //bye Jacob Share
                outGrasper.open()
        );

        TrajectoryActionBuilder WallGrabSpotr2 = drive.actionBuilder(new Pose2d(-9, 26, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(31, 13), Math.toRadians(90));

        TrajectoryActionBuilder WallGrabSpotr22 = drive.actionBuilder(new Pose2d(31, 13, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(31, 10), Math.toRadians(90));

        TrajectoryActionBuilder PutItOnr2 = drive.actionBuilder(new Pose2d(31, 10, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-9, 34), Math.toRadians(90));

        TrajectoryActionBuilder backItUpr2 = drive.actionBuilder(new Pose2d(-9, 34, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-9, 26), Math.toRadians(90));

        SequentialAction grab2 = new SequentialAction(
                WallGrabSpotr2.build(),
                outSwivel.intake(),
                outLowerSwivel.intake(),
                outGrasper.open(),
                sleep.half(),
                WallGrabSpotr22.build(),
                sleep.quarter(),
                outGrasper.close(),
                sleep.quarter(),
                new ParallelAction(
                        inSlider.in(),
                        inRotator.straight(),
                        PutItOnr2.build(),
                        outGrasper.close(),
                        outLowerSwivel.specimen(),
                        outSwivel.NO()
                ),
                sleep.quarter(),
                outSwivel.specimenEnd(),
                outLowerSwivel.specimenEnd(),
                sleep.none(),
                backItUpr2.build(),
                sleep.none(),
                outGrasper.open()
        );

        TrajectoryActionBuilder WallGrabSpotr3 = drive.actionBuilder(new Pose2d(-6, 26, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(31, 13), Math.toRadians(90));

        TrajectoryActionBuilder WallGrabSpotr32 = drive.actionBuilder(new Pose2d(31, 13, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(31, 10), Math.toRadians(90));

        TrajectoryActionBuilder PutItOnr3 = drive.actionBuilder(new Pose2d(31, 10, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-6, 34), Math.toRadians(90));

        TrajectoryActionBuilder backItUpr3 = drive.actionBuilder(new Pose2d(-6, 34, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-6, 26), Math.toRadians(90));

        SequentialAction grab3 = new SequentialAction(
                WallGrabSpotr3.build(),
                outSwivel.intake(),
                outLowerSwivel.intake(),
                outGrasper.open(),
                sleep.half(),
                WallGrabSpotr32.build(),
                sleep.quarter(),
                outGrasper.close(),
                sleep.quarter(),
                new ParallelAction(
                        inSlider.in(),
                        inRotator.straight(),
                        PutItOnr3.build(),
                        outGrasper.close(),
                        outLowerSwivel.specimen(),
                        outSwivel.NO()
                ),
                sleep.quarter(),
                outSwivel.specimenEnd(),
                outLowerSwivel.specimenEnd(),
                sleep.none(),
                backItUpr3.build(),
                sleep.none(),
                outGrasper.open()
        );

        TrajectoryActionBuilder WallGrabSpotr4 = drive.actionBuilder(new Pose2d(-6, 26, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(31, 13), Math.toRadians(90));

        TrajectoryActionBuilder WallGrabSpotr42 = drive.actionBuilder(new Pose2d(31, 13, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(31, 10), Math.toRadians(90));

        TrajectoryActionBuilder PutItOnr4 = drive.actionBuilder(new Pose2d(31, 10, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-3, 34), Math.toRadians(90));

        TrajectoryActionBuilder backItUpr4 = drive.actionBuilder(new Pose2d(-3, 34, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-3, 26), Math.toRadians(90));

        SequentialAction grab4 = new SequentialAction(
                WallGrabSpotr4.build(),
                outSwivel.intake(),
                outLowerSwivel.intake(),
                outGrasper.open(),
                sleep.half(),
                WallGrabSpotr42.build(),
                sleep.quarter(),
                outGrasper.close(),
                sleep.quarter(),
                new ParallelAction(
                        inSlider.in(),
                        inRotator.straight(),
                        PutItOnr4.build(),
                        outGrasper.close(),
                        outLowerSwivel.specimen(),
                        outSwivel.NO()
                ),
                sleep.quarter(),
                outSwivel.specimenEnd(),
                outLowerSwivel.specimenEnd(),
                sleep.none(),
                backItUpr4.build(),
                sleep.none(),
                outGrasper.open()
        );

        waitForStart();


        Actions.runBlocking(
                new SequentialAction(

                        inSwivel.scan(),
                        new ParallelAction(
                                inSlider.in(),
                                inRotator.straight(),
                                goToWallFirst.build(),
                                outGrasper.close(),
                                outLowerSwivel.specimen(),
                                outSwivel.specimenStart()
                        ),
                        sleep.none(),
                        outSwivel.specimenEnd(),
                        outLowerSwivel.specimenEnd(),
                        sleep.none(),
                        backupALittle.build(),
                        sleep.none(),
                        outGrasper.open(),
                        GO.build(),
                        inGrasper.open(),
                        new ParallelAction(
                                inSlider.out(),
                                inSwivel.down(),
                                inRotator.grab()
                        ),
                        sleep.half(),
                        inGrasper.close(),
                        sleep.quarter(),
                        new ParallelAction(
                                inSwivel.scan(),
                                Swing1.build()
                        ),
                        inGrasper.open(),
                        SwingBack.build(),
                        sleep.small(),
                        inSwivel.down(),
                        sleep.quarter(),
                        inGrasper.close(),
                        sleep.small(),
                        inSwivel.scan(),
                        Swing2.build(),
                        inGrasper.open(),
                        SwingBack2.build(),
                        sleep.small(),
                        inSwivel.down(),
                        sleep.small(),
                        inGrasper.close(),
                        sleep.small(),
                        new ParallelAction(
                                Swing3.build(),
                                inSwivel.scan()
                        ),
                        inGrasper.open(),
                        outSwivel.intake(),
                        outLowerSwivel.intake(),
                        outGrasper.open(),
                        grab1,
                        grab2,
                        grab3,
                        grab4
                )
        );


        while (opModeIsActive())
        {

        }
    }
}

