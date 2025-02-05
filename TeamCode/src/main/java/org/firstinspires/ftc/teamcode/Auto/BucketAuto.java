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

@Autonomous(name="BucketAuto :(", group="Autonomous")
public class BucketAuto extends LinearOpMode {

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

        public Action straight() {
            return new RotateStraight();
        }

        public Action left() {
            return new RotateLeft();
        }

        public Action right() {
            return new RotateRight();
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

        public IntakeSlider(HardwareMap hardwareMap) {
            inSlider = hardwareMap.get(Servo.class, "intake1");
        }

        public class IntakeOut implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inSlider.setPosition(INTAKE_OUT);
                return false;
            }
        }

        public class IntakeIn implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inSlider.setPosition(INTAKE_IN);
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

    }




    @Override
    public void runOpMode() {

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


        TrajectoryActionBuilder action = drive.actionBuilder(pose)
                .strafeToLinearHeading(new Vector2d(-15, 8), Math.toRadians(45));


        waitForStart();

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

        Actions.runBlocking(
                new SequentialAction(
                        outGrasper.open(),
                        transfer,
                        action.build(),
                        outGrasper.close()

                )
        );


        while (opModeIsActive())
        {

        }



    }
}

