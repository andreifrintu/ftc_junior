package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Vector;

import Utils.SimpleControls;

// Robot mic

@TeleOp
public class JunioriTeleOpe extends LinearOpMode
{
    SampleMecanumDrive mecanumDrive;
    DcMotorEx liftMotor1, liftMotor2, plateMotor;
    int cp1 = 0, cp2 = 0, pp = 0;
    SimpleControls simple;

    double suppress;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Init();
        waitForStart();

        while (!isStopRequested() && opModeIsActive())
        {
            Suppress();
            controlArm();
            controlWheels();
            Temeletry();
        }
    }


    void Init()
    {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanumDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
        plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        plateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        simple = new SimpleControls();
    }

    private void Suppress()
    {
        if (gamepad1.right_bumper)
            suppress = 0.3;
        else
            suppress = 1;
    }

    private void controlWheels()
    {
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();

        Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * suppress,
                    -gamepad1.left_stick_x * suppress
            ).rotated(-poseEstimate.getHeading());
        mecanumDrive.setDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -(gamepad1.right_trigger-gamepad1.left_trigger) * suppress
                )
        );

        telemetry.addData("mV", input);
    }

    // bravo codu n-ai initializat motoarele
    private void controlArm() {
        if (gamepad2.left_stick_y != 0) {
            liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            cp1 = liftMotor1.getCurrentPosition();
            liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            cp2 = liftMotor2.getCurrentPosition();
            if(liftMotor1.getTargetPosition() >= 0f && gamepad2.left_stick_y > 0) liftMotor1.setPower(0f);
            else liftMotor1.setPower(1f * gamepad2.left_stick_y);
            if(liftMotor2.getTargetPosition() >= 0f && gamepad2.left_stick_y > 0) liftMotor2.setPower(0f);
            else liftMotor2.setPower(1f * gamepad2.left_stick_y);
        } else   {
            liftMotor1.setTargetPosition(cp1);
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setTargetPosition(cp2);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(liftMotor1.isBusy()) {
                if (liftMotor1.getCurrentPosition() > -10 && liftMotor1.getTargetPosition() > -10)
                    liftMotor1.setPower(0f);
                else liftMotor1.setPower(1f);
            } else {
                if (liftMotor1.getCurrentPosition() > -10 && liftMotor1.getTargetPosition() > -10)
                    liftMotor1.setPower(0f);
                else liftMotor1.setPower(0.1f);
            }
            if(liftMotor2.isBusy()) {
                if (liftMotor2.getCurrentPosition() > -10 && liftMotor2.getTargetPosition() > -10)
                    liftMotor2.setPower(0f);
                else liftMotor2.setPower(1f);
            } else {
                if (liftMotor2.getCurrentPosition() > -10 && liftMotor2.getTargetPosition() > -10)
                    liftMotor2.setPower(0f);
                else liftMotor2.setPower(0.1f);
            }
        }
        if (gamepad2.right_trigger != 0 || gamepad2.left_trigger != 0) {
            plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pp = plateMotor.getCurrentPosition();
            plateMotor.setPower(1f * (gamepad2.right_trigger - gamepad2.left_trigger));
        } else {
            plateMotor.setTargetPosition(pp);
            plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(plateMotor.isBusy())
                plateMotor.setPower(1f);
            else
                plateMotor.setPower(0f);
        }
    }


    void Temeletry()
    {
        telemetry.addData("X:", gamepad1.left_stick_x);
        telemetry.addData("Y:", gamepad1.left_stick_y);
        telemetry.update();
    }
}