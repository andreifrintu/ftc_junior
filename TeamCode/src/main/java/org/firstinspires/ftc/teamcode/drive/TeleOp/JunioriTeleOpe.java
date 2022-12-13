package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Vector;

import Utils.SimpleControls;

// Robot mic

@TeleOp
public class JunioriTeleOpe extends LinearOpMode
{
    SampleMecanumDrive mecanumDrive;

    SimpleControls simple;

    double suppress;

    boolean lrSwitch = false; // ?

    @Override
    public void runOpMode() throws InterruptedException
    {
        Init();
        waitForStart();

        while (!isStopRequested())
        {
            Suppress();
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

        simple = new SimpleControls();
    }

    private void Suppress()
    {
        if (gamepad1.right_bumper)
            suppress = 0.3;
        else
            suppress = 0.6;
    }

    boolean lastPressedStart = false;
    private void controlWheels()
    {
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
        boolean pressedStart = gamepad1.start;
        if (pressedStart&&!lastPressedStart)
            lrSwitch = !lrSwitch;
        lastPressedStart = pressedStart;

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

    void Temeletry()
    {
        telemetry.addData("X:", gamepad1.left_stick_x);
        telemetry.addData("Y:", gamepad1.left_stick_y);
        telemetry.update();
    }
}