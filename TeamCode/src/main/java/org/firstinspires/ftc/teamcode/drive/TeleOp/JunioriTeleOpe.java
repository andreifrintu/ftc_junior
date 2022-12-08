package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import Utils.SimpleControls;

// Robot mic

@TeleOp
public class JunioriTeleOpe extends LinearOpMode
{
    SampleMecanumDrive mecanumDrive;
    Gamepad gamepad1;

    SimpleControls simple;

    double suppress1 = 0.2;

    boolean lrSwitch = false; // ?

    @Override
    public void runOpMode() throws InterruptedException
    {
        Init();


        while (!isStopRequested())
        {
            controlWheels();
            Temeletry();
        }
    }


    void Init()
    {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);

        gamepad1 = new Gamepad();

        simple = new SimpleControls();
    }

    void Movement()
    {
        if (gamepad1.left_bumper || gamepad1.right_bumper)
            suppress1 = 0.3;
        else
            suppress1 = 0.6;

        Pose2d moveVec = new Pose2d(gamepad1.left_stick_x * suppress1, gamepad1.left_stick_y * suppress1);

        mecanumDrive.setWeightedDrivePower(moveVec);
    }

    boolean lastPressedStart = false;
    private void controlWheels()
    {
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
        boolean pressedStart = gamepad1.start;
        if (pressedStart&&!lastPressedStart)
            lrSwitch = !lrSwitch;
        lastPressedStart = pressedStart;

        Vector2d input;
        if (lrSwitch)
        {
            input = new Vector2d(
                    -gamepad1.left_stick_y*suppress1,
                    -gamepad1.left_stick_x*suppress1
            ).rotated(-poseEstimate.getHeading());
        }
        else
        {
            input = new Vector2d(
                    -gamepad1.right_stick_y*suppress1,
                    -gamepad1.right_stick_x*suppress1
            ).rotated(-poseEstimate.getHeading());
        }

        mecanumDrive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -(gamepad1.right_trigger-gamepad1.left_trigger)*suppress1
                )
        );
    }

    void Temeletry()
    {
        telemetry.addData("X:", gamepad1.left_stick_x);
        telemetry.addData("Y:", gamepad1.left_stick_y);
        telemetry.update();
    }
}