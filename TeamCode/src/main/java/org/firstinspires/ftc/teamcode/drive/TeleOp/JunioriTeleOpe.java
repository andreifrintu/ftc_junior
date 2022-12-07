package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import Utils.SimpleControls;

@TeleOp
public class JunioriTeleOpe extends LinearOpMode
{
    SampleMecanumDrive mecanumDrive;
    Gamepad gp1;

    SimpleControls simple;

    double suppress1 = 0.2;

    boolean lr=false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Init();


        while (!isStopRequested())
        {
            controlWheels();
            Temetry();
        }
    }


    void Init()
    {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);

        gp1 = new Gamepad();

        simple = new SimpleControls();
    }

    void Movement()
    {
        if (gp1.left_bumper)
            suppress1 = 0.4;
        else
            suppress1 = 0.7;

        Pose2d moveVec = new Pose2d(gp1.left_stick_x * suppress1, gp1.left_stick_y * suppress1);

        mecanumDrive.setWeightedDrivePower(moveVec);
    }

    boolean lastPressedStart = false;
    private void controlWheels()
    {
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
        boolean pressedStart = gamepad1.start;
        if (pressedStart&&!lastPressedStart) lr = !lr;
        lastPressedStart = pressedStart;
        if (lr) {
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y*suppress1,
                    -gamepad1.left_stick_x*suppress1
            ).rotated(-poseEstimate.getHeading());
            mecanumDrive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -(gamepad1.right_trigger-gamepad1.left_trigger)*suppress1
                    )
            );
        }
        else {
            Vector2d input = new Vector2d(
                    -gamepad1.right_stick_y*suppress1,
                    -gamepad1.right_stick_x*suppress1
            ).rotated(-poseEstimate.getHeading());
            mecanumDrive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -(gamepad1.right_trigger-gamepad1.left_trigger)*suppress1
                    )
            );
        }
    }

    void Temetry()
    {
        telemetry.addData("X:", gp1.left_stick_x);
        telemetry.addData("Y:", gp1.left_stick_y);
        telemetry.update();
    }
}