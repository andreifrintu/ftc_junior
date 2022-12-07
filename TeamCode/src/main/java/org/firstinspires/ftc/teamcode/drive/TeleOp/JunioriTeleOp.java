package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import Utils.SimpleControls;

@TeleOp
public class JunioriTeleOp extends LinearOpMode
{
    SampleMecanumDrive Drive;
    Gamepad gp1;

    SimpleControls simple;

    double suppress;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Init();


        while (!isStopRequested())
        {
            Movement();
        }
    }


    void Init()
    {
        Drive = new SampleMecanumDrive(hardwareMap);

        gp1 = new Gamepad();

        simple = new SimpleControls();
    }

    void Movement()
    {
        if (gp1.left_bumper)
            suppress = 0.4;
        else
            suppress = 0.7;

        Pose2d moveVec = new Pose2d(gp1.left_stick_x * suppress, gp1.left_stick_y * suppress);

        Drive.setWeightedDrivePower(moveVec);
    }
}