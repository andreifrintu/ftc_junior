package org.firstinspires.ftc.teamcode.drive.Autonom;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import Utils.SimpleControls;

class Positions
{
    static int Ground = 0;
    static int Low = 0;
    static int Medium = 0;
    static int High = 0;
};

public class Autonom extends LinearOpMode
{
    SampleMecanumDrive drive;
    DcMotorEx liftMotor1, liftMotor2, plateMotor;
    Servo catcher;

    SimpleControls controls;

    TrajectorySequence trajectory;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Init();
        trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(50)
                .back(20)
                .addTemporalMarker(0.0, () -> controls.SetLiftPos(liftMotor1, liftMotor2, Positions.High, Positions.High))
                .build();
        waitForStart();

        Run();
    }

    private void Run()
    {

    }

    private void Init()
    {
        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
        plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
        catcher = hardwareMap.get(Servo.class, "catcherServo");
        catcher.setPosition(0);
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        plateMotor.setDirection(DcMotor.Direction.REVERSE);
        plateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        controls = new SimpleControls();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setPoseEstimate(new Pose2d(0, 0));
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
