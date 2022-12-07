package org.firstinspires.ftc.teamcode.drive.Autonom;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "autonom")
public class Autonom extends LinearOpMode
{

    SampleMecanumDrive Drive;

    double maxMotorPower = 0.1; // TODO: max motor power

    @Override
    public void runOpMode() throws InterruptedException
    {
        Init();
        waitForStart();
        while (!isStopRequested())
        {
            Trajectory tr1 = Drive.trajectoryBuilder(new Pose2d())
                    .forward(40)
                    .build();
            Trajectory tr2 = Drive.trajectoryBuilder(new Pose2d(0,40))
                    .back(40)
                    .build();
            Drive.followTrajectory(tr1);
            sleep(1000);
            Drive.followTrajectory(tr2);

            sleep(30000);
        }
    }

    void Init()
    {
        Drive = new SampleMecanumDrive(hardwareMap);
        Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Drive.setPoseEstimate(new Pose2d(0, 0));
        Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}