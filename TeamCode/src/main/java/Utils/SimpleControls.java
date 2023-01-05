package Utils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class SimpleControls
{
// Drive

    public void SetLiftPos(DcMotor Lift, int pos)
    {
        SetLiftPos(Lift, pos, 1.0);
    }

    public void SetLiftPos(DcMotor Lift, int pos, double power)
    {
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setTargetPosition(-pos);
        Lift.setPower(power);
    }


    public void SetLiftPos(DcMotor Lift1, DcMotor Lift2, int pos1, int pos2)
    {
        SetLiftPos(Lift1, Lift2, pos1, pos2, 1.0f);
    }

    public void SetLiftPos(DcMotor Lift1, DcMotor Lift2, int pos1, int pos2, double power)
    {
        Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Lift1.setTargetPosition(-pos1);
        Lift2.setTargetPosition(-pos2);

        Lift1.setPower(power);
        Lift2.setPower(power);
    }

// Servo
    public void SetServo(Servo servo,boolean open)
    {
        servo.setPosition(open ? 0.4 : 0.0);
    }
}
