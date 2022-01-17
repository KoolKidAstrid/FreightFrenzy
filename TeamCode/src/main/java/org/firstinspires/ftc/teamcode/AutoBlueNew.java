package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Auto Blue Main", group="Linear Opmode")
public class AutoBlueNew extends LinearOpMode {

    @Override
    public void runOpMode() {
        NyxTheRobot nyx = new NyxTheRobot(this);
        nyx.initialize();
        waitForStart();

//        drive to wobble thing
        nyx.drive(-12, 0.5);

        int down = 1050;
//        raise head to wobble thing top layer
//        while (nyx.ARM2.getCurrentPosition() > down + 10 || nyx.ARM2.getCurrentPosition() < down - 10)
//            nyx.setArm2((float) down/1725f);
//        nyx.ARM2.setPower(0);

        nyx.setArmAuto(down);

//        deposit block
        nyx.intake(-1);
        sleep(3000);
        nyx.intake(0);

//        lower head
//        while (nyx.ARM2.getCurrentPosition() > 50 || nyx.ARM2.getCurrentPosition() < -50)
//            nyx.setArm2(0f/1725f);
//        nyx.ARM2.setPower(0);

        nyx.setArmAuto(0);

//        drive backwards
        nyx.drive(6, 0.5);

//        turn to carousel
        nyx.newTurn(90);

//        drive to carousel
        nyx.drive(48, 0.5);
        nyx.drive(3, 0.25);

//        spin :)
        nyx.autoDucks(5, 0.25);


//        turn to square
        nyx.drive(-3, 0.25);
        nyx.newTurn(-120);

//        park
        nyx.drive(-16, 0.5);
        nyx.newTurn(30);
        nyx.drive(-2, 0.5);
    }
}
