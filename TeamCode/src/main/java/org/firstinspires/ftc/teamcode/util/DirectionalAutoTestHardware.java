package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.Timer;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DirectionalAutoTestHardware
{
    
    public double driveVelMax = 2400;        // maximum drive velocity (ticks/sec)
    
    // liftScore, liftPowerScore, boxArmScore, boxArmPre, blinkF, blinkB
    public double[][] targets = {
        {790, 0.95, 0.45, 0.61, 94, 94}, //Lev3
        {0, 0.95, 0.15, 0.61, 84, 84}, //Shared
        {790, 0.4, 0.24, 0.24, 96, 96}, //Lev2
        
        {850, 0.95, 0.47, 0.61, 97, 94}, //Lev3A
        {0, 0.95, 0.15, 0.61, 97, 84}, //SharedA
        {850, 0.4, 0.24, 0.24, 97, 96}, //Lev2A
        
        {430, 0.95, 0.47, 0.61, 100, 94}, //Lev3D
        {0, 0.95, 0.15, 0.61, 100, 84}, //SharedD
        {270, 0.4, 0.30, 0.24, 100, 96}, //Lev2D
        
        {430, 0.95, 0.47, 0.61, 100, 97}, //Lev3DA
        {0, 0.95, 0.15, 0.61, 100, 97}, //SharedDA
        {270, 0.4, 0.30, 0.24, 100, 97}, //Lev2DA
        
        {850, 0.95, 0.45, 0.61, 68, 68}, //Lev3P
        {750, 0.4, 0.24, 0.24, 68, 68}, //Lev2P
        {770, 0.4, 0, 0.61, 68, 68} //Lev1P
    }; 

    public int intakeArmC = 0;        // position of arm "UP"
    public int intakeArmF = 510;        // ticks from "UP" to magnetF
    public int intakeArmB = -490;       // ticks from "UP" to magnetB
    public int intakeArmDown = intakeArmB;   // where to go on Mode.INTAKE
    public int intakeArmRange = 50;     // acceptable "in position" range
    public double intakeArmMid = 0.45;
    public double intakeArmVel = 0;
    public double intakeArmVelMax = 2800;

    public int liftRange = 50;
    public int liftScore = 790;
    public double liftPower = 0;
    public double liftPowerScore = 0.95;
    public boolean liftRetract = false;
    public int liftOffset = 0;

    public double boxArmFLoad = 0.90;
    public double boxArmFPre = 0.61;
    public double boxArmFScore = 0.47;
    public double boxArmFPos = boxArmFLoad;

    public double boxArmBLoad = 0.90;
    public double boxArmBPre = 0.61;
    public double boxArmBScore = 0.47;
    public double boxArmBPos = boxArmBLoad;

    public double boxDropOpen = 0.7;
    public double boxDropClose = 0.27;

    public double turretC = 0.56;
    public double turretF = 0.775;
    public double turretB = 0.325;
    public double turretDown = turretF;
    public double turretSwing = 0.05;

    public double intakePower = 0;
    public double intakePowerIntake = 1;
    public double intakePowerOuttake = -1;

    public boolean redCalibrated = false;
    public boolean blueCalibrated = false;
    
    public double th = 0;
    public boolean heading = false;
    
    public int lineBOffset = 0;
    public int lineFOffset = 0;
    
    public double tapePos = 0.5;
    public double panPos = 0.2;
    public double tiltPos = 0.65;

    public enum Mode {
        IDLE, INTAKE, LOAD, EXTEND,
        EXTENDED, SCORE, RETRACT, INTAKEDOWN,
        PREEXTEND, LOADIDLE, AUTOEXTEND
    }
    public Mode smodeNext[] = { 
        Mode.IDLE, Mode.LOAD, Mode.EXTEND, Mode.EXTENDED,
        Mode.IDLE, Mode.RETRACT, Mode.IDLE, Mode.IDLE, 
        Mode.IDLE, Mode.IDLE, Mode.SCORE
    };
    public Mode smode = Mode.IDLE;
    ElapsedTime smodeTimer = null;
    double modetime = 0;
    int wallDisablePos = Integer.MAX_VALUE;
    int wallTargetPos = wallDisablePos;
    int wallDecelTicks = 600;
    double wallVel = driveVelMax / 3.0;
    DcMotorEx wallEncoder = null;

    //Create variables for hardware
    public DcMotorEx lf   = null;
    public DcMotorEx rf   = null;
    public DcMotorEx lb   = null;
    public DcMotorEx rb   = null;

    public DcMotor lift = null;
    public DcMotorEx intakeArm = null;
    public DcMotor intake = null;

    public Servo turret = null;
    public Servo boxDrop = null;
    public Servo boxArmF = null;
    public Servo boxArmB = null;
    public Servo carousel = null;
    public Servo pan = null;
    public Servo tilt = null;
    public Servo tape = null;

    public DigitalChannel magnetF = null;
    public DigitalChannel magnetB = null;
    public DigitalChannel limit = null;
    public DigitalChannel grabdist = null;
    public DistanceSensor intakedist = null;

    public BNO055IMU imu = null;

    public RevBlinkinLedDriver blink0 = null;
    public RevBlinkinLedDriver blink1 = null;
    public RevBlinkinLedDriver.BlinkinPattern[] bpatterns = 
        RevBlinkinLedDriver.BlinkinPattern.values();
    
    public DigitalChannel lineB = null;
    public DigitalChannel lineF = null;

    //Create Hardware Map Object
    HardwareMap hwMap = null;

    //Initialize Hardware That
    //Comes from the Config
    public void init(HardwareMap ahwMap) {
        
        // Save reference to Hardware map
        hwMap = ahwMap;
        
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        
        smodeTimer = new ElapsedTime();

        // Define and Initialize Motors
        lf = (DcMotorEx)hwMap.get(DcMotor.class, "lf");
        rf = (DcMotorEx)hwMap.get(DcMotor.class, "rf");
        lb = (DcMotorEx)hwMap.get(DcMotor.class, "lb");
        rb = (DcMotorEx)hwMap.get(DcMotor.class, "rb");
        lift = hwMap.get(DcMotor.class, "elev");
        intakeArm = (DcMotorEx)hwMap.get(DcMotor.class, "iarm");
        intake = hwMap.get(DcMotor.class, "intake");

        turret = hwMap.get(Servo.class, "turret");
        boxArmF = hwMap.get(Servo.class, "sarm2");
        boxArmB = hwMap.get(Servo.class, "sarm1");
        boxDrop = hwMap.get(Servo.class, "sbox");
        carousel = hwMap.get(Servo.class, "carousel");
        
        pan = hwMap.get(Servo.class, "pan");
        tilt = hwMap.get(Servo.class, "tilt");
        tape = hwMap.get(Servo.class, "tape");
        
        magnetF = hwMap.get(DigitalChannel.class, "magnetRed");
        magnetB = hwMap.get(DigitalChannel.class, "magnetBlue");
        limit = hwMap.get(DigitalChannel.class, "limit");
        // grabdist = hwMap.get(DigitalChannel.class, "grabdist");
        intakedist = hwMap.get(DistanceSensor.class, "intakedist");
        
        lineB = hwMap.get(DigitalChannel.class, "lineb");
        lineF = hwMap.get(DigitalChannel.class, "linef");

        // Set all motors to zero power
        //Set servos to starting position
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);

        lift.setPower(0);
        intakeArm.setVelocity(0);
        intakeArm.setVelocityPIDFCoefficients(10, 0, 0, 12);
        intakeArm.setPositionPIDFCoefficients(12);
        intake.setPower(0);

        boxArmF.setPosition(boxArmFLoad);
        boxArmB.setPosition(boxArmBLoad);
        boxDrop.setPosition(boxDropClose);
        carousel.setPosition(0.5);
        turret.setPosition(turretC);
        
        pan.setPosition(panPos);
        tilt.setPosition(tiltPos);
        tape.setPosition(tapePos);
        
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD); 
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setDirection(DcMotor.Direction.FORWARD);
        intakeArm.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeArm.setTargetPosition(0);
        intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        magnetF.setMode(DigitalChannel.Mode.INPUT);
        magnetB.setMode(DigitalChannel.Mode.INPUT);
        limit.setMode(DigitalChannel.Mode.INPUT);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*
        lf.setVelocityPIDFCoefficients(5, 2, 0.5, 11);
        rf.setVelocityPIDFCoefficients(5, 2, 0.5, 11);
        lb.setVelocityPIDFCoefficients(5, 2, 0.5, 11);
        rb.setVelocityPIDFCoefficients(5, 2, 0.5, 11);
        */
        
        blink0 = hwMap.get(RevBlinkinLedDriver.class, "blink0");
        blink1 = hwMap.get(RevBlinkinLedDriver.class, "blink1");
        
        setIntakeB();
        wallEncoder = rb;
        
        intakeArmC = intakeArm.getCurrentPosition()+240;

        initIMU(hwMap);
        
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    
    public void update(){
        boolean downF = !magnetF.getState();
        boolean downB = !magnetB.getState();

        if (downF) {
            intakeArmC = (intakeArm.getCurrentPosition() - intakeArmF);
        }
        if (downB) {
            intakeArmC = (intakeArm.getCurrentPosition() - intakeArmB);
        }
        
        if (smode == Mode.INTAKE || smode == Mode.INTAKEDOWN) {
            boxDrop.setPosition(boxDropOpen);
            intakeArmVel = intakeArmVelMax;
            if (inRange(intakeArm, intakeArmRange) && (downF || downB)) {
                intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                intakeArmVel = 0;
            }
            intakeArmDown(intakeArmVel);
            double swing = turretSwing*Math.sin(smodeTimer.seconds()*Math.PI*2*2);
            turret.setPosition(turretDown+swing);
            if (smode == Mode.INTAKEDOWN && inRange(intakeArm, intakeArmRange)) {
                nextMode();
            }
        }
        if (smode == Mode.INTAKE) {
            if (intakePower >= 0) {
                intakePower = 1.0;
            }
            if(isIntakeFull()) {
                nextMode();
            }
        }

        if (smode == Mode.LOAD || smode == Mode.LOADIDLE) {
            boxDrop.setPosition(boxDropOpen);
            intakeArmUp();
            if (intakePower >= 0) {
                intakePower = 1;
            }
            if(inRange(intakeArm, intakeArmRange)){
                turret.setPosition(turretC);
                
                if (intakePower >= 0) {
                    intakePower = 0.75;
                }
                //if (isGrabFull(false)) { nextMode(); }
                if (intakedist.getDistance(DistanceUnit.CM) > 4) {
                    nextMode();
                }
            }
            else{
                turret.setPosition((turretC*2 + turretDown)/3);
            }
        }

        if (smode == Mode.PREEXTEND) {
            intakeArmMid();
            boxDrop.setPosition(boxDropClose);
            if(inRange(intakeArm, intakeArmRange)){
                // boxArmFPos = boxArmFPre;
                // boxArmBPos = boxArmBPre;
                nextMode();
            }
        }

        if (smode == Mode.EXTEND) {
            intakeArmMid();
            boxDrop.setPosition(boxDropClose);
            if(inRange(intakeArm, intakeArmRange)){
                lift.setTargetPosition(liftScore);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftPower = liftPowerScore;
                boxArmFPos = boxArmFScore;
                boxArmBPos = boxArmBScore;
                if (inRange(lift, liftRange)) {
                    nextMode ();
                }
            }
        }

        if (smode == Mode.AUTOEXTEND) {
            boxDrop.setPosition(boxDropClose);
            lift.setTargetPosition(liftScore);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftPower = liftPowerScore;
            boxArmFPos = boxArmFScore;
            boxArmBPos = boxArmBScore;
            if (lift.getCurrentPosition() > 100) {
                intakeArmUp();
            }
            if (inRange(lift, liftRange)) {
                intakeArmDown(intakeArmVelMax);
                turret.setPosition(turretDown);
                nextMode ();
            }
        }

        if (smode == Mode.EXTENDED) {
            liftPower = liftPowerScore - 0.1;
            
        }        
        
        if (smode == Mode.SCORE) {
            boxDrop.setPosition(boxDropOpen);
            if (!isGrabFull(true)) { nextMode(); }
            if (smodeTimer.seconds() > 0.3) {
                nextMode();
            }
        }
        
        if (smode == Mode.RETRACT) {
            boxArmFPos = boxArmFLoad;
            boxArmBPos = boxArmBLoad;
            if (!limit.getState())  nextMode();
            else {
                liftRetract = true;
            }
            
        }
        
        if (liftRetract) {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftPower = -1;
            if (inRange(intakeArm, 200, intakeArmC)) {
                intakeArmMid();
                liftPower = 0;
            }
            boxArmFPos = boxArmFLoad;
            boxArmBPos = boxArmBLoad;
            if (!limit.getState() || smodeTimer.seconds() > 3) {
                liftPower = 0;
                liftRetract = false;
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        
        lift.setPower(liftPower);
        liftPower = 0;

        // intakePower = 0;
        intake.setPower (intakePower);
        intakePower = 0;
        
        boxArmF.setPosition(boxArmFPos);
        boxArmB.setPosition(boxArmBPos);
        
        pan.setPosition(panPos);
        tilt.setPosition(tiltPos);
        tape.setPosition(tapePos);
        
        wallFollow();
    }
    
    public void stop() {
        driveYXW(0,0,0);
    }
    
    public void setMode(Mode m){
        if (smode != m) {
            modetime = smodeTimer.seconds();
            smode = m;
            smodeTimer.reset();
        }
    }

    public void nextMode() {
        setMode(smodeNext[smode.ordinal()]);
    }
    
    public void chainMode(Mode m, Mode n) {
        smodeNext[m.ordinal()] = n;
    }

    public boolean isMode(Mode m) {
        return smode == m;
    }
    
    public boolean isIdle() {
        return isMode(Mode.IDLE);
    }

    public void setIntakeF() {
        intakeArmDown = intakeArmF + 40;
        turretDown = turretF;
    }
    
    public void setIntakeB() {
        intakeArmDown = intakeArmB - 40;
        turretDown = turretB;
    }
    
    public boolean isIntakeFull() {
        return (intakedist.getDistance(DistanceUnit.CM) < 4.5); 
    }
    
    public void intakeArmMid() {
        intakeArm.setTargetPosition(intakeArmC + (int)(intakeArmDown * intakeArmMid));
        intakeArm.setVelocity(intakeArmVelMax);
    }
    
    public void intakeArmUp() {
        intakeArm.setTargetPosition(intakeArmC);
        intakeArm.setVelocity(intakeArmVelMax);
    }
    
    public void intakeArmDown(double vel) {
        intakeArm.setTargetPosition(intakeArmC + intakeArmDown);
        intakeArm.setVelocity(vel);
    }

    public void setTurretSwing(double s)  {
        turretSwing = s;
    }
    
    public void setOuttake() {
        intakePower = -1;
    }
    
    public boolean isGrabFull(boolean v) {
        if (grabdist != null) return !grabdist.getState();
        return v;
    }
    
    public void setCarousel(double speed) {
        carousel.setPosition(speed);
    }
    
    public void setScore(int row){
        liftScore = (int)targets[row][0];
        liftPowerScore = targets[row][1];
        boxArmFScore = targets[row][2];
        boxArmBScore = targets[row][2];
        boxArmFPre = targets[row][3];
        boxArmBPre = targets[row][3];
        blink1.setPattern(bpatterns[(int)(targets[row][4]-1)]);
        blink0.setPattern(bpatterns[(int)(targets[row][5]-1)]);
    }
    
    public void setScoreLev3() { setScore(0); }
    public void setScoreLev3A() { setScore(3); }
    public void setScoreLev2() { setScore(2); }
    public void setScoreShared() { setScore(1); }

    public boolean inRange(DcMotor m, int range, int targetPos) {
        return Math.abs(targetPos - m.getCurrentPosition()) <= range;
    }

    public boolean inRange(DcMotor m, int range) {
        return inRange(m, range, m.getTargetPosition());
    }
    
    
    public void initIMU(HardwareMap hwMap) {
        driveYXW(0,0,0);
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
    }
    
    public double getIMUHeading() {
        Orientation angles = imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public double getHeading() {
        return getIMUHeading();
    }
    
    public void driveYXW(double ry, double rx, double rw, double vel) {
        // ry == forward, rx == strafe, rw == turn
        lf.setVelocity(vel*(ry + rw + rx));
        rf.setVelocity(vel*(ry - rw - rx));
        lb.setVelocity(vel*(ry + rw - rx));
        rb.setVelocity(vel*(ry - rw + rx));
    }
    
    public void driveYXW(double ry, double rx, double rw) {
        driveYXW(ry, rx, rw, driveVelMax);
    }
    
    public void setWallTargetAbs(int targetPos, double vel, int decel, int offsetF, int offsetB) {
        wallTargetPos = targetPos;
        wallDecelTicks = decel;
        wallVel = vel * driveVelMax;
        lineFOffset = offsetF;
        lineBOffset = offsetB;
    }
    
    public void setWallTargetRel(int targetPos, double vel, int decel, int offsetF, int offsetB) {
        setWallTargetAbs(wallEncoder.getCurrentPosition() + targetPos, vel, decel, offsetF, offsetB);
    }
    
    public void stopWallTarget() {
        wallTargetPos = wallDisablePos;
    }

    public void wallFollow() {
        if (wallTargetPos == wallDisablePos) return;
        int currentPos = wallEncoder.getCurrentPosition ();
        if(!lineF.getState() && lineFOffset != 0){
            wallTargetPos = currentPos + lineFOffset;
        }
        else if(!lineB.getState() && lineBOffset != 0){
            wallTargetPos = currentPos + lineBOffset;
        }
        
        if(!lineF.getState()){
            blink1.setPattern(bpatterns[89]);
            blink0.setPattern(bpatterns[89]);
        }
        else if(!lineB.getState()){
            blink1.setPattern(bpatterns[78]);
            blink0.setPattern(bpatterns[78]);
        }
        else{
            blink1.setPattern(bpatterns[99]);
            blink0.setPattern(bpatterns[99]);
        }
        
        int dir = (currentPos < wallTargetPos) ? 1 : -1;
        double dist = Math.abs(wallTargetPos - currentPos);
        double vel = Range.clip(wallVel * dist / wallDecelTicks, 0, wallVel);
        double herror = getHeading() - th;
        
        if(th != 0 && !heading){
            driveYXW(dir, 0, herror * 0.04, vel);
        }
        else{
            driveYXW(dir, 0.17, herror * 0.04, vel);
        }
        
    }
    
    public void setTH(double heading, boolean bool){
        this.th = heading;
        this.heading = bool;
    }

    public boolean atWallTarget() {
        return inRange(wallEncoder, 100, wallTargetPos);
    }
    
    public int getWallPos() {
        return wallEncoder.getCurrentPosition();
    }
    public int getWallPos(DcMotorEx m){
        wallEncoder = m;
        return getWallPos();
    }
        
    
    public void liftAdjust(double pow) {
        setMode(Mode.IDLE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftPower = pow;
        liftRetract = false;
    }
    
    /*
    public void liftAdjust(int target) {
        setMode(Mode.EXTENDED);
        liftOffset += target;
        liftRetract = false;
    }
    */
    public void boxArmAdjust(double step) {
        setMode(Mode.IDLE);
        boxArmFPos += step;
        boxArmBPos -= step;
    }
    
    public void setlineBOffset(int i){
        lineBOffset = i;
    }
    
    public void setlineFOffset(int i){
        lineFOffset = i;
    }
    
    public void pan(double offset){
        panPos += offset;
    }
    
    public void tilt(double offset){
        tiltPos += offset;
    }
    
    public void setPan(double offset){
        panPos = offset;
    }
    
    public void setTilt(double offset){
        tiltPos = offset;
    }
    
    public void tape(double speed){
        tapePos = speed;
    }

    public void telemetry(Telemetry t) {
        t.addData("smode", smode);
        t.addData("modetime", modetime);
        if (grabdist != null)
            t.addData("grabdist", !grabdist.getState());
        // t.addData("PIDF", intakeArm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
        t.addData("intakeArmVel", intakeArmVel);
        t.addData("iarmPos", intakeArm.getCurrentPosition());
        t.addData("iarmTarget", intakeArm.getTargetPosition());
        t.addData("intakeArmC", intakeArmC);
        t.addData("liftPos", lift.getCurrentPosition());
        t.addData ("wallTargetPos", wallTargetPos);
        t.addData ("rb", rb.getCurrentPosition());
        t.addData ("lb", lb.getCurrentPosition());
        t.addData ("rf", rf.getCurrentPosition());
        t.addData ("lf", lf.getCurrentPosition());
        t.addData ("currentPos", rb.getCurrentPosition());
        t.addData("atWallTarget", atWallTarget());
        t.addData ("intakeDist", intakedist.getDistance(DistanceUnit.CM));
        t.addData("liftRetract", liftRetract);
        t.addData("pan", panPos);
        t.addData("tilt", tiltPos);
        t.addData("tape", tapePos);
        t.addData("Intake Arm Range", inRange(intakeArm, intakeArmRange));
        t.update();
    }
}


