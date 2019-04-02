//Head Coder: Nishant Sathe
//Teammates: Alex King, Michelle Lee, Hrithik Basu
//Team: Press F To Pay Respects (D5)

//Including all libraries
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <math.h>
#include <LCDColors.h>
#include <FEHRPS.h>

//Instantiation of devices
AnalogInputPin photoresis(FEHIO::P1_0);
FEHMotor leftmotor(FEHMotor::Motor0,9.0);//the left motor is on port 0 on the proteus
FEHMotor rightmotor(FEHMotor::Motor1,9.0);//the right motor is on port 1 on the proteus
DigitalEncoder left_encoder(FEHIO::P0_0);//left motor encoder is current set to second port of 0 bank on proteus
DigitalEncoder right_encoder(FEHIO::P0_1);//right motor encoder is currently set to first port of 0 bank on proteus
FEHServo tokenServo(FEHServo::Servo7);
FEHServo foosballArm(FEHServo::Servo0);


//Defining constants for use throughout the code
#define FORWARD_PERCENT -25.0//This defines the default speed at which the robot goes forward
#define COUNTS_PER_INCH 33.74//This defines how far per inch the robot will go in specification with the encoder. There are 318 counts in one revolution.
#define BACKWARDS_PERCENT 18.0//This defines how fast the robot will go while moving backwards
#define COUNTS_PER_DEGREE 2.15//This defines how many counts the encoder should do per degree


//PID constants, tweak for tuning
float Kp = 1.5;
float Ki = 5;
float Kd = 0;

int DEBUG = 0;

int ddrCheck=0;

void checkHeading(int degree){
    float timeout = TimeNow();
    float finalTime = timeout+5.;

    while(1==1){
        LCD.WriteLine("Degree");
        LCD.WriteLine(RPS.Heading());
        timeout++;
       if(timeout>finalTime){
            break;
        }
       else if(RPS.Heading()<degree){
        leftmotor.SetPercent(15.);
        rightmotor.SetPercent(-15.);
        Sleep(100);
        rightmotor.Stop();
        leftmotor.Stop();
        }
        else if(RPS.Heading()>degree){
            leftmotor.SetPercent(-15.);
            rightmotor.SetPercent(15.);
            Sleep(100);
            rightmotor.Stop();
            leftmotor.Stop();
            Sleep(100);
        }

        else{
            break;
        }


    }

}


void startUp(){//This function waits until the proteus screen has been tapped to start the instruction set

        float x,y;
        LCD.Clear(BLACK);
        LCD.SetFontColor(WHITE);
        LCD.WriteLine("Touch the screen to start");
        while(!LCD.Touch(&x,&y)){} //Wait for screen to be pressed
        while(LCD.Touch(&x,&y)){} //Wait for screen to be unpressed

       float currentTime=TimeNow();
       float finalTime=TimeNow()+35;
        //button to button is 5.25, between the lights 13.5
        while(photoresis.Value()>.4){
            currentTime=TimeNow();
            if(currentTime>finalTime){
                break;
            }

        }
}



void move(float speed, float distance){//direction 1 is forward, direction -1 is backwards, distance in inches, this function tells the robot which direction to move in and how far
    int counts = abs(distance)*COUNTS_PER_INCH;//the number of counts is equal to the distance times the defined constant COUNTS_PER_INCH

    float inputSpeed = (speed/100)*225;      //Convert from percentage speed value to a speed in RPMs


    //DEBUG
    if(DEBUG>=1){
        LCD.Write("Target speed: ");
        LCD.WriteLine(inputSpeed);
        if(distance > 0){//If forward, show an acknowledgement on the proteus screen
            LCD.WriteLine("Going forward");
        }else if(distance < 0){//If backward, show an acknowledgement on the proteus screen
            LCD.WriteLine("Going backwards");
        }else{
            LCD.WriteLine("Improper distance parameter");
        }
        Sleep(1.5);
    }


    float leftSpeed, rightSpeed;            //Set up a variable for the speed on each side, since we're going to control them independantly
    leftSpeed = rightSpeed = inputSpeed;    //Start with both sides at the same speed we passed in

    right_encoder.ResetCounts();//reset the counts for both encoders
    left_encoder.ResetCounts();


    //Variable insantiation for PID
    float lastTime = TimeNow();                                                          //Previous timestamp. Used to calculate dt (change in time)
    int leftLastCounts = left_encoder.Counts();                                          //Previous encoder positions, used to measure the change in position
    int rightLastCounts = right_encoder.Counts();
    int leftCurrentCounts, rightCurrentCounts;                                           //The current position of the encoders. Used to calculate change in position, and thus, rpm
    float lRealSpeed, rRealSpeed, currTime, dt, lError, rError, lInt, rInt, lDeriv, rDeriv, lPrevError, rPrevError;     //A bunch of other variables used in the PID loop
    lInt = rInt = 0;
    Sleep(0.001);       //This somehow fixes things. I literally have no explanation, it just does.


    //Loop until we've measured the desired distance in encoder counts
    while(left_encoder.Counts() < counts){                  // && right_encoder.Counts() < counts
    //Update a bunch of variables to reflect the current state of the robot for PID calculations
        leftCurrentCounts = left_encoder.Counts();
        rightCurrentCounts = right_encoder.Counts();
        currTime = TimeNow();
        dt = currTime-lastTime;

        lRealSpeed = ((leftCurrentCounts-leftLastCounts)/318.0)/(dt/60.0);              //Calculate RPMs for the current iteration
        rRealSpeed = ((rightCurrentCounts-rightLastCounts)/318.0)/(dt/60.0);              //Calculate RPMs for the current iteration

    //PID for the left side
        lError = inputSpeed - lRealSpeed;                       //Proportional part of PID. Calculate the current error value
        lInt = lInt + (lError*dt);                              //Integral portion of PID. Sums error based on time with a reinmann sum
        lDeriv = (lError - lPrevError)/dt;                      //Derivative portion of PID. Change in error/change in time is calculated here
        leftSpeed = ((Kp*lError) + (Ki*lInt) + (Kd*lDeriv));     //Take all these values and set the speed to this

    //PID for the right side
        rError = inputSpeed - rRealSpeed;                       //Proportional part of PID. Calculate the current error value
        rInt = rInt + (rError*dt);                              //Integral portion of PID. Sums error based on time with a reinmann sum
        rDeriv = (rError - rPrevError)/dt;                      //Derivative portion of PID. Change in error/change in time is calculated here
        rightSpeed = ((Kp*rError) + (Ki*rInt) + (Kd*rDeriv));     //Take all these values and set the speed to this


        //Set motors to calculated values. Have to convert back to percentage from RPMs
        leftmotor.SetPercent((distance/abs(distance))*(leftSpeed*-1/2.25));
        rightmotor.SetPercent((distance/abs(distance))*(rightSpeed*-1/2.25));


        //DEBUG
        if(DEBUG>=2){
            //lError    lInt    rError  rInt
            LCD.SetFontColor(CRIMSON);
            LCD.Write(((int)(((lError))*10))/10);
            LCD.Write("\t");
            LCD.SetFontColor(LIME);
            LCD.Write(((int)(lInt*10))/10);
            LCD.Write("\t");
            LCD.SetFontColor(RED);
            LCD.Write(((int)(((rError))*10))/10);
            LCD.Write("\t");
            LCD.SetFontColor(GREEN);
            LCD.Write(((int)(rInt*10))/10);
            LCD.Write("\n");
        }

        //Move values from 'current' variables to 'last' variables to prep for next iteration
        lastTime = currTime;
        leftLastCounts = leftCurrentCounts;
        lPrevError = lError;
        rightLastCounts = rightCurrentCounts;
        rPrevError = rError;

        //Delay between iterations to make sure the motors have time to do something before adjusting again
        Sleep(10);
    }


    rightmotor.Stop();//stop motors
    leftmotor.Stop();
    Sleep(100);//wait for 50 ms to allow for some downtime so nothing weird happens
}


void turn(int direction, int angle){//direction 0 is left, direction 1 is right, angle is from 0 to 360 degrees
    int counts = angle*COUNTS_PER_DEGREE;//counts here is the equivalent of the angle times the defined constant COUNTS_PER_DEGREE

    right_encoder.ResetCounts();//Reset counts for both encoders
    left_encoder.ResetCounts();
    if (direction == 0){//right
        LCD.WriteLine("Turning Right");
        leftmotor.SetPercent(18);//Set turn percent to negative for the left motor, positive for the right motor to turn in place
        rightmotor.SetPercent(-18);
        while((/*left_encoder.Counts()*/left_encoder.Counts()) < counts){}//continue while the average encoder counts is less than counts
        rightmotor.Stop();//Stop both motors
        leftmotor.Stop();


    }
   else if (direction == 1){//left
        LCD.WriteLine("Turning Left");
        leftmotor.SetPercent(-18);//Set turn percent to negative for the right motor and positive for the left motor
        rightmotor.SetPercent(18);
        while(left_encoder.Counts() < counts){}//continue while average encoder counts < counts
        rightmotor.Stop();//Stop both motors
        leftmotor.Stop();
    }
    Sleep(100);//Sleep Check
}


void runDDR(){//red is 1, blue is 2. This function is to detect the light at DDR and act accordingly.
    //while(photoresis.Value()>.4){}
    LCD.WriteLine("Passed the while");
    if(photoresis.Value()<.3){                               //
        LCD.SetBackgroundColor(RED);
        turn(0,46);
        move(30,1);
        turn(0,46);
        move(30,-3);
        leftmotor.SetPercent(25);
        rightmotor.SetPercent(25);
        Sleep(5.5);
        move(30,5);
        turn(1, 94);
        move(30,7);
        turn(0,94);
    }else{                          //Blue
        LCD.SetBackgroundColor(BLUE);
        move(30, 6);
        turn(0,92);
        move(30,-4);
        leftmotor.SetPercent(25);
        rightmotor.SetPercent(25);
        Sleep(5.5);
    }
}

void coinDrop(){
   /* move(30.,2);
    turn(1,40);
    move(30.,15);
    turn(0,40);
    move(60.,45);
    turn(0,90);
    move(30.,6);
    turn(0,90);
    move(30.,2);
    turn(1,90);
    move(30.,4);
    turn(0,90);
    rightmotor.SetPercent(30.);
    leftmotor.SetPercent(30.);
    Sleep(3.0);
    rightmotor.Stop();
    leftmotor.Stop();
    */
   // servo.SetDegree(90);
    move(30.,9);
    turn(1,47);
   // checkHeading(0);
    move(30.,14);
    turn(0,93);
   // checkHeading(90);
    move(60.,20);
    move(30.,18);
    turn(1,55);
    move(30.,-13);
    turn(1,35);
    move(30.,-2);
    turn(0,90);
    rightmotor.SetPercent(30.);
    leftmotor.SetPercent(30.);
    Sleep(1250);
    rightmotor.Stop();
    leftmotor.Stop();
    tokenServo.SetDegree(0);
    Sleep(2.0);
    move(30.,4);
    turn(1,45);
    move(30.,10);
    turn(1,45);
    move(30.,2);
    turn(0,105);
    move(30.,8);
    turn(0,45);
    move(30.,3);
    turn(0,45);
    move(30.,10);
    turn(0,45);
    move(30.,10);
}



void foosball(){
    move(30.,9);//moving out from start
    turn(1,50);//turning towards right side
    //checkHeading(270);
    move(30.,13);//move to center for ramp
    turn(0,95);//turn towards ramp
   // checkHeading(0);
    move(35.,46.5);//move up ramp
   // move(30.,5);
   // turn(0,60);//turn left
   // move(30.,4);//move a bit
   // turn(0,30);//turn left
    turn(0,103);
    leftmotor.SetPercent(20.);
    rightmotor.SetPercent(20.);
    Sleep(3.0);
    leftmotor.Stop();
    rightmotor.Stop();
    move(20.,1);
    foosballArm.SetDegree(90);//Arm goes down
    Sleep(250);
    move(30.,4);//move with foosball
    Sleep(200);
    turn(0,5);
    Sleep(200);
    move(30.,4);
    Sleep(200);
    turn(0,3);
    Sleep(200);
    move(30.,1);
    foosballArm.SetDegree(170);//Arm goes up
    Sleep(500);
    turn(0,10);
    move(30.,8);
    turn(0,35);
    move(30.,5);
    turn(0,52.5);
    move(30.,60);


}



void instructionSet(){//This function is the instr  uction set that is a list of instructions
    foosballArm.SetDegree(170);
   // turn(0,45);
    //move(30.,60);//undershoot
   // turn(1,45);
   // Sleep(3.0);
    RPS.InitializeTouchMenu();

    startUp();


   // coinDrop();
    foosball();
   /* leftmotor.SetPercent(-32.5);
    rightmotor.SetPercent(-30.);
    Sleep(2.0);
    leftmotor.Stop();
    rightmotor.Stop();
    */


  //  turn(0,90);

    //This is the DDR code
   // move(30.,13.5);
   //runDDR();

    //move(30.,3)
   // turn(0,52);
   // move(80.,40);
 //  move(40.,60);


    //turn(1,67);

   // move(30.,14);
   // turn(0,98);
   // move(60.,51);
   // turn(0,93);
   // move(30.,25);

}

void runComp(){
    //reset servo

    LCD.WriteLine("Resetting servo");
    foosballArm.SetDegree(170);
    //Leave the starting zone and align for DDR
    move(30, 1.5);
    LCD.WriteLine("Turning towards DDR");
    turn(1, 43);
    checkHeading(0);
    //Do ddr things
    move(30.,12);
    LCD.WriteLine("Runing DDR");
    runDDR();
    //turn(0, 5);

    //Scoot up the ramp
    LCD.WriteLine("Going up the ramp");
    move(30,1);
    checkHeading(90);
    move(30, 29);
    LCD.WriteLine("Stabilizing...");
    Sleep(1.0);
    checkHeading(90);
    move(20, 29);
    //Align for foosball
    LCD.WriteLine("Aligning for foosball");
    move(30, -1);
    turn(0, 94);
    move(30, -3);
    move(30, 1);
    Sleep(1.0);
    //drop foosball arm
    foosballArm.SetDegree(80);

    //Carry foosball counters
    LCD.WriteLine("Moving foosball...");
    move(30, 5);
    turn(0, 5);
    move(30, 4);
    //Raise arm
    foosballArm.SetDegree(170);

    //Begin lever flipping
    LCD.WriteLine("Aligning for lever");
    move(20, 6);
    turn(0, 35);
    move(20, 3);
    turn(0, 10);
    //Finesse the line up

    //drop arm
    LCD.WriteLine("Lowering arm");
    foosballArm.SetDegree(100);
    Sleep(0.5);
    foosballArm.SetDegree(170);
    move(20, 3);
    //Reset arm

    //move(30., 4);
    Sleep(0.5);
    turn(0, 43);
    checkHeading(270);
    //Go down the ramp towards home
    LCD.WriteLine("Going home");
    move(20, 40);

    //Token time
    LCD.WriteLine("Going for token");
    turn(1, 30);                        //eligable for tweaks
    move(30, -10);
    Sleep(1.0);
    //Drop the token
    LCD.WriteLine("Dropping token");
    tokenServo.SetDegree(0);

    move(30, 30);
    LCD.WriteLine("Hitting the button");
    //Hit the button!
}

int main(void){//The main function is intentionally bare to make things easy to read
    RPS.InitializeTouchMenu();
    startUp();
    runComp();
}
