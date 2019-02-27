/********************************/
/*      Proteus Test Code       */
/*     OSU FEH Spring 2016      */
/*        Drew Phillips         */
/*    02/29/16  Version 3.0     */
/********************************/

/* Include preprocessor directives */
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHAccel.h>
#include <FEHBattery.h>
#include <FEHBuzzer.h>
#include <FEHRPS.h>
#include <FEHSD.h>
#include <string.h>
#include <stdio.h>

/* Define colors for parts of menus */
#define MENU_C WHITE
#define TEXT_C GOLD
#define SELT_C RED
#define SHOW_C BLUE
#define HI_C GREEN

/* Define menu number values */
#define MN_MENU 0
#define DC_MENU 1
#define SV_MENU 2
#define DI_MENU 3
#define AI_MENU 4
#define AC_MENU 5
#define TO_MENU 6
#define DO_MENU 7
#define RP_MENU 8

/* Define time for beep */
#define beep_t 10 // int milliseconds

/* Global variable to keep track of being initialized to RPS */
int RPS_init = 0;

/* Function to write input value as true or false to icon */
void WriteLogicValue(FEHIcon::Icon icon, int val)
{
    if (val)
    {
        icon.ChangeLabelString("True");
    }
    else
    {
        icon.ChangeLabelString("False");
    }
}

/* Main Menu */
int MNMenu()
{
    LCD.Clear(BLACK);

    /* Create icons for main menu */
    FEHIcon::Icon MAIN_T[1];
    char main_t_label[1][20] = {"PROTEUS TEST CODE"};
    FEHIcon::DrawIconArray(MAIN_T, 1, 1, 1, 201, 1, 1, main_t_label, HI_C, TEXT_C);
    MAIN_T[0].Select();

    FEHIcon::Icon MAIN[6];
    char main_label[8][20] = {"DC", "Servo", "Digital In", "Analog In", "Accel", "Touch", "Digital Out", "RPS"};
    FEHIcon::DrawIconArray(MAIN, 4, 2, 40, 20, 1, 1, main_label, MENU_C, TEXT_C);

    LCD.SetFontColor(HI_C);
    LCD.WriteAt("AHP V3.0", 220, 222);

    LCD.SetFontColor(TEXT_C);
    LCD.WriteAt("BATT:        V", 0, 222);

    Buzzer.Buzz(beep_t);

    int menu=MN_MENU, n;
    float x, y;
    float m = 0, bat_v = 0;

    while(menu==MN_MENU)
    {
        /* Display average battery voltage to screen */
        bat_v = ((bat_v*m)+Battery.Voltage());
        bat_v = bat_v/(++m);
        LCD.WriteAt(bat_v, 72, 222);
        if (LCD.Touch(&x, &y))
        {
            /* Check to see if a main menu icon has been touched */
            for (n=0; n<=7; n++)
            {
                if (MAIN[n].Pressed(x, y, 0))
                {
                    menu = n+1;
                    MAIN[n].WhilePressed(x, y);
                    break;
                }
            }
        }
    }
    return menu;
}

/* DC Motors Menu */
int DCMenu()
{
    /* Declare DC Motor ports */
    FEHMotor motor0 (FEHMotor::Motor0, 5.0);
    FEHMotor motor1 (FEHMotor::Motor1, 5.0);
    FEHMotor motor2 (FEHMotor::Motor2, 5.0);
    FEHMotor motor3 (FEHMotor::Motor3, 5.0);

    LCD.Clear(BLACK);

    /* Create icons for DC motors menu */
    FEHIcon::Icon DC_T[1];
    char dc_t_label[1][20] = {"DC Motors"};
    FEHIcon::DrawIconArray(DC_T, 1, 1, 1, 201, 1, 1, dc_t_label, MENU_C, TEXT_C);

    FEHIcon::Icon Back[1];
    char back_label[1][20] = {"<-"};
    FEHIcon::DrawIconArray(Back, 1, 1, 1, 201, 1, 260, back_label, MENU_C, TEXT_C);

    FEHIcon::Icon DC[4];
    char dc_labels[4][20] = {"Motor0", "Motor1", "Motor2", "Motor3"};
    FEHIcon::DrawIconArray(DC, 2, 2, 40, 1, 1, 60, dc_labels, SHOW_C, TEXT_C);

    FEHIcon::Icon Run[2];
    char run_labels[2][20] = {"F", "B"};
    FEHIcon::DrawIconArray(Run, 2, 1, 40, 1, 261, 1, run_labels, SELT_C, TEXT_C);

    Buzzer.Buzz(beep_t);

    int menu=DC_MENU, n, m;
    float x, y;
    int run[4] = {0, 0, 0, 0};

    while(menu==DC_MENU)
    {
        if (LCD.Touch(&x, &y))
        {
            /* Check to see if a motor icon has been touched */
            for (n=0; n<=3; n++)
            {
                if (DC[n].Pressed(x, y, 0))
                {
                    /* Select or deselect motor icon for running */
                    if (!run[n])
                    {
                        run[n] = 1;
                    }
                    else if (run[n])
                    {
                        run[n] = 0;
                    }
                    DC[n].WhilePressed(x, y);
                }
            }
            /* Check to see if a forward or backward run button has been touched */
            for (m=0; m<=1; m++)
            {
                if (Run[m].Pressed(x, y, 0))
                {
                    /* Run motors according to direction */
                    if (run[0]==1)
                    {
                        motor0.SetPercent((m*(-100))+50);
                    }
                    if (run[1]==1)
                    {
                        motor1.SetPercent((m*(-100))+50);
                    }
                    if (run[2]==1)
                    {
                        motor2.SetPercent((m*(-100))+50);
                    }
                    if (run[3]==1)
                    {
                        motor3.SetPercent((m*(-100))+50);
                    }
                    Run[m].WhilePressed(x, y);
                    Run[m].Deselect();
                    motor0.Stop();
                    motor1.Stop();
                    motor2.Stop();
                    motor3.Stop();
                }
            }
            /* If back button has been touched, go to main menu */
            if (Back[0].Pressed(x, y, 0))
            {
                Back[0].WhilePressed(x, y);
                menu = MN_MENU;
            }
        }
    }
    return menu;
}

/* Servo Motors Menu */
int SVMenu()
{
    /* Declare Servo Motor ports */
    FEHServo servo0 (FEHServo::Servo0);
    FEHServo servo1 (FEHServo::Servo1);
    FEHServo servo2 (FEHServo::Servo2);
    FEHServo servo3 (FEHServo::Servo3);
    FEHServo servo4 (FEHServo::Servo4);
    FEHServo servo5 (FEHServo::Servo5);
    FEHServo servo6 (FEHServo::Servo6);
    FEHServo servo7 (FEHServo::Servo7);

    LCD.Clear(BLACK);

    /* Create icons for servo motors menu */
    FEHIcon::Icon SV_T[1];
    char sv_t_label[1][20] = {"Servo Motors"};
    FEHIcon::DrawIconArray(SV_T, 1, 1, 1, 201, 1, 1, sv_t_label, MENU_C, TEXT_C);

    FEHIcon::Icon Back[1];
    char back_label[1][20] = {"<-"};
    FEHIcon::DrawIconArray(Back, 1, 1, 1, 201, 1, 260, back_label, MENU_C, TEXT_C);

    FEHIcon::Icon SV[8];
    char sv_labels[8][20] = {"Servo0", "Servo1", "Servo2", "Servo3", "Servo4", "Servo5", "Servo6", "Servo7"};
    FEHIcon::DrawIconArray(SV, 4, 2, 40, 1, 1, 60, sv_labels, SHOW_C, TEXT_C);

    FEHIcon::Icon Run[2];
    char run_labels[2][20] = {"F", "B"};
    FEHIcon::DrawIconArray(Run, 2, 1, 40, 41, 261, 1, run_labels, SELT_C, TEXT_C);

    FEHIcon::Icon Deg[1];
    char deg_labels[1][20] = {"0"};
    FEHIcon::DrawIconArray(Deg, 1, 1, 198, 4, 261, 1, deg_labels, SELT_C, TEXT_C);

    Buzzer.Buzz(beep_t);

    int menu=SV_MENU, n, m;
    float x, y;
    int deg=0;
    int run[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    while(menu==SV_MENU)
    {
        if (LCD.Touch(&x, &y))
        {
            /* Check to see if a servo motor icon has been touched */
            for (n=0; n<=7; n++)
            {
                if (SV[n].Pressed(x, y, 0))
                {
                    /* Select or deselect motor icon for running */
                    if (!run[n])
                    {
                        run[n] = 1;
                    }
                    else if (run[n])
                    {
                        run[n] = 0;
                    }
                    SV[n].WhilePressed(x, y);
                }
            }
            /* Check to see if the forward or backward run icons have been touched */
            for (m=0; m<=1; m++)
            {
                if (Run[m].Pressed(x, y, 0))
                {
                    while(Run[m].Pressed(x, y, 1))
                    {
                        /* While the run button is being touched, increase or decrease the servo angle within the limits */
                        LCD.Touch(&x, &y);
                        deg = deg+1-(m*2);
                        if (deg<0)
                        {
                            deg = 0;
                        }
                        else if (deg>180)
                        {
                            deg = 180;
                        }
                        Deg[0].ChangeLabelInt(deg);
                        if (run[0]==1)
                        {
                            servo0.SetDegree(deg);
                        }
                        if (run[1]==1)
                        {
                            servo1.SetDegree(deg);
                        }
                        if (run[2]==1)
                        {
                            servo2.SetDegree(deg);
                        }
                        if (run[3]==1)
                        {
                            servo3.SetDegree(deg);
                        }
                        if (run[4]==1)
                        {
                            servo4.SetDegree(deg);
                        }
                        if (run[5]==1)
                        {
                            servo5.SetDegree(deg);
                        }
                        if (run[6]==1)
                        {
                            servo6.SetDegree(deg);
                        }
                        if (run[7]==1)
                        {
                            servo7.SetDegree(deg);
                        }
                    }
                    Run[m].Deselect();
                }
            }
            /* If back button has been touched, go to main menu */
            if (Back[0].Pressed(x, y, 0))
            {
                Back[0].WhilePressed(x, y);
                menu = MN_MENU;
            }
        }
    }
    return menu;
}

/* Digital Input Menu */
int DIMenu()
{
    /* Declare ports for digital input */
    DigitalInputPin di00 (FEHIO::P0_0);
    DigitalInputPin di01 (FEHIO::P0_1);
    DigitalInputPin di02 (FEHIO::P0_2);
    DigitalInputPin di03 (FEHIO::P0_3);
    DigitalInputPin di04 (FEHIO::P0_4);
    DigitalInputPin di05 (FEHIO::P0_5);
    DigitalInputPin di06 (FEHIO::P0_6);
    DigitalInputPin di07 (FEHIO::P0_7);
    DigitalInputPin di10 (FEHIO::P1_0);
    DigitalInputPin di11 (FEHIO::P1_1);
    DigitalInputPin di12 (FEHIO::P1_2);
    DigitalInputPin di13 (FEHIO::P1_3);
    DigitalInputPin di14 (FEHIO::P1_4);
    DigitalInputPin di15 (FEHIO::P1_5);
    DigitalInputPin di16 (FEHIO::P1_6);
    DigitalInputPin di17 (FEHIO::P1_7);
    DigitalInputPin di20 (FEHIO::P2_0);
    DigitalInputPin di21 (FEHIO::P2_1);
    DigitalInputPin di22 (FEHIO::P2_2);
    DigitalInputPin di23 (FEHIO::P2_3);
    DigitalInputPin di24 (FEHIO::P2_4);
    DigitalInputPin di25 (FEHIO::P2_5);
    DigitalInputPin di26 (FEHIO::P2_6);
    DigitalInputPin di27 (FEHIO::P2_7);
    DigitalInputPin di30 (FEHIO::P3_0);
    DigitalInputPin di31 (FEHIO::P3_1);
    DigitalInputPin di32 (FEHIO::P3_2);
    DigitalInputPin di33 (FEHIO::P3_3);
    DigitalInputPin di34 (FEHIO::P3_4);
    DigitalInputPin di35 (FEHIO::P3_5);
    DigitalInputPin di36 (FEHIO::P3_6);
    DigitalInputPin di37 (FEHIO::P3_7);

    LCD.Clear(BLACK);

    /* Create digital input menu icons */
    FEHIcon::Icon DI_T[1];
    char di_t_label[1][20] = {"Digital Input"};
    FEHIcon::DrawIconArray(DI_T, 1, 1, 1, 201, 1, 1, di_t_label, MENU_C, TEXT_C);

    FEHIcon::Icon Back[1];
    char back_label[1][20] = {"<-"};
    FEHIcon::DrawIconArray(Back, 1, 1, 1, 201, 1, 260, back_label, MENU_C, TEXT_C);

    FEHIcon::Icon BANKS[4];
    char banks_label[4][20] = {"Bank0", "Bank1", "Bank2", "Bank3"};
    FEHIcon::DrawIconArray(BANKS, 1, 4, 40, 161, 1, 1, banks_label, SELT_C, TEXT_C);

    FEHIcon::Icon DI_B0[16];
    char di_b0_labels[16][20] = {"P0_0", "P0_1", "P0_2", "P0_3", " ", " ", " ", " ", "P0_4", "P0_5", "P0_6", "P0_7", " ", " ", " ", " "};

    FEHIcon::Icon DI_B1[16];
    char di_b1_labels[16][20] = {"P1_0", "P1_1", "P1_2", "P1_3", " ", " ", " ", " ", "P1_4", "P1_5", "P1_6", "P1_7", " ", " ", " ", " "};

    FEHIcon::Icon DI_B2[16];
    char di_b2_labels[16][20] = {"P2_0", "P2_1", "P2_2", "P2_3", " ", " ", " ", " ", "P2_4", "P2_5", "P2_6", "P2_7", " ", " ", " ", " "};

    FEHIcon::Icon DI_B3[16];
    char di_b3_labels[16][20] = {"P3_0", "P3_1", "P3_2", "P3_3", " ", " ", " ", " ", "P3_4", "P3_5", "P3_6", "P3_7", " ", " ", " ", " "};

    Buzzer.Buzz(beep_t);

    int menu=DI_MENU, bank=0, bank_i, n;
    float x, y;

    while(menu==DI_MENU)
    {
        /* Draw selected bank's ports */
        switch (bank)
        {
        case 0:
            FEHIcon::DrawIconArray(DI_B0, 4, 4, 80, 1, 1, 1, di_b0_labels, SHOW_C, TEXT_C);
            break;
        case 1:
            FEHIcon::DrawIconArray(DI_B1, 4, 4, 80, 1, 1, 1, di_b1_labels, SHOW_C, TEXT_C);
            break;
        case 2:
            FEHIcon::DrawIconArray(DI_B2, 4, 4, 80, 1, 1, 1, di_b2_labels, SHOW_C, TEXT_C);
            break;
        case 3:
            FEHIcon::DrawIconArray(DI_B3, 4, 4, 80, 1, 1, 1, di_b3_labels, SHOW_C, TEXT_C);
            break;
        }
        bank_i = bank;
        while(bank==bank_i)
        {
            /* For each bank, display the digital input values */
            if (bank==0)
            {
                WriteLogicValue(DI_B0[4], di00.Value());
                WriteLogicValue(DI_B0[5], di01.Value());
                WriteLogicValue(DI_B0[6], di02.Value());
                WriteLogicValue(DI_B0[7], di03.Value());
                WriteLogicValue(DI_B0[12], di04.Value());
                WriteLogicValue(DI_B0[13], di05.Value());
                WriteLogicValue(DI_B0[14], di06.Value());
                WriteLogicValue(DI_B0[15], di07.Value());
            }
            else if (bank==1)
            {
                WriteLogicValue(DI_B1[4], di10.Value());
                WriteLogicValue(DI_B1[5], di11.Value());
                WriteLogicValue(DI_B1[6], di12.Value());
                WriteLogicValue(DI_B1[7], di13.Value());
                WriteLogicValue(DI_B1[12], di14.Value());
                WriteLogicValue(DI_B1[13], di15.Value());
                WriteLogicValue(DI_B1[14], di16.Value());
                WriteLogicValue(DI_B1[15], di17.Value());
            }
            else if (bank==2)
            {
                WriteLogicValue(DI_B2[4], di20.Value());
                WriteLogicValue(DI_B2[5], di21.Value());
                WriteLogicValue(DI_B2[6], di22.Value());
                WriteLogicValue(DI_B2[7], di23.Value());
                WriteLogicValue(DI_B2[12], di24.Value());
                WriteLogicValue(DI_B2[13], di25.Value());
                WriteLogicValue(DI_B2[14], di26.Value());
                WriteLogicValue(DI_B2[15], di27.Value());
            }
            else if (bank==3)
            {
                WriteLogicValue(DI_B3[4], di30.Value());
                WriteLogicValue(DI_B3[5], di31.Value());
                WriteLogicValue(DI_B3[6], di32.Value());
                WriteLogicValue(DI_B3[7], di33.Value());
                WriteLogicValue(DI_B3[12], di34.Value());
                WriteLogicValue(DI_B3[13], di35.Value());
                WriteLogicValue(DI_B3[14], di36.Value());
                WriteLogicValue(DI_B3[15], di37.Value());
            }
            if (LCD.Touch(&x, &y))
            {
                /* Check to see if any of the banks icons have been touched */
                for (n=0; n<=3; n++)
                {
                    if (BANKS[n].Pressed(x, y, 0))
                    {
                        /* Change selected bank number to update screen */
                        BANKS[n].WhilePressed(x, y);
                        BANKS[n].Deselect();
                        bank = n;
                    }
                }
                /* If back button has been touched, go to main menu */
                if (Back[0].Pressed(x, y, 0))
                {
                    Back[0].WhilePressed(x, y);
                    menu = MN_MENU;
                    bank = -1;
                }
            }
        }
    }
    return menu;
}

/* Analog Input Menu */
int AIMenu()
{
    /* Declare ports for digital input */  // Trust me, you have to declare them as digital and analog inputs in order for this to work
    DigitalInputPin di00 (FEHIO::P0_0);
    DigitalInputPin di01 (FEHIO::P0_1);
    DigitalInputPin di02 (FEHIO::P0_2);
    DigitalInputPin di03 (FEHIO::P0_3);
    DigitalInputPin di04 (FEHIO::P0_4);
    DigitalInputPin di05 (FEHIO::P0_5);
    DigitalInputPin di06 (FEHIO::P0_6);
    DigitalInputPin di07 (FEHIO::P0_7);
    DigitalInputPin di10 (FEHIO::P1_0);
    DigitalInputPin di11 (FEHIO::P1_1);
    DigitalInputPin di12 (FEHIO::P1_2);
    DigitalInputPin di13 (FEHIO::P1_3);
    DigitalInputPin di14 (FEHIO::P1_4);
    DigitalInputPin di15 (FEHIO::P1_5);
    DigitalInputPin di16 (FEHIO::P1_6);
    DigitalInputPin di17 (FEHIO::P1_7);
    DigitalInputPin di20 (FEHIO::P2_0);
    DigitalInputPin di21 (FEHIO::P2_1);
    DigitalInputPin di22 (FEHIO::P2_2);
    DigitalInputPin di23 (FEHIO::P2_3);
    DigitalInputPin di24 (FEHIO::P2_4);
    DigitalInputPin di25 (FEHIO::P2_5);
    DigitalInputPin di26 (FEHIO::P2_6);
    DigitalInputPin di27 (FEHIO::P2_7);
    DigitalInputPin di30 (FEHIO::P3_0);
    DigitalInputPin di31 (FEHIO::P3_1);
    DigitalInputPin di32 (FEHIO::P3_2);
    DigitalInputPin di33 (FEHIO::P3_3);
    DigitalInputPin di34 (FEHIO::P3_4);
    DigitalInputPin di35 (FEHIO::P3_5);
    DigitalInputPin di36 (FEHIO::P3_6);
    DigitalInputPin di37 (FEHIO::P3_7);

    /* Declare ports for analog input */
    AnalogInputPin ai00 (FEHIO::P0_0);
    AnalogInputPin ai01 (FEHIO::P0_1);
    AnalogInputPin ai02 (FEHIO::P0_2);
    AnalogInputPin ai03 (FEHIO::P0_3);
    AnalogInputPin ai04 (FEHIO::P0_4);
    AnalogInputPin ai05 (FEHIO::P0_5);
    AnalogInputPin ai06 (FEHIO::P0_6);
    AnalogInputPin ai07 (FEHIO::P0_7);
    AnalogInputPin ai10 (FEHIO::P1_0);
    AnalogInputPin ai11 (FEHIO::P1_1);
    AnalogInputPin ai12 (FEHIO::P1_2);
    AnalogInputPin ai13 (FEHIO::P1_3);
    AnalogInputPin ai14 (FEHIO::P1_4);
    AnalogInputPin ai15 (FEHIO::P1_5);
    AnalogInputPin ai16 (FEHIO::P1_6);
    AnalogInputPin ai17 (FEHIO::P1_7);
    AnalogInputPin ai20 (FEHIO::P2_0);
    AnalogInputPin ai21 (FEHIO::P2_1);
    AnalogInputPin ai22 (FEHIO::P2_2);
    AnalogInputPin ai23 (FEHIO::P2_3);
    AnalogInputPin ai24 (FEHIO::P2_4);
    AnalogInputPin ai25 (FEHIO::P2_5);
    AnalogInputPin ai26 (FEHIO::P2_6);
    AnalogInputPin ai27 (FEHIO::P2_7);
    AnalogInputPin ai30 (FEHIO::P3_0);
    AnalogInputPin ai31 (FEHIO::P3_1);
    AnalogInputPin ai32 (FEHIO::P3_2);
    AnalogInputPin ai33 (FEHIO::P3_3);
    AnalogInputPin ai34 (FEHIO::P3_4);
    AnalogInputPin ai35 (FEHIO::P3_5);
    AnalogInputPin ai36 (FEHIO::P3_6);
    AnalogInputPin ai37 (FEHIO::P3_7);

    LCD.Clear(BLACK);

    /* Create analog input menu icons */
    FEHIcon::Icon AI_T[1];
    char ai_t_label[1][20] = {"Analog Input"};
    FEHIcon::DrawIconArray(AI_T, 1, 1, 1, 201, 1, 1, ai_t_label, MENU_C, TEXT_C);

    FEHIcon::Icon Back[1];
    char back_label[1][20] = {"<-"};
    FEHIcon::DrawIconArray(Back, 1, 1, 1, 201, 1, 260, back_label, MENU_C, TEXT_C);

    FEHIcon::Icon BANKS[4];
    char banks_label[4][20] = {"Bank0", "Bank1", "Bank2", "Bank3"};
    FEHIcon::DrawIconArray(BANKS, 1, 4, 40, 161, 1, 1, banks_label, SELT_C, TEXT_C);

    FEHIcon::Icon AI_B0[16];
    char AI_b0_labels[16][20] = {"P0_0", "P0_1", "P0_2", "P0_3", " ", " ", " ", " ", "P0_4", "P0_5", "P0_6", "P0_7", " ", " ", " ", " "};

    FEHIcon::Icon AI_B1[16];
    char AI_b1_labels[16][20] = {"P1_0", "P1_1", "P1_2", "P1_3", " ", " ", " ", " ", "P1_4", "P1_5", "P1_6", "P1_7", " ", " ", " ", " "};

    FEHIcon::Icon AI_B2[16];
    char AI_b2_labels[16][20] = {"P2_0", "P2_1", "P2_2", "P2_3", " ", " ", " ", " ", "P2_4", "P2_5", "P2_6", "P2_7", " ", " ", " ", " "};

    FEHIcon::Icon AI_B3[16];
    char AI_b3_labels[16][20] = {"P3_0", "P3_1", "P3_2", "P3_3", " ", " ", " ", " ", "P3_4", "P3_5", "P3_6", "P3_7", " ", " ", " ", " "};

    Buzzer.Buzz(beep_t);

    int menu=AI_MENU, bank=0, bank_i, n;
    float x, y;

    while(menu==AI_MENU)
    {
        /* Draw selected bank's ports */
        switch (bank)
        {
        case 0:
            FEHIcon::DrawIconArray(AI_B0, 4, 4, 80, 1, 1, 1, AI_b0_labels, SHOW_C, TEXT_C);
            break;
        case 1:
            FEHIcon::DrawIconArray(AI_B1, 4, 4, 80, 1, 1, 1, AI_b1_labels, SHOW_C, TEXT_C);
            break;
        case 2:
            FEHIcon::DrawIconArray(AI_B2, 4, 4, 80, 1, 1, 1, AI_b2_labels, SHOW_C, TEXT_C);
            break;
        case 3:
            FEHIcon::DrawIconArray(AI_B3, 4, 4, 80, 1, 1, 1, AI_b3_labels, SHOW_C, TEXT_C);
            break;
        }
        bank_i = bank;
        while(bank==bank_i)
        {
            /* Draw each bank's analog input values */
            if (bank==0)
            {
                AI_B0[4].ChangeLabelFloat(ai00.Value());
                AI_B0[5].ChangeLabelFloat(ai01.Value());
                AI_B0[6].ChangeLabelFloat(ai02.Value());
                AI_B0[7].ChangeLabelFloat(ai03.Value());
                AI_B0[12].ChangeLabelFloat(ai04.Value());
                AI_B0[13].ChangeLabelFloat(ai05.Value());
                AI_B0[14].ChangeLabelFloat(ai06.Value());
                AI_B0[15].ChangeLabelFloat(ai07.Value());
            }
            else if (bank==1)
            {
                AI_B1[4].ChangeLabelFloat(ai10.Value());
                AI_B1[5].ChangeLabelFloat(ai11.Value());
                AI_B1[6].ChangeLabelFloat(ai12.Value());
                AI_B1[7].ChangeLabelFloat(ai13.Value());
                AI_B1[12].ChangeLabelFloat(ai14.Value());
                AI_B1[13].ChangeLabelFloat(ai15.Value());
                AI_B1[14].ChangeLabelFloat(ai16.Value());
                AI_B1[15].ChangeLabelFloat(ai17.Value());
            }
            else if (bank==2)
            {
                AI_B2[4].ChangeLabelFloat(ai20.Value());
                AI_B2[5].ChangeLabelFloat(ai21.Value());
                AI_B2[6].ChangeLabelFloat(ai22.Value());
                AI_B2[7].ChangeLabelFloat(ai23.Value());
                AI_B2[12].ChangeLabelFloat(ai24.Value());
                AI_B2[13].ChangeLabelFloat(ai25.Value());
                AI_B2[14].ChangeLabelFloat(ai26.Value());
                AI_B2[15].ChangeLabelFloat(ai27.Value());
            }
            else if (bank==3)
            {
                AI_B3[4].ChangeLabelFloat(ai30.Value());
                AI_B3[5].ChangeLabelFloat(ai31.Value());
                AI_B3[6].ChangeLabelFloat(ai32.Value());
                AI_B3[7].ChangeLabelFloat(ai33.Value());
                AI_B3[12].ChangeLabelFloat(ai34.Value());
                AI_B3[13].ChangeLabelFloat(ai35.Value());
                AI_B3[14].ChangeLabelFloat(ai36.Value());
                AI_B3[15].ChangeLabelFloat(ai37.Value());
            }
            if (LCD.Touch(&x, &y))
            {
                /* Check to see if any of the banks icons have been touched */
                for (n=0; n<=3; n++)
                {
                    if (BANKS[n].Pressed(x, y, 0))
                    {
                        /* Change selected bank to update */
                        BANKS[n].WhilePressed(x, y);
                        BANKS[n].Deselect();
                        bank = n;
                    }
                }
                /* If back button has been touched, go to main menu */
                if (Back[0].Pressed(x, y, 0))
                {
                    Back[0].WhilePressed(x, y);
                    menu = MN_MENU;
                    bank = -1;
                }
            }
        }
    }
    return menu;
}

/* Accelerometer Menu */
int ACMenu()
{
    LCD.Clear(BLACK);

    /* Create accelerometer menu icons */
    FEHIcon::Icon AC_T[1];
    char ac_t_label[1][20] = {"Accelerometer"};
    FEHIcon::DrawIconArray(AC_T, 1, 1, 1, 201, 1, 1, ac_t_label, MENU_C, TEXT_C);

    FEHIcon::Icon Back[1];
    char back_label[1][20] = {"<-"};
    FEHIcon::DrawIconArray(Back, 1, 1, 1, 201, 1, 260, back_label, MENU_C, TEXT_C);

    FEHIcon::Icon AC_XYZ[3];
    char ac_xyz_label[3][20] = {"X", "Y", "Z"};
    FEHIcon::DrawIconArray(AC_XYZ, 1, 3, 40, 120, 1, 1, ac_xyz_label, SHOW_C, TEXT_C);

    FEHIcon::Icon AC_VAL[3];
    char ac_val_label[3][20] = {"", "", ""};
    FEHIcon::DrawIconArray(AC_VAL, 1, 3, 120, 40, 1, 1, ac_val_label, SHOW_C, TEXT_C);

    FEHIcon::Icon AC_CAL[1];
    char ac_cal_label[1][20] = {"Calibrate"};
    FEHIcon::DrawIconArray(AC_CAL, 1, 1, 201, 2, 1, 1, ac_cal_label, SELT_C, TEXT_C);

    Buzzer.Buzz(beep_t);

    int menu=AC_MENU;
    float x, y, xo=0, yo=0, zo=0;

    while(menu==AC_MENU)
    {
        /* Update accelerometer readings taking into account an offset (calibration) */
        AC_VAL[0].ChangeLabelFloat(Accel.X()-xo);
        AC_VAL[1].ChangeLabelFloat(Accel.Y()-yo);
        AC_VAL[2].ChangeLabelFloat(Accel.Z()-zo);
        if (LCD.Touch(&x, &y))
        {
            /* Check to see if the calibrate icon has been touched */
            if (AC_CAL[0].Pressed(x, y, 0))
            {
                /* Set offsets to current accelerometer data to calibrate to zeros at that orientation */
                AC_CAL[0].WhilePressed(x, y);
                AC_CAL[0].Deselect();
                xo = Accel.X();
                yo = Accel.Y();
                zo = Accel.Z();
            }
            /* If back button has been touched, go to main menu */
            if (Back[0].Pressed(x, y, 0))
            {
                Back[0].WhilePressed(x, y);
                menu = MN_MENU;
            }
        }
    }
    return menu;
}

/* Touch Menu */
int TOMenu()
{
    LCD.Clear(BLACK);

    /* Create touch menu icons */
    FEHIcon::Icon TO_T[1];
    char to_t_label[1][20] = {"Touch Screen"};
    FEHIcon::DrawIconArray(TO_T, 1, 1, 1, 201, 1, 1, to_t_label, MENU_C, TEXT_C);

    FEHIcon::Icon Back[1];
    char back_label[1][20] = {"<-"};
    FEHIcon::DrawIconArray(Back, 1, 1, 1, 201, 1, 260, back_label, MENU_C, TEXT_C);

    FEHIcon::Icon TO[4];
    char to_label[4][20] = {"X", " ", "Y", " "};
    FEHIcon::DrawIconArray(TO, 2, 2, 40, 161, 1, 1, to_label, SHOW_C, TEXT_C);

    FEHIcon::Icon TO_SW[1];
    char to_sw_label[1][20] = {"SWITCH"};
    FEHIcon::DrawIconArray(TO_SW, 1, 1, 80, 121, 1, 1, to_sw_label, SELT_C, TEXT_C);

    Buzzer.Buzz(beep_t);

    int menu=TO_MENU, side=0;
    float x, y;

    /* Sleep briefly to avoid touching after selecting menu */
    Sleep(100);

    while(menu==TO_MENU)
    {
        if (LCD.Touch(&x, &y))
        {
            /* Update x and y of touched locations */
            TO[1].ChangeLabelFloat(x);
            TO[3].ChangeLabelFloat(y);
            if (TO_SW[0].Pressed(x, y, 0))
            {
                /* If switch is touched, switch the side of the screen with the icons to allow access to both halfs of the touch screen for testing */
                TO_SW[0].WhilePressed(x, y);
                TO_SW[0].Deselect();
                if (side==0)
                {
                    LCD.Clear(BLACK);
                    FEHIcon::DrawIconArray(TO_T, 1, 1, 121, 80, 1, 1, to_t_label, MENU_C, TEXT_C);
                    FEHIcon::DrawIconArray(Back, 1, 1, 121, 81, 1, 260, back_label, MENU_C, TEXT_C);
                    FEHIcon::DrawIconArray(TO, 2, 2, 161, 40, 1, 1, to_label, SHOW_C, TEXT_C);
                    FEHIcon::DrawIconArray(TO_SW, 1, 1, 201, 2, 1, 1, to_sw_label, SELT_C, TEXT_C);
                    side = 1;
                }
                else if (side==1)
                {
                    LCD.Clear(BLACK);
                    FEHIcon::DrawIconArray(TO_T, 1, 1, 1, 201, 1, 1, to_t_label, MENU_C, TEXT_C);
                    FEHIcon::DrawIconArray(Back, 1, 1, 1, 201, 1, 260, back_label, MENU_C, TEXT_C);
                    FEHIcon::DrawIconArray(TO, 2, 2, 40, 161, 1, 1, to_label, SHOW_C, TEXT_C);
                    FEHIcon::DrawIconArray(TO_SW, 1, 1, 80, 121, 1, 1, to_sw_label, SELT_C, TEXT_C);
                    side = 0;
                }
            }
            /* If back button has been touched, go to main menu */
            if (Back[0].Pressed(x, y, 0))
            {
                Back[0].WhilePressed(x, y);
                menu = MN_MENU;
            }
        }
    }
    return menu;
}

/* Digital Output Menu */
int DOMenu()
{
    /* Declare ports for digital output */
    DigitalOutputPin do00 (FEHIO::P0_0);
    DigitalOutputPin do01 (FEHIO::P0_1);
    DigitalOutputPin do02 (FEHIO::P0_2);
    DigitalOutputPin do03 (FEHIO::P0_3);
    DigitalOutputPin do04 (FEHIO::P0_4);
    DigitalOutputPin do05 (FEHIO::P0_5);
    DigitalOutputPin do06 (FEHIO::P0_6);
    DigitalOutputPin do07 (FEHIO::P0_7);
    DigitalOutputPin do10 (FEHIO::P1_0);
    DigitalOutputPin do11 (FEHIO::P1_1);
    DigitalOutputPin do12 (FEHIO::P1_2);
    DigitalOutputPin do13 (FEHIO::P1_3);
    DigitalOutputPin do14 (FEHIO::P1_4);
    DigitalOutputPin do15 (FEHIO::P1_5);
    DigitalOutputPin do16 (FEHIO::P1_6);
    DigitalOutputPin do17 (FEHIO::P1_7);
    DigitalOutputPin do20 (FEHIO::P2_0);
    DigitalOutputPin do21 (FEHIO::P2_1);
    DigitalOutputPin do22 (FEHIO::P2_2);
    DigitalOutputPin do23 (FEHIO::P2_3);
    DigitalOutputPin do24 (FEHIO::P2_4);
    DigitalOutputPin do25 (FEHIO::P2_5);
    DigitalOutputPin do26 (FEHIO::P2_6);
    DigitalOutputPin do27 (FEHIO::P2_7);
    DigitalOutputPin do30 (FEHIO::P3_0);
    DigitalOutputPin do31 (FEHIO::P3_1);
    DigitalOutputPin do32 (FEHIO::P3_2);
    DigitalOutputPin do33 (FEHIO::P3_3);
    DigitalOutputPin do34 (FEHIO::P3_4);
    DigitalOutputPin do35 (FEHIO::P3_5);
    DigitalOutputPin do36 (FEHIO::P3_6);
    DigitalOutputPin do37 (FEHIO::P3_7);

    LCD.Clear(BLACK);

    /* Create digital output menu icons */
    FEHIcon::Icon DO_T[1];
    char do_t_label[1][20] = {"Digital Output"};
    FEHIcon::DrawIconArray(DO_T, 1, 1, 1, 201, 1, 1, do_t_label, MENU_C, TEXT_C);

    FEHIcon::Icon Back[1];
    char back_label[1][20] = {"<-"};
    FEHIcon::DrawIconArray(Back, 1, 1, 1, 201, 1, 260, back_label, MENU_C, TEXT_C);

    FEHIcon::Icon BANKS[4];
    char banks_label[4][20] = {"Bank0", "Bank1", "Bank2", "Bank3"};
    FEHIcon::DrawIconArray(BANKS, 1, 4, 40, 161, 1, 1, banks_label, SELT_C, TEXT_C);

    FEHIcon::Icon OUT[1];
    char out_label[1][20] = {"Toggle"};
    FEHIcon::DrawIconArray(OUT, 1, 1, 161, 3, 1, 1, out_label, SELT_C, TEXT_C);

    FEHIcon::Icon DO_B0[8];
    char do_b0_labels[8][20] = {"P0_0", "P0_1", "P0_2", "P0_3", "P0_4", "P0_5", "P0_6", "P0_7"};

    FEHIcon::Icon DO_B1[8];
    char do_b1_labels[8][20] = {"P1_0", "P1_1", "P1_2", "P1_3", "P1_4", "P1_5", "P1_6", "P1_7"};

    FEHIcon::Icon DO_B2[8];
    char do_b2_labels[8][20] = {"P2_0", "P2_1", "P2_2", "P2_3", "P2_4", "P2_5", "P2_6", "P2_7"};

    FEHIcon::Icon DO_B3[8];
    char do_b3_labels[8][20] = {"P3_0", "P3_1", "P3_2", "P3_3", "P3_4", "P3_5", "P3_6", "P3_7"};

    Buzzer.Buzz(beep_t);

    int menu=DO_MENU, bank=0, bank_i, n, m;
    float x, y;

    int output[4][8] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    while(menu==DO_MENU)
    {
        /* Deselect all selected ports when switching banks for visibility */
        for (n=0; n<=7; n++)
        {
            if (output[0][n]==1)
            {
                DO_B0[n].Deselect();
            }
        }
        for (n=0; n<=7; n++)
        {
            if (output[1][n]==1)
            {
                DO_B1[n].Deselect();
            }
        }
        for (n=0; n<=7; n++)
        {
            if (output[2][n]==1)
            {
                DO_B2[n].Deselect();
            }
        }
        for (n=0; n<=7; n++)
        {
            if (output[3][n]==1)
            {
                DO_B3[n].Deselect();
            }
        }
        /* Draw each bank's ports and re-draw selected ports if selected earlier */
        switch (bank)
        {
        case 0:
            FEHIcon::DrawIconArray(DO_B0, 2, 4, 80, 80, 1, 1, do_b0_labels, SHOW_C, TEXT_C);
            for (n=0; n<=7; n++)
            {
                if (output[0][n]==1)
                {
                    DO_B0[n].Select();
                }
            }
            break;
        case 1:
            FEHIcon::DrawIconArray(DO_B1, 2, 4, 80, 80, 1, 1, do_b1_labels, SHOW_C, TEXT_C);
            for (n=0; n<=7; n++)
            {
                if (output[1][n]==1)
                {
                    DO_B0[n].Select();
                }
            }
            break;
        case 2:
            FEHIcon::DrawIconArray(DO_B2, 2, 4, 80, 80, 1, 1, do_b2_labels, SHOW_C, TEXT_C);
            for (n=0; n<=7; n++)
            {
                if (output[2][n]==1)
                {
                    DO_B0[n].Select();
                }
            }
            break;
        case 3:
            FEHIcon::DrawIconArray(DO_B3, 2, 4, 80, 80, 1, 1, do_b3_labels, SHOW_C, TEXT_C);
            for (n=0; n<=7; n++)
            {
                if (output[3][n]==1)
                {
                    DO_B0[n].Select();
                }
            }
            break;
        }
        bank_i = bank;
        while(bank==bank_i)
        {
            /* Check to see if each bank's port icons are touched and set the output bit for each port to be able to toggle or not */
            if (bank==0)
            {
                if (LCD.Touch(&x, &y))
                {
                    for (m=0; m<=7; m++)
                    {
                        if (DO_B0[m].Pressed(x, y, 0))
                        {
                            DO_B0[m].WhilePressed(x, y);
                            if (output[0][m]==0)
                            {
                                output[0][m] = 1;
                            }
                            else if (output[0][m]==1)
                            {
                                output[0][m] = 0;
                            }
                        }
                    }
                }
            }
            else if (bank==1)
            {
                if (LCD.Touch(&x, &y))
                {
                    for (m=0; m<=7; m++)
                    {
                        if (DO_B1[m].Pressed(x, y, 0))
                        {
                            DO_B1[m].WhilePressed(x, y);
                            if (output[1][m]==0)
                            {
                                output[1][m] = 1;
                            }
                            else if (output[1][m]==1)
                            {
                                output[1][m] = 0;
                            }
                        }
                    }
                }
            }
            else if (bank==2)
            {
                if (LCD.Touch(&x, &y))
                {
                    for (m=0; m<=7; m++)
                    {
                        if (DO_B2[m].Pressed(x, y, 0))
                        {
                            DO_B2[m].WhilePressed(x, y);
                            if (output[2][m]==0)
                            {
                                output[2][m] = 1;
                            }
                            else if (output[2][m]==1)
                            {
                                output[2][m] = 0;
                            }
                        }
                    }
                }
            }
            else if (bank==3)
            {
                if (LCD.Touch(&x, &y))
                {
                    for (m=0; m<=7; m++)
                    {
                        if (DO_B3[m].Pressed(x, y, 0))
                        {
                            DO_B3[m].WhilePressed(x, y);
                            if (output[3][m]==0)
                            {
                                output[3][m] = 1;
                            }
                            else if (output[3][m]==1)
                            {
                                output[3][m] = 0;
                            }
                        }
                    }
                }
            }
            if (LCD.Touch(&x, &y))
            {
                /* Check to see if any of the banks icons have been touched */
                for (n=0; n<=3; n++)
                {
                    if (BANKS[n].Pressed(x, y, 0))
                    {
                        /* Change selected bank to update */
                        BANKS[n].WhilePressed(x, y);
                        BANKS[n].Deselect();
                        bank = n;
                    }
                }
                /* Check to see if the toggle icon has been touched */
                if (OUT[0].Pressed(x, y, 0))
                {
                    OUT[0].WhilePressed(x, y);
                    OUT[0].Deselect();
                    /* Toggle the state of all selected output ports */
                    if (output[0][0]==1)
                    {
                        do00.Toggle();
                    }
                    if (output[0][1]==1)
                    {
                        do01.Toggle();
                    }
                    if (output[0][2]==1)
                    {
                        do02.Toggle();
                    }
                    if (output[0][3]==1)
                    {
                        do03.Toggle();
                    }
                    if (output[0][4]==1)
                    {
                        do04.Toggle();
                    }
                    if (output[0][5]==1)
                    {
                        do05.Toggle();
                    }
                    if (output[0][6]==1)
                    {
                        do06.Toggle();
                    }
                    if (output[0][7]==1)
                    {
                        do07.Toggle();
                    }
                    if (output[1][0]==1)
                    {
                        do10.Toggle();
                    }
                    if (output[1][1]==1)
                    {
                        do11.Toggle();
                    }
                    if (output[1][2]==1)
                    {
                        do12.Toggle();
                    }
                    if (output[1][3]==1)
                    {
                        do13.Toggle();
                    }
                    if (output[1][4]==1)
                    {
                        do14.Toggle();
                    }
                    if (output[1][5]==1)
                    {
                        do15.Toggle();
                    }
                    if (output[1][6]==1)
                    {
                        do16.Toggle();
                    }
                    if (output[1][7]==1)
                    {
                        do17.Toggle();
                    }
                    if (output[2][0]==1)
                    {
                        do20.Toggle();
                    }
                    if (output[2][1]==1)
                    {
                        do21.Toggle();
                    }
                    if (output[2][2]==1)
                    {
                        do22.Toggle();
                    }
                    if (output[2][3]==1)
                    {
                        do23.Toggle();
                    }
                    if (output[2][4]==1)
                    {
                        do24.Toggle();
                    }
                    if (output[2][5]==1)
                    {
                        do25.Toggle();
                    }
                    if (output[2][6]==1)
                    {
                        do26.Toggle();
                    }
                    if (output[2][7]==1)
                    {
                        do27.Toggle();
                    }
                    if (output[3][0]==1)
                    {
                        do30.Toggle();
                    }
                    if (output[3][1]==1)
                    {
                        do31.Toggle();
                    }
                    if (output[3][2]==1)
                    {
                        do32.Toggle();
                    }
                    if (output[3][3]==1)
                    {
                        do33.Toggle();
                    }
                    if (output[3][4]==1)
                    {
                        do34.Toggle();
                    }
                    if (output[3][5]==1)
                    {
                        do35.Toggle();
                    }
                    if (output[3][6]==1)
                    {
                        do36.Toggle();
                    }
                    if (output[3][7]==1)
                    {
                        do37.Toggle();
                    }
                }
                /* If back button has been touched, go to main menu */
                if (Back[0].Pressed(x, y, 0))
                {
                    Back[0].WhilePressed(x, y);
                    menu = MN_MENU;
                    bank = -1;
                }
            }
        }
    }
    /* Turn off all digital output ports when leaving menu */
    do00.Write(0);
    do01.Write(0);
    do02.Write(0);
    do03.Write(0);
    do04.Write(0);
    do05.Write(0);
    do06.Write(0);
    do07.Write(0);
    do10.Write(0);
    do11.Write(0);
    do12.Write(0);
    do13.Write(0);
    do14.Write(0);
    do15.Write(0);
    do16.Write(0);
    do17.Write(0);
    do20.Write(0);
    do21.Write(0);
    do22.Write(0);
    do23.Write(0);
    do24.Write(0);
    do25.Write(0);
    do26.Write(0);
    do27.Write(0);
    do30.Write(0);
    do31.Write(0);
    do32.Write(0);
    do33.Write(0);
    do34.Write(0);
    do35.Write(0);
    do36.Write(0);
    do37.Write(0);
    return menu;
}

/* RPS Menu function */
int RPMenu()
{
    LCD.Clear(BLACK);

    /* Open log file */
    SD.OpenLog();
    LCD.Clear(BLACK);

    /* Check to see if the Proteus has already initialized to a course region */
    if (!RPS_init)
    {
        RPS.InitializeTouchMenu();
        RPS_init = 1;
    }

    LCD.Clear(BLACK);

    /* Create RPS menu icons */
    FEHIcon::Icon RP_T[1];
    char rp_t_label[1][20] = {"RPS"};
    FEHIcon::DrawIconArray(RP_T, 1, 1, 1, 201, 1, 1, rp_t_label, MENU_C, TEXT_C);

    FEHIcon::Icon Back[1];
    char back_label[1][20] = {"<-"};
    FEHIcon::DrawIconArray(Back, 1, 1, 1, 201, 1, 260, back_label, MENU_C, TEXT_C);

    FEHIcon::Icon RP_XYH[3];
    char rp_xyh_label[3][20] = {"X", "Y", "Heading"};
    FEHIcon::DrawIconArray(RP_XYH, 1, 3, 40, 120, 1, 1, rp_xyh_label, SHOW_C, TEXT_C);

    FEHIcon::Icon RP_VAL[3];
    char rp_val_label[3][20] = {"", "", ""};
    FEHIcon::DrawIconArray(RP_VAL, 1, 3, 120, 40, 1, 1, rp_val_label, SHOW_C, TEXT_C);

    FEHIcon::Icon RP_LOG[1];
    char rp_log_label[1][20] = {"Log Data"};
    FEHIcon::DrawIconArray(RP_LOG, 1, 1, 201, 2, 1, 1, rp_log_label, SELT_C, TEXT_C);

    Buzzer.Buzz(beep_t);

    int menu=RP_MENU;
    float x, y;

    while(menu==RP_MENU)
    {
        /* Update RPS x, y, and heading values */
        RP_VAL[0].ChangeLabelFloat(RPS.X());
        RP_VAL[1].ChangeLabelFloat(RPS.Y());
        RP_VAL[2].ChangeLabelFloat(RPS.Heading());
        if (LCD.Touch(&x, &y))
        {
            /* Check to see if log data icon has been touched */
            if (RP_LOG[0].Pressed(x, y, 0))
            {
                /* While log data icon is pressed, update values and write them to log file */
                while (RP_LOG[0].Pressed(x, y, 1))
                {
                    RP_VAL[0].ChangeLabelFloat(RPS.X());
                    RP_VAL[1].ChangeLabelFloat(RPS.Y());
                    RP_VAL[2].ChangeLabelFloat(RPS.Heading());
                    SD.Printf("X: %f, Y: %f, Heading: %f\n", RPS.X(), RPS.Y(), RPS.Heading());
                }
                RP_LOG[0].Deselect();
            }
            /* If back button has been touched, go to main menu */
            if (Back[0].Pressed(x, y, 0))
            {
                Back[0].WhilePressed(x, y);
                menu = MN_MENU;
            }
        }
    }
    /* Close log file */
    SD.CloseLog();

    return menu;
}

/* Main function to control menu system */
int main(void)
{
    int menu=MN_MENU;
    while (true)
    {
        switch (menu)
        {
        case MN_MENU:
            menu = MNMenu();
            break;
        case DC_MENU:
            menu = DCMenu();
            break;
        case SV_MENU:
            menu = SVMenu();
            break;
        case DI_MENU:
            menu = DIMenu();
            break;
        case AI_MENU:
            menu = AIMenu();
            break;
        case AC_MENU:
            menu = ACMenu();
            break;
        case TO_MENU:
            menu = TOMenu();
            break;
        case DO_MENU:
            menu = DOMenu();
            break;
        case RP_MENU:
            menu = RPMenu();
            break;
        }
    }
}
