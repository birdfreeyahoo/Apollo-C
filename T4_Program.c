#include "T4_Program.h"
#include "defines.h"
#include "instructions.h"
#include "util.h"
#include "flags.h"
#include "channels.h"
#include "AlarmAbort.h"
#include "alarms.h"
#include "registers.h"
#include "Executive.h"
#include "Imu.h"

#define DISPLAY_SHOW_FLAG BITMASK1(15)

#define DIS_IND_NOATT   BITMASK1(2)


#define DISPLAY_ROW_SETTING_MASK 003777
#define DISPLAY_HI5 076000

#define T4_LOAD_VALUE 16372

#define PROC_JOB_PRIORITY 030000

#define BIT14 020000

// Starts IMU stabilization and removes ISS Turn On Request
#define CH12_ISS_DELAY_COMPLETE BITMASK1(15)
#define CH12_TVC_ENABLE         BITMASK1(8)
#define CH12_ICDU_ERR_CNT       BITMASK1(6)
#define CH12_ICDU_ZERO          BITMASK1(5)
#define CH12_IMU_COARSE_ALIGN   BITMASK1(4)
#define CH12_OCDU_ERR_CNT       BITMASK1(2)


#define CH30_TMP_IN_LIMITS      BITMASK1(15)
#define CH30_ISS_TURN_ON_REQ    BITMASK1(14)
#define CH30_IMU_FAIL           BITMASK1(13)
#define CH30_IMU_CDU_FAIL       BITMASK1(12)
#define CH30_IMU_CAGE           BITMASK1(11)
#define CH30_PIPA_FAIL          BITMASK1(10)
#define CH30_IMU_OPERATE        BITMASK1(9)

#define CH30_IMU_MASK (CH30_TMP_IN_LIMITS      | \
                       CH30_ISS_TURN_ON_REQ    | \
                       CH30_IMU_FAIL           | \
                       CH30_IMU_CDU_FAIL       | \
                       CH30_IMU_CAGE           | \
                       CH30_IMU_OPERATE)


#define MODES30_ISS_START_WAIT              BITMASK1(7)
#define MODES30_CAGING2                     BITMASK1(6)
#define MODES30_CAGING1                     BITMASK1(5)
#define MODES30_INHIB_IMU_FAIL              BITMASK1(4)
#define MODES30_INHIB_ICDU_FAIL             BITMASK1(3)
#define MODES30_ISS_TURN_ON_SEQ_FAIL        BITMASK1(2)
#define MODES30_INHIB_PIPA_FAIL             BITMASK1(1)

#define MODES30_FAIL_INHIB_MASK (MODES30_INHIB_IMU_FAIL  | \
                                 MODES30_INHIB_ICDU_FAIL | \
                                 MODES30_INHIB_PIPA_FAIL)

#define CH11_ISS_LAMP   BITMASK1(1)
#define CH11_TMP_LAMP   BITMASK1(4)
#define CH11_SPS_ENGINE BITMASK1(13)

#define CH14_CDUX_OUTPUT BITMASK1(15)
#define CH14_CDUY_OUTPUT BITMASK1(14)
#define CH14_CDUZ_OUTPUT BITMASK1(13)
#define CH14_CDUT_OUTPUT BITMASK1(12)
#define CH14_CDUS_OUTPUT BITMASK1(11)
#define CH14_GYRO_OUTPUT BITMASK1(10)
#define CH14_GYRO_POLARITY BITMASK1(9)
#define CH14_GYRO_AXIS1  BITMASK1(8)
#define CH14_GYRO_AXIS2  BITMASK1(7)
#define CH14_GYRO_POWER  BITMASK1(6)

#define MODES33_LAMP_TEST   BITMASK1(1)
#define MODES33_DAP_DISABLE BITMASK1(6)

#define STATE_RENDEZVOUS    BITMASK1(7)
#define STATE_IMU_USE       BITMASK1(8)

static sint15 dsruptsw;
static sint15 ruptreg1;

static sint15 IModes30;
static sint15 IModes33;

static sint15 State;

static sint15 Zone;

enum {
    DIS_REGULAR_SIZE = 10,
    DIS_INDICATORS = 11,
    DIS_SIZE
};

enum {
    OPTTEST,
    OPTMON,
    IMUMON,
    NOTHING,
};

// Table of display row addresses (in octal)
static const sint15 DisplayRowTable[DIS_SIZE] = {
    04025,
    010003,
    014031,
    020033,
    024017,
    030036,
    034034,
    040023,
    044035,
    050037,
    054000,
    060000
};


static const sint15 ActivityTable[8] = {
    OPTTEST,
    OPTMON,
    IMUMON,
    NOTHING,
    OPTTEST,
    OPTMON,
    IMUMON,
    NOTHING,
};

static sint15 DisplayBuffer[DIS_SIZE];

// Counter of how many updates have been requested
static sint15 DisTableRequestCnt;
static sint15 DisCnt;

// TODO: ???
static void Hang20(void)
{
    dsruptsw -= 022400;
}

static void OutputNoDisplay(void)
{
    Dis_Write(0);

    // TODO: Set time
}

static bool OutputNextDisplayRow(void)
{
    DisTableRequestCnt--;
    uint8 firstPass = 0;

    while (DisCnt > 0)
    {
        sint15 display = DisplayBuffer[DisCnt];
        if (display & DISPLAY_SHOW_FLAG)
        {
            DisplayBuffer[DisCnt] = ~DisplayBuffer[DisCnt];
            // Mask out row address bits
            sint15 relays = DisplayBuffer[DisCnt] & DISPLAY_ROW_SETTING_MASK;
            // Add correct row address bits for writing to display
            sint15 row = DisplayRowTable[DisCnt] & DISPLAY_HI5;
            // Combine row with relay settings
            row += relays;
            Dis_Write(row);

            // Displayed something
            return TRUE;
        }
        else
        {
            if (DisCnt > 0)
            {
                DisCnt--;
                continue;
            }
            // +0 impossible
            else if (DisCnt == 0)
            {
                // Second pass
                if (firstPass == 0)
                {
                    DisTableRequestCnt = 0;
                }
                // First pass
                else
                {
                    // Next time will be second pass
                    firstPass = 0;
                    DisCnt = DIS_REGULAR_SIZE;
                }
            }
        }
    }

    // Displayed nothing
    return FALSE;
}

/* Return: If a display was output */
static bool OutputDisplay(void)
{
    sint15 dsky_flag = FLAG_GET(FLAG_DSKY);
    bool output = FALSE;

    // Only display when dsky flag is set and there was activity
    if(dsky_flag && (DisTableRequestCnt > 0))
    {
        output = OutputNextDisplayRow();
    }

    return output;
}

static void ProcessProceedButton(void)
{
    // # THE STATUS OF THE PROCEED PUSHBUTTON IS MONITORED EVERY 120 MILLISECONDS VIA THE CHANNEL 32 BIT 14 INBIT.
    // # THE STATE OF THIS INBIT IS COMPARED WITH ITS STATE DURING THE PREVIOUS T4RUPT AND IS PROCESSED AS FOLLOWS.
    // #	IF PREV ON AND NOW ON 	-- BYPASS
    // #	IF PREV ON AND NOW OFF	-- UPDATE IMODES33
    // #	IF PREV OFF AND NOW ON	-- UPDATE IMODES33 AND PROCESS VIA PINBALL
    // #	IF PREV OFF AND NOW OFF	-- BYPASS
    // # THE LOGIC EMPLOYED REQUIRES ONLY 9 MCT (APPROX. 108 MICROSECONDS) OF COMPUTER TIME WHEN NO CHANGES OCCUR.

    // Bit 14 will be 0 when pressed and 1 when released
    // Check if it was changed

    sint15 ch32 = CH32_Read();
    sint15 changedMask = (IModes33 ^ ch32) & BIT14;

    if (changedMask != 0)
    {
        // Update IModes33 with bit 14 change
        IModes33 ^= changedMask;

        sint15 proceed = IModes33 & BIT14;
        if (proceed)
        {
            // Was on, now off, do nothing more
        }
        else
        {
            // Was off, now on, schedule process via pinball
            Exec_JobRequest_Novac(/*TODO*/ 0, PROC_JOB_PRIORITY);
        }
    }
}

// # FUNCTIONAL DESCRIPTION:  THIS PROGRAM IS CALLED BY IMUMON WHEN A CHANGE OF BIT 14 OF CHANNEL 30 (ISS TURN-ON
// # REQUEST) IS DETECTED.  UPON ENTRY, ITURNON CHECKS IF A TURN-ON DELAY SEQUENCE HAS FAILED, AND IF SO, IT EXITS.
// # IF NOT, IT CHECKS WHETHER THE TURN-ON REQUEST CHANGE IS TO ON OR OFF.  IF ON, IT SETS BIT 7 OF IMODES30 TO 1 SO
// # THAT TNONTEST WILL INITIATE THE ISS INITIALIZATION SEQUENCE.  IF OFF, THE TURN-ON DELAY SIGNAL, CHANNEL 12 BIT
// # 15, IS CHECKED AND IF IT IS ON, ITURNON EXITS.  IF THE DELAY SIGNAL IS OFF, PROGRAM ALARM 00207 IS ISSUED, BIT 2
// # OF IMODES30 IS SET TO 1 AND THE PROGRAM EXITS.
// #
// # THE SETTING OF BIT 2 OF IMODES30 (ISS DELAY SEQUENCE FAIL) INHIBITS THIS ROUTINE AND IMUOP FROM
// # PROCESSING ANY CHANGES.  THIS BIT WILL BE RESET BY THE ENDTNON ROUTINE WHEN THE CURRENT 90 SECOND DELAY PERIOD
// # ENDS.
void CheckISSTurnOn(void)
{
    if(IModes30 & MODES30_ISS_TURN_ON_SEQ_FAIL)
    {
        // Fail in turn on sequence, dont do anything
    }
    else
    {
        sint15 turnOnReq = !(IModes30 & CH30_ISS_TURN_ON_REQ);

        if(turnOnReq)
        {
            // Mark initialization as started
            IModes30 |= MODES30_ISS_START_WAIT;
        }
        else
        {
            // Check if delay is complete
            sint15 delayComplete = CH12 & CH12_ISS_DELAY_COMPLETE;

            if(delayComplete)
            {
                // Delay was completed, which automatically reset the ISS turn on request
                // So there is nothing to do, since everything went as expected
            }
            else
            {
                // Delay not complete, set fail flag
                IModes30 |= MODES30_ISS_TURN_ON_SEQ_FAIL;

                // Trigger alarm
                Alarm_Trigger(ALARM_ISS_SEQ_FAIL);
            }
        }
    }
}

// # FUNCTIONAL DESCRIPTION:  THIS PROGRAM TURNS THE ISS WARNING LAMP ON AND OFF (CHANNEL 11 BIT 1 = 1 FOR ON,
// # 0 FOR OFF) DEPENDING ON THE STATUS OF IMODES30 BITS 13 (IMU FAIL) AND 4 (INHIBIT IMU FAIL), 12 (ICDU FAIL) AND
// # 3 (INHIBIT ICDU FAIL), AND 10 (PIPA FAIL) AND 1 (INHIBIT PIPA FAIL).  THE LAMP IS LEFT ON IF A LAMP TEST IS IN
// # PROGRESS.
void UpdateISSWarningLamp(void)
{
    const sint15 MODES30_ISS_LAMP_FAILS = (CH30_IMU_FAIL | CH30_IMU_CDU_FAIL | CH30_PIPA_FAIL);

    sint15 inhib = IModes30 & MODES30_FAIL_INHIB_MASK;

    // Shift so that they overlap with the actual fail bits
    inhib <<= 9;

    // Since 0 indicates fail, a NOR will tell us if there is an allowed fail
    sint15 fail = ~(IModes30 | inhib) & MODES30_ISS_LAMP_FAILS;

    if(fail)
    {
        // At least one fail, turn on lamp

        // Trigger alarm
        // Alarm code is fail bits - 1 (?)
        Alarm_Trigger(fail - 1);

        // Turn on lamp
        CH11 |= CH11_ISS_LAMP;
    }
    else
    {
        // No fails, turn off lamp if not in test
        if(IModes33 & MODES33_LAMP_TEST)
        {
            // In test, leave lamp on
        }
        else
        {
            // Not in test, turn off lamp
            CH11 &= ~CH11_ISS_LAMP;
        }
    }
}

// # FUNCTIONAL DESCRIPTION:  THIS PROGRAM PROCESSES CHANGES OF THE IMUCAGE INBIT, CHANNEL 30 BITS 11.  IF THE BIT
// # CHANGES TO 0 (CAGE BUTTON PRESSED), THE ISS IS CAGED (ICDU ZERO + COARSE ALIGN + NO ATT LAMP) UNTIL THE
// # ASTRONAUT SELECTS ANOTHER PROGRAM TO ALIGN THE ISS.  ANY PULSE TRAINS TO THE ICDU'S AND GYRO'S ARE TERMINATED,
// # THE ASSOCIATE OUTCOUNTERS ARE ZEROED AND THE GYRO'S ARE DE-SELECTED.  NO ACTION OCCURS WHEN THE BUTTON IS
// # RELEASED (INBIT CHANGES TO 1).
void CheckIMUCage(void)
{
    const sint15 CH14_OUTPUTS = (CH14_CDUX_OUTPUT | CH14_CDUY_OUTPUT | CH14_CDUZ_OUTPUT | CH14_CDUS_OUTPUT | CH14_CDUT_OUTPUT | CH14_GYRO_OUTPUT);
    const sint15 CH12_OUTPUTS = (CH12_TVC_ENABLE | CH12_ICDU_ERR_CNT | CH12_ICDU_ZERO | CH12_IMU_COARSE_ALIGN | CH12_OCDU_ERR_CNT);

    if(IModes30 & CH30_IMU_CAGE)
    {
        // Was turned off, no action
        // TODO: ISSZERO
    }
    else
    {
        // Disable pulse outputs for CDU, Optics and Gyros
        CH14 &= ~CH14_OUTPUTS;

        // Disable TVC, Error counter, coarse align and zero cdu
        CH12 &= ~CH12_OUTPUTS;

        // Turn off engines
        CH11 &= ~CH11_SPS_ENGINE;

        // Show NO ATT lamp
        DisplayBuffer[DIS_INDICATORS] |= (DIS_IND_NOATT | DISPLAY_SHOW_FLAG);

        // Set flags and inhibit errors
        IModes30 |= (MODES30_FAIL_INHIB_MASK | MODES30_CAGING1 | MODES30_CAGING2);

        // Disable DAP and hold modes
        IModes33 |= MODES33_DAP_DISABLE;

        // Clear Track, REFSMMAT and Drift flags
        Imu_Rndrefdr();

        // Zero commands
        CDUXCMD_W(0);
        CDUYCMD_W(0);
        CDUZCMD_W(0);
        GYROCMD_W(0);

        // Deselect gyros
        const sint15 GYRO_DESELECT = ~(CH14_GYRO_POLARITY | CH14_GYRO_AXIS1 | CH14_GYRO_AXIS2 | CH14_GYRO_POWER);
        CH14 &= GYRO_DESELECT;
    }
}

// # FUNCTIONAL DESCRIPTION:  THIS PROGRAM PROCESSES CHANGES IN THE ISS OPERATE DISCRETE, BIT 9 OF CHANNEL 30.
// # IF THE INBIT CHANGES TO 0, INDICATING ISS ON, IMUOP GENERALLY SETS BIT 7 OF IMODES30 TO 1 TO REQUEST ISS
// # INITIALIZATION VIA TNONTEST.  AN EXCEPTION IS DURING A FAILED ISS DELAY DURING WHICH BIT 2 OF IMODES30 IS SET
// # TO 1 AND NO FURTHER INITIALIZATION IS REQUIRED.  WHEN THE INBIT CHANGES TO 1, INDICATING ISS OFF, IMUSEFLG IS
// # TESTED TO SEE IF ANY PROGRAM WAS USING THE ISS.  IF SO, PROGRAM ALARM 00214 IS ISSUED.
void CheckISSOperate(void)
{
    if(IModes30 & CH30_IMU_OPERATE)
    {
        // Was turned off

        if(IModes30 & MODES30_ISS_TURN_ON_SEQ_FAIL)
        {
            // ISS failed to turn on recently, no action before 90 seconds run out
        }
        else
        {
            // Wait 1 sample
            IModes30 |= MODES30_ISS_START_WAIT;
        }
    }
    else
    {
        // Was turned on

        // Disable DAP
        IModes33 |= MODES33_DAP_DISABLE;
        // Clear Track, REFSMMAT and Drift flags
        Imu_Rndrefdr();

        sint15 imuInUse = State & STATE_IMU_USE;
    
        // Clear Rendezvous and Imu use flags
        State &= ~(STATE_RENDEZVOUS | STATE_IMU_USE);

        if(imuInUse)
        {
            // IMU was in use, trigger alarm
            Alarm_Trigger(ALARM_ISS_OFF_IN_USE);
        }
    }
}

// # FUNCTIONAL DESCRIPTION:  THIS PROGRAM IS ENTERED EVERY 480 MS.  IT DETECTS CHANGES OF THE IMU STATUS BITS IN
// # CHANNEL 30 AND CALLS THE APPROPRIATE SUBROUTINES.  THE BITS PROCESSED AND THEIR RELEVANT SUBROUTINES ARE:
// #
// #	FUNCTION		BIT	SUBROUTINE CALLED
// #	--------		---	-----------------
// #	TEMP IN LIMITS		 15	TLIM
// #	ISS TURN-ON REQUEST	 14	ITURNON
// #	IMU FAIL		 13	IMUFAIL (SETISSW)
// #	IMU CDU FAIL		 12	ICDUFAIL (SETISSW)
// #	IMU CAGE		 11	IMUCAGE
// #	IMU OPERATE		  9	IMUOP
// #
// # THE LAST SAMPLED STATE OF THESE BITS IS LEFT IN IMODES30.  ALSO, EACH SUBROUTINE CALLED FINDS THE NEW
// # VALUE OF THE BIT IN A, WITH Q SET TO THE PROPER RETURN LOCATION NXTIFAIL.
// #
void MonitorIMU(void)
{
    // Check which bits changed
    sint15 changedMask = (IModes30 ^ CH30_Read()) & CH30_IMU_MASK;

    // If any bits changed
    if(changedMask)
    {
        // Update IModes30 with bit changes
        IModes30 ^= changedMask;

        // Check if temp limit bit changed
        if(changedMask & CH30_TMP_IN_LIMITS)
        {
            // Update temp lamp
            // bit is inverted logic
            if(!(IModes30 & CH30_TMP_IN_LIMITS))
            {
                // Temp in limits: OK
                // Dont turn off when test active
                if(!(IModes33 & MODES33_LAMP_TEST))
                {
                    CH11 &= ~CH11_TMP_LAMP;
                }
            }
            else
            {
                // Temp out of limits, turn on lamp
                CH11 |= CH11_TMP_LAMP;
            }
        }

        sint15 remainingChanges = changedMask & ~CH30_TMP_IN_LIMITS;

        for(sint15 bitPos = 14; remainingChanges; bitPos--)
        {
            sint15 bitMask = 1 << bitPos;
            if(remainingChanges & bitMask)
            {
                // Clear bit
                remainingChanges &= ~bitMask;

                switch(bitMask)
                {
                    case CH30_ISS_TURN_ON_REQ:
                        CheckISSTurnOn();
                    break;

                    case CH30_IMU_FAIL:     
                    case CH30_IMU_CDU_FAIL:
                        UpdateISSWarningLamp();
                    break;

                    case CH30_IMU_CAGE:
                        CheckIMUCage();
                    break;

                    case CH30_IMU_OPERATE:
                        CheckISSOperate();
                    break;
                }

            }
        }
    }

    // TNONTEST
}

static sint15 Optind;
static sint15 SwSample;

static sint15 Optmodes;

static sint15 DesiredOptT;
static sint15 DesiredOptS;

void DriveOptics(void)
{
    // shaft angle
    sint15 cduS = CDUS_R();

    /*

    CDUS spans 360°. It is a 15-bit 2-complement unsigned value.
    We can partition it in 45° sectors, each of which is 12 bits long:

    YYY XXXX XXXX XXXX
    |_| |_____________|
     |       |
     |       +-- Degree value 0° - 45°
     +-- Sector index

    For original CCS instructions, that means:

    > 0: +070000 decrements the sector. In most cases that leads to an overflow, adding back 
         what was lost due to CCS ABSD operation.
    = 0: Does nothing
    < 0: ABSD will invert all bits and subtract 1. Again, sector will be decremented.

    Mapping:
    u15 input | u15 output | s15 output
        30°   |    345°    |    -15°
        75°   |     30°    |     30°
        120°  |     75°    |     75°
        165°  |    120°    |    120°
        210°  |    165°    |    165°
        255°  |    210°    |   -150°
        300°  |    255°    |   -105°
        345°  |    300°    |    -60°
          0°  |      0°    |      0°
        "-0"  |    360°    |     -0°

    So all that reflects a subtraction of 45°

    So what will be filtered by BZMF?
    0 - 45° and 225 - 360°
    45 includes 010000 and 225 includes 050000

    225-360° actually not since the ABS at the beginning!
    */

    // We just filter manually for <= 45°
    // TODO: Check if this is correct
    if(cduS <= 010000)
    {
        Zone = 0;
    }
    else if(Zone == 0)
    {
        // If zone is already 0, change to + or -
        Zone = cduS;
    }
    else
    {
        // Leave zone as is
    }

    if(Optind >= 0 && SwSample > 0)
    {
        // Check if OCDUS was zeroes since last FSTART
        sint15 ocdus = Optmodes & 01000;
        if(ocdus == 0)
        {
            // Optics not zeroed
            Alarm_Trigger(ALARM_OPT_NOT_ZEROED);
        }

        if(CH12 & CH12_OCDU_ERR_CNT)
        {
            Optind = 1;

            // Get trunion command
            sint15 cduT = CDUT_R();

            sint15 tDiff = -cduT + DesiredOptT; 

            sint15 sDiff = DesiredOptS - cduS;

        }
        else
        {
            // Error counter not enabled, enable it
            CH12 |= CH12_OCDU_ERR_CNT;

            // Continue next time
        }
    }
    else
    {

    }
}


void T4_Program(void)
{
    if(dsruptsw == 0)
    {
        dsruptsw = 7;
    }
    if(dsruptsw >= 0)
    {
        dsruptsw--;

        // Per-second activity executed now
        sint15 activityIndex = dsruptsw;
        
        if(DisplayBuffer[DIS_INDICATORS] & DISPLAY_SHOW_FLAG)
        {
            // Show indicators
            DisplayBuffer[DIS_INDICATORS] &= DISPLAY_ROW_SETTING_MASK;
            Dis_Write(DisplayBuffer[DIS_INDICATORS] + DisplayRowTable[DIS_INDICATORS]);

            Hang20();
        }
        else
        {
            // Output all display rows that have been requested
            if(OutputDisplay())
            {
                Hang20();
            }
            else
            {
                OutputNoDisplay();

                // TODO : Set time
            }
        }

        // TODO: Set time

        ProcessProceedButton();
        

        // Appropriate once per second activity, which is rotated every interrupt
        switch(ActivityTable[activityIndex])
        {
            case OPTTEST:
            {
                DriveOptics();
            }
            break;

            case OPTMON:

            break;

            case IMUMON:
                MonitorIMU();
            break;

            case NOTHING:

            break;
        }
        
        
    }
    else
    {
        // GOTO QUIKDSP
    }
}
