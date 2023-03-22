#include "T4_Program.h"
#include "defines.h"
#include "instructions.h"
#include "util.h"
#include "flags.h"
#include "channels.h"
#include "AlarmAbort.h"
#include "alarms.h"
#include "registers.h"
#include "s15.h"
#include "Executive.h"
#include "Waitlist.h"
#include "Imu.h"

#define MAX_PULSE 165

#define DISPLAY_SHOW_FLAG BITMASK1(15)

#define DIS_IND_OCDU_FAIL   BITMASK1(8)
#define DIS_IND_NOATT       BITMASK1(2)


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
#define CH12_OCDU_ZERO          BITMASK1(1)


#define CH30_TMP_IN_LIMITS      BITMASK1(15)
#define CH30_ISS_TURN_ON_REQ    BITMASK1(14)
#define CH30_IMU_FAIL           BITMASK1(13)
#define CH30_IMU_CDU_FAIL       BITMASK1(12)
#define CH30_IMU_CAGE           BITMASK1(11)
#define CH30_PIPA_FAIL          BITMASK1(10)
#define CH30_IMU_OPERATE        BITMASK1(9)
#define CH30_OCDU_FAIL          BITMASK1(7)

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

#define CH33_CSC BITMASK1(5)
#define CH33_4   BITMASK1(4)

#define MODES33_LAMP_TEST   BITMASK1(1)
#define MODES33_DAP_DISABLE BITMASK1(6)

#define STATE_RENDEZVOUS    BITMASK1(7)
#define STATE_IMU_USE       BITMASK1(8)

#define OPTMODES_ZOPT_SINCE_START   BITMASK1(10)
#define OPTMODES_RET_TO_COARSE      BITMASK1(9)
#define OPTMODES_OCDU_FAIL          BITMASK1(7)
#define OPTMODES_ZOPT_PROCESSING    BITMASK1(3)
#define OPTMODES_OCDU_FAIL_INHIB    BITMASK1(2)
#define OPTMODES_ZOPT_TASK_RUNNING  BITMASK1(1)

static int15_t dsruptsw;
static int15_t ruptreg1;

static int15_t IModes30;
static int15_t IModes33;
static int15_t Optmodes;

static int16_t OpticsMode;

enum {
    OPT_CMC = 1,
    OPT_MANUAL = 0,
    OPT_ZERO = -1
};


static int15_t State;

static int15_t Zone;

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
static const int15_t DisplayRowTable[DIS_SIZE] = {
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


static const int15_t ActivityTable[8] = {
    OPTTEST,
    OPTMON,
    IMUMON,
    NOTHING,
    OPTTEST,
    OPTMON,
    IMUMON,
    NOTHING,
};

static int15_t DisplayBuffer[DIS_SIZE];

// Counter of how many updates have been requested
static int15_t DisTableRequestCnt;
static int15_t DisCnt;

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
        int15_t display = DisplayBuffer[DisCnt];
        if (display & DISPLAY_SHOW_FLAG)
        {
            DisplayBuffer[DisCnt] = ~DisplayBuffer[DisCnt];
            // Mask out row address bits
            int15_t relays = DisplayBuffer[DisCnt] & DISPLAY_ROW_SETTING_MASK;
            // Add correct row address bits for writing to display
            int15_t row = DisplayRowTable[DisCnt] & DISPLAY_HI5;
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
    int15_t dsky_flag = FLAG_GET(FLAG_DSKY);
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

    int15_t ch32 = CH32_Read();
    int15_t changedMask = (IModes33 ^ ch32) & BIT14;

    if (changedMask != 0)
    {
        // Update IModes33 with bit 14 change
        IModes33 ^= changedMask;

        int15_t proceed = IModes33 & BIT14;
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
        int15_t turnOnReq = !(IModes30 & CH30_ISS_TURN_ON_REQ);

        if(turnOnReq)
        {
            // Mark initialization as started
            IModes30 |= MODES30_ISS_START_WAIT;
        }
        else
        {
            // Check if delay is complete
            int15_t delayComplete = CH12 & CH12_ISS_DELAY_COMPLETE;

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
    const int15_t MODES30_ISS_LAMP_FAILS = (CH30_IMU_FAIL | CH30_IMU_CDU_FAIL | CH30_PIPA_FAIL);

    int15_t inhib = IModes30 & MODES30_FAIL_INHIB_MASK;

    // Shift so that they overlap with the actual fail bits
    inhib <<= 9;

    // Since 0 indicates fail, a NOR will tell us if there is an allowed fail
    int15_t fail = ~(IModes30 | inhib) & MODES30_ISS_LAMP_FAILS;

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
        if(!(IModes33 & MODES33_LAMP_TEST))
        {
            // Not in test, turn off lamp
            CH11 &= ~CH11_ISS_LAMP;
        }
    }
}


void ISSUp(void)
{
    // Remove caging and inhbit flags
    IModes30 &= ~(MODES30_INHIB_ICDU_FAIL | MODES30_INHIB_IMU_FAIL | MODES30_CAGING2);

    // Enable DAP
    IModes33 &= ~MODES33_DAP_DISABLE;

    // There may be an ISS warning, which was inhibited until now
    UpdateISSWarningLamp();

    CH12 &= ~CH12_ISS_DELAY_COMPLETE;

    // Dont enable prog alarm on pip fail for 4 seconds
    Waitlist_Waitlist(400, Imu_PipFailOk);

    Waitlist_TaskOver();
}

void OnISSZeroDelay(void)
{
    Imu_ZeroICDU();

    // Finish zero process by taking back zero flag and resetting coarse align
    CH12 &= ~(CH12_IMU_COARSE_ALIGN | CH12_ICDU_ZERO);

    // Wait 10 seconds to find gimbals
    Waitlist_VarDelay(1024, ISSUp);
}

// # FUNCTIONAL DESCRIPTION:  THIS PROGRAM PROCESSES CHANGES OF THE IMUCAGE INBIT, CHANNEL 30 BITS 11.  IF THE BIT
// # CHANGES TO 0 (CAGE BUTTON PRESSED), THE ISS IS CAGED (ICDU ZERO + COARSE ALIGN + NO ATT LAMP) UNTIL THE
// # ASTRONAUT SELECTS ANOTHER PROGRAM TO ALIGN THE ISS.  ANY PULSE TRAINS TO THE ICDU'S AND GYRO'S ARE TERMINATED,
// # THE ASSOCIATE OUTCOUNTERS ARE ZEROED AND THE GYRO'S ARE DE-SELECTED.  NO ACTION OCCURS WHEN THE BUTTON IS
// # RELEASED (INBIT CHANGES TO 1).
void CheckIMUCage(void)
{
    const int15_t CH14_OUTPUTS = (CH14_CDUX_OUTPUT | CH14_CDUY_OUTPUT | CH14_CDUZ_OUTPUT | CH14_CDUS_OUTPUT | CH14_CDUT_OUTPUT | CH14_GYRO_OUTPUT);
    const int15_t CH12_OUTPUTS = (CH12_TVC_ENABLE | CH12_ICDU_ERR_CNT | CH12_ICDU_ZERO | CH12_IMU_COARSE_ALIGN | CH12_OCDU_ERR_CNT);

    if(IModes30 & CH30_IMU_CAGE)
    {
        // Was turned off, zero ISS
        Imu_NoAttOff();

        CH12 |= CH12_ICDU_ZERO;

        Imu_ZeroICDU();

        Waitlist_Waitlist(32, OnISSZeroDelay);

        // TODO: C33Test
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
        const int15_t GYRO_DESELECT = ~(CH14_GYRO_POLARITY | CH14_GYRO_AXIS1 | CH14_GYRO_AXIS2 | CH14_GYRO_POWER);
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

        // Disable DAP
        IModes33 |= MODES33_DAP_DISABLE;
        // Clear Track, REFSMMAT and Drift flags
        Imu_Rndrefdr();

        int15_t imuInUse = State & STATE_IMU_USE;
    
        // Clear Rendezvous and Imu use flags
        State &= ~(STATE_RENDEZVOUS | STATE_IMU_USE);

        if(imuInUse)
        {
            // IMU was in use, trigger alarm
            Alarm_Trigger(ALARM_ISS_OFF_IN_USE);
        }
    }
    else
    {
        // Was turned on

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
    int15_t changedMask = (IModes30 ^ CH30_Read()) & CH30_IMU_MASK;

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

        int15_t remainingChanges = changedMask & ~CH30_TMP_IN_LIMITS;

        for(int15_t bitPos = 14; remainingChanges; bitPos--)
        {
            // Bit pos is 1-indexed
            int15_t bitMask = 1 << (bitPos - 1);
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

    // TODO: TNONTEST
}

int15_t GetOptCommand(int16_t cdu, int16_t desired)
{
    int15_t sDiff = s15((int16_t)(desired - cdu));

    int15_t mulR[2];

    // Multiply by 2^12
    // By using highest word as command, this is a division by 4
    s15_mulDP(sDiff, 010000, mulR);

    mulR[1] = s15_add(mulR[1], mulR[1]);
    if(s15_getOvf(mulR[1]) != 0)
    {
        // Overflow will occur when the remainder of this "division by 4" is at least 0.5
        // Then add 1 to the result to round up
        mulR[0] = s15_add(mulR[0], s15_getOvf(mulR[1]));
    }

    // Store command
    return s15_ovfC(mulR[0]);
}

int15_t SetupCommand(int15_t command)
{
    int16_t command16 = s16(command);
    if (command16 > MAX_PULSE)
    {
        command16 = MAX_PULSE;
    }
    else if (command16 < -MAX_PULSE)
    {
        command16 = -MAX_PULSE;
    }

    // Zero command needs to be minus zero
    return s15_add(s15(command16), S15_MINUS_ZERO);
}

static int16_t Optind;

static uint16_t DesiredOptT;
static uint16_t DesiredOptS;
static int16_t DesiredOptMode;

void CheckZone(int16_t cduS)
{
    // We just filter manually for <= 45°
    // TODO: Check if this is correct
    if(s15_abs(cduS) <= 01000)
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
}

static void CalculateCommands(int16_t cduS, int16_t cduT, int15_t* cmdS, int15_t* cmdT)
{
    int15_t commandoShaft = GetOptCommand(cduS, DesiredOptS);
    int15_t commandoTrunion;

    // Check trunion command
    int16_t cduT = CDUT_R();

    // If the command is greater than 45°, we use maximum command
    /*
        Assumption: It is not an unsigned value, but a 2-complement signed value.

        Example: CDUT is somewhat over -45°, Desired is more over 0°
        1000000..11 -> CS -> 0_011111..00 -> AD with 000000000..111 ->
        0_10000..11 -> Positive overflow!

        Example: CDUT is near to 45°, Desired is under 0°
        0111111..11 -> CS -> 1_10000000.. -> AD with 1111111..10 ->
        1_01111..10 -> Negative overflow!

        If we add Command to inverted cduT, an overflow will indicate a difference > 45°
    */

    // TODO: Check calculations
    int15_t diff = s15_add(s15_se(DesiredOptT), s15_se(~cduT));
    int15_t ovf = s15_getOvf(diff);

    if (ovf == 0)
    {
        // Command is less than 45°
        // Use command
        commandoTrunion = GetOptCommand(cduT, DesiredOptT);
    }
    else if (ovf > 0)
    {
        // Command is greater than 45°
        // Use positive maximum command
        commandoTrunion = S15_MAXPOS;
    }
    else
    {
        // Command is less than -45°
        // Use negative maximum command
        commandoTrunion = S15_MAXNEG;
    }

    *cmdS = commandoShaft;
    *cmdT = commandoTrunion;
}

static void ShaftStopAvoidance(int15_t cduS, int15_t* cmdS)
{
    int15_t commandoShaft = *cmdS;

    // If CduS is greater than + or - 90°, check for possible stop problem
    // TODO: Maybe use s16 and test for positive and negative?
    if (s15_abs(cduS) > 020001)
    {
        // Greater than 90°

        if (Zone != 0)
        {
            int15_t zSign = Zone & S15_SIGN;
            int15_t cSign = commandoShaft & S15_SIGN;

            if (!(zSign ^ cSign))
            {
                // Zone and command have same sign
                // This threatens to reach end stop
                // TODO: Is the minus 1 relevant from CCS?
                if (s15_abs(DesiredOptS) <= 020001)
                {
                    // Target is in first or fourth quadrant
                    // Reverse command
                    commandoShaft = ~commandoShaft;

                    // TODO: How can zone and shaft have same sign
                    // if the command is less than the cdu? The signs here could be weird.
                    // Also the CCS command makes no sense, since XOR yields negative value.
                }
            }
        }
    }

    *cmdS = commandoShaft;
}

void DriveOptics(void)
{
    // shaft angle
    int16_t cduS = CDUS_R();

    // trunion angle
    int16_t cduT = CDUT_R();

    CheckZone(cduS);

    if(Optind >= 0 && OpticsMode == OPT_CMC)
    {
        // Check if OCDUS were zeroed since last FSTART
        int15_t zeroed = Optmodes & 01000;
        if(zeroed == 0)
        {
            // Optics not zeroed
            Alarm_Trigger(ALARM_OPT_NOT_ZEROED);
        }

        if(CH12 & CH12_OCDU_ERR_CNT)
        {
            int15_t commandoShaft;
            int15_t commandoTrunion;

            // Calculate commands
            CalculateCommands(cduS, cduT, &commandoShaft, &commandoTrunion);
            
            // Shaft stop avoidance
            ShaftStopAvoidance(cduS, &commandoShaft);
            

            Optind = 1;

            uint16_t commandCounter = 0;
            
            // Send command
            int15_t command = SetupCommand(commandoShaft);
            // Only send if not zero
            if(command == S15_MINUS_ZERO) commandCounter++;
            CDUSCMD_W(command);

            Optind--;

            command = SetupCommand(commandoTrunion);
            // Only send if not zero
            if(command == S15_MINUS_ZERO) commandCounter++;
            CDUTCMD_W(command);

            if(commandCounter > 0)
            {
                // Enable output
                CH14 |= (CH14_CDUS_OUTPUT | CH14_CDUT_OUTPUT);
            }
        }
        else
        {
            // Error counter not enabled, enable it
            CH12 |= CH12_OCDU_ERR_CNT;

            // Continue next time
        }
    }
}

/* This simplifies a very beautiful assembler control flow */
void ProcOCDUFail(void)
{
    // Check value of actual bit
    if(CH30_Read() & CH30_OCDU_FAIL)
    {
        // Dont turn off if lamp test
        if(!(IModes33 & MODES33_LAMP_TEST))
        {
            // Light off
            DisplayBuffer[DIS_INDICATORS] &= ~DIS_IND_OCDU_FAIL;
            DisplayBuffer[DIS_INDICATORS] |= DISPLAY_SHOW_FLAG;
        }
    }
    else
    {
        // Light on unless inhibited
        if(!(Optmodes & OPTMODES_OCDU_FAIL_INHIB))
        {
            DisplayBuffer[DIS_INDICATORS] |= (DIS_IND_OCDU_FAIL | DISPLAY_SHOW_FLAG);
        }
    }
}

static int16_t WTOption;
static int16_t ZOptCnt;

void InitZeroOptics(void)
{
    // Initialize for Zero optics
    WTOption = 0;

    // Inhibit errors and set zero processing
    Optmodes |= (OPTMODES_OCDU_FAIL_INHIB | OPTMODES_ZOPT_PROCESSING);
}

static void StartZeroOptics(void)
{
    // Setup 32 sample wait
    ZOptCnt = 32;

    InitZeroOptics();
}

static void CheckCoarsOpt(void)
{
    // Check for coars opt return
    if (Optmodes & OPTMODES_RET_TO_COARSE)
    {
        // Set coars opt working
        Optind = 1;
        CH12 |= CH12_OCDU_ERR_CNT;
    }
}

static void CancelZOpt(void)
{
    // Cancel Zero optics
    Optmodes &= ~(OPTMODES_ZOPT_PROCESSING | OPTMODES_OCDU_FAIL_INHIB);
    CH12 &= ~CH12_OCDU_ZERO;
}

static void EndZeroOptics2(void)
{
    Optmodes |= OPTMODES_ZOPT_SINCE_START;
    // Enable fails 
    Optmodes &= ~(OPTMODES_ZOPT_PROCESSING | OPTMODES_OCDU_FAIL_INHIB | OPTMODES_ZOPT_TASK_RUNNING);

    // Check OCDU Fail bit after enable
    ProcOCDUFail();

    // Finish this task
    Waitlist_TaskOver();
}

static void EndZeroOptics(void)
{
    CDUS_W(0);
    Zone = 0;
    // -20°
    CDUT_W((int16_t)-7200);

    CH12 &= ~CH12_OCDU_ZERO;

    // Delay 200ms for CDUs to resynchronize
    Waitlist_VarDelay(20, EndZeroOptics2);
}

/* Optics monitoring and zero routines */
void MonitorOptics(void)
{
    // Check if OCDU Fail bit changed
    int15_t ch30 = CH30_Read();
    int15_t changes = (Optmodes ^ ch30) & CH30_OCDU_FAIL;
    ruptreg1 = changes;

    if(changes)
    {
        ProcOCDUFail();
    }

    // Bypass if TVC Takeover
    if(Optind != S15_MINUS_ZERO)
    {
        changes = (Optmodes ^ CH33) & (CH33_CSC | CH33_4);
        ruptreg1 += changes;

        // Update Optmodes
        Optmodes ^= ruptreg1;

        // Invert to inspect switch settings
        int15_t switches = ~Optmodes;

        if(switches == 0)
        {
            OpticsMode = OPT_MANUAL;
        }
        else
        {
            if (switches & CH33_CSC)
            {
                // CSC - Optical Switches positive
                // (This indicates that computer is in control of optics)
                OpticsMode = OPT_CMC;
            }
            else
            {
                // Zero Optics
                OpticsMode = OPT_ZERO;
            }
        }

        // Previous setting
        if(DesiredOptMode == OPT_CMC)
        {
            if(OpticsMode == OPT_ZERO)
            {
                StartZeroOptics();
            }
            if(OpticsMode != OPT_CMC)
            {
                // If coars working
                if(Optind >= 0)
                {
                    // Coars is working and switch not CMC - kill coars
                    Optind = -1;
                    CH12 &= ~CH12_OCDU_ERR_CNT;
                    Optmodes |= OPTMODES_RET_TO_COARSE;
                }
            }
        }
        else if(DesiredOptMode == OPT_MANUAL)
        {
            // Manual

            if(OpticsMode == OPT_CMC)
            {
                // Cancel Zero optics
                WTOption = 0;
                ZOptCnt = 0;

                // Check for coars opt return
                CheckCoarsOpt();
            }
            else if(OpticsMode == OPT_MANUAL)
            {
                // Decrement return option time
                if(WTOption > 0)
                    WTOption--;
            }
            else if(OpticsMode == OPT_ZERO)
            {
                // Check return option
                if(WTOption > 0)
                {
                    // 5 sec return good - continue zoptics
                    // Show zero optics processing
                    InitZeroOptics();
                }
                else if(WTOption == 0)
                {
                    // 5 sec ran out - restart zero optics
                    StartZeroOptics();
                }
            }
        }
        else if(DesiredOptMode == OPT_ZERO)
        {

            if (Optmodes & OPTMODES_ZOPT_TASK_RUNNING)
            {
                // Zeroing not done yet, switch not allowed
                // Wait 1 sample
                return;
            }

            // Zero Optics
            // Check current switch status
            if(OpticsMode == OPT_CMC)
            {   
                if(Optmodes & OPTMODES_ZOPT_PROCESSING)
                {
                    // Alarm if processing ZOPT
                    Alarm_Trigger(ALARM_SWITCH_WHILE_ZOPT);

                    CancelZOpt();

                    WTOption = 0;
                    ZOptCnt = 0;
                }

                // Check for coars opt return
                CheckCoarsOpt();
                
            }
            else if(OpticsMode == OPT_MANUAL)
            {
                if(Optmodes & OPTMODES_ZOPT_PROCESSING)
                {
                    Alarm_Trigger(ALARM_SWITCH_WHILE_ZOPT);

                    // Set return option
                    WTOption = 11;
                    CancelZOpt();
                }
            }
            else if(OpticsMode == OPT_ZERO)
            {
                if(Optmodes & OPTMODES_ZOPT_PROCESSING)
                {
                    // Still processing - check counter
                    if(ZOptCnt > 0)
                    {
                        // Not finished
                        ZOptCnt--;
                    }
                    else
                    {
                        // Finished samples
                        CH12 |= CH12_OCDU_ZERO;

                        // Hold zero for 200ms
                        Waitlist_Waitlist(20, EndZeroOptics);

                        // Show task working
                        Optmodes |= OPTMODES_ZOPT_TASK_RUNNING;
                    }
                }
            }
        }

        // Update DesiredOptMode
        DesiredOptMode = OpticsMode;
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
        int15_t activityIndex = dsruptsw;
        
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
                DriveOptics();
            break;

            case OPTMON:
                MonitorOptics();
            break;

            case IMUMON:
                MonitorIMU();
            break;

            case NOTHING:
                // Nothing, leave interrupt
            break;
        }
    }
    else
    {
        // GOTO QUIKDSP
    }
}
