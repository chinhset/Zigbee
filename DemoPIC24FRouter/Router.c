/* Microchip ZigBee2006 Residential Stack
 * Demo Router
 *
 *******************************************************************************
 * FileName:        Router.c
 * Date modified:   27/11/2012
 *
 * Microcontroller: PIC24FJ128GA306
 * Transceiver:     Microchip MRF24J40
 *
 * Editor:          MPLAB X IDE v1.50
 * Complier:        MCC30 v3.31
 *
 * Developer:       Nguyen Tien Dat
 * Class:           KSTN - DTVT - K54
 * Group:           WSAN
 * Organization:    Lab411 - Hanoi University of Technology
 *
 * Description:     Chuong trinh nay mo ta cach setup mot thiet bi tro thanh mot
 *                  node mang dong vai tro la mot Zigbee Router. Chuong trinh
 *                  cung duoc thiet ke de tuong tac voi cac thiet bi khac trong
 *                  mang theo chuan Zigbee (Zigbee Router hoac Zigbee End Device).
 *                  Cac hoat dong do bao gom: cho phep gia nhap/khong gia nhap
 *                  mang, yeu cau ban tin tu mot thiet bi hoac mot nhom thiet bi
 *                  trong mang, gui ban tin toi mot thiet bi hoac mot nhom cac
 *                  thiet bi trong mang.
 *
 * Note:            Xem lich su ma nguon trong file Readme.txt
*******************************************************************************/
//******************************************************************************
// Header Files
//******************************************************************************

// Include the main ZigBee header file.
#include "zAPL.h"
#include "zNVM.h"
#include "zPHY_MRF24J40.h"
#ifdef I_SUPPORT_SECURITY
    #include "zSecurity.h"
#endif

// If you are going to send data to a terminal, include this file.
#if defined(USE_USART) || defined(ROUTER_EMB) || defined(USE_CONTROL_PUMP) || defined(USE_MicroWaveS)
    #include "console.h"
#endif

// If you are going to get temperature and humidity, include this file.
#if defined(USE_SHT10)
    #include "delay.h"
    #include "SHT1x.h"
#endif
//******************************************************************************
// Configuration Bits
//******************************************************************************

#if defined(__PIC24F__)
    // Configure by dat_a3cbq91
    _CONFIG1(JTAGEN_OFF & GCP_OFF & GWRP_OFF & LPCFG_OFF & ICS_PGx1 & WINDIS_OFF & FWDTEN_WDT_SW &  FWPSA_PR128 & WDTPS_PS512);
    _CONFIG2(IESO_ON & FNOSC_FRC & FCKSM_CSECME & OSCIOFCN_OFF & IOL1WAY_OFF & POSCMD_HS);
    _CONFIG3(BOREN_ON);
    // End by dat_a3cbq91
#elif defined(__dsPIC33F__) || defined(__PIC24H__)
    // Explorer 16 board
    _FOSCSEL(FNOSC_PRI)			// primary osc
    _FOSC(OSCIOFNC_OFF & POSCMD_XT)	// XT Osc
    _FWDT(FWDTEN_OFF)			// Disable Watchdog timer
    // JTAG should be disabled as well
#elif defined(__dsPIC30F__)
    // dsPICDEM 1.1 board
    _FOSC(XT_PLL16)		// XT Osc + 16X PLL
    _FWDT(WDT_OFF)		// Disable Watchdog timer
    _FBORPOR(MCLR_EN & PBOR_OFF & PWRT_OFF)
#else
    #error Other compilers are not yet supported.
#endif

//******************************************************************************
// Constants                        //defined application service   @dat_a3cbq91
//******************************************************************************
//Hardware definations
#if defined(CHAMSOCLAN) || defined(THEODOIDONGVAT) || defined(ROUTER_EMB)
    #define WSAN_LED_TRIS 		TRISFbits.TRISF3
    #define WSAN_LED 			LATFbits.LATF3
#else
    #define POWER_LED_TRIS 		TRISEbits.TRISE4
    #define POWER_LED 			LATEbits.LATE4
#endif
    #define REF_VOLTAGE_TRIS            TRISBbits.TRISB8
    #define REF_ANALOG			ANSBbits.ANSB8

#define ON              1
#define SET             1
#define ENABLE          1
#define OFF             0
#define CLEAR           0
#define DISABLE         0

// Gia tri cho biet bo ADC dang do tin hieu tu cam bien khoi hay tu nguon cap
#define MQ6_Mode_ADC    0b00
#define Power_Mode_ADC  0b01
#define Finish_Convert  0b10

// Cac trang thai cua node mang
// 0x02 --> phat hien co khoi
// 0x03 --> muc nang luong co dau hieu can kiet
// 0x04 --> phat hien co doi tuong dang xam nhap
#define HasSmoke        0x02
#define LowPower        0x03
#define Security        0x04

// Cac gia tri nay dung de thiet lap cho thanh ghi AD1CON2 lay dien ap tham chieu o dau?
#define Measure_MQ6     0b01
#define Measure_Voltage 0b00

// Gia tri nguong cua cac tin hieu tu cam bien khoi va nguon nang luong
// cho phep node gui tin hieu canh bao ve Router-EMB
#define ThresholdMQ6	900
#define ThresholdPower	1350    //2,4V @ADC 12 bits
//******************************************************************************
// Function Prototypes
//******************************************************************************
//functions that have been created and modified by dat_a3cbq91
void HardwareInit( void );
void ProcessNONZigBeeTasks(void);
void ProcessZigBeePrimitives(void);
//DKCS
void SendOneByte(BYTE data, WORD ClusterID);
BYTE SendDataEnable = CLEAR;
WORD SendDataCount;
WORD ReceiveDataCout;
WORD MessageSuccessfulyCount = 0;
extern ROUTE_DISCOVERY_ENTRY *routeDiscoveryTablePointer[ROUTE_DISCOVERY_TABLE_SIZE];
extern BYTE rdIndex;

extern void RemoveAllGroups(void);
extern BOOL LookupAPSAddress(LONG_ADDR *);
#if defined(I_SUPPORT_BINDINGS)
    extern void RemoveAllBindings(SHORT_ADDR);
#endif

//******************************************************************************
// Application Variables
//******************************************************************************
NETWORK_DESCRIPTOR  *currentNetworkDescriptor;
NETWORK_DESCRIPTOR  *NetworkDescriptor;
ZIGBEE_PRIMITIVE    currentPrimitive;
BYTE                orphanTries;
#ifdef I_SUPPORT_SECURITY
    extern KEY_VAL	KeyVal;
    #ifdef USE_EXTERNAL_NVM
        extern WORD trustCenterLongAddr;
        extern NETWORK_KEY_INFO plainSecurityKey[2];
    #else
        extern ROM LONG_ADDR trustCenterLongAddr;
    #endif
#endif

BYTE AllowJoin = ENABLE;
//DKCS: Kiem tra LQI theo cong suat phat
BYTE TxPower;
BYTE SetTxPower = CLEAR;
WORD_VAL MSGPacketCount;
BYTE energy_level;
extern NWK_STATUS nwkStatus;

//variables has modified by dat_a3cbq91
#if defined(ENERGY_TRACKING)
    WORD Energy_Level;//Luu tru ket qua do dien ap nguon @dat_a3cbq91
#endif

#if defined(ENERGY_TRACKING) && defined(USE_MQ6)
    WORD ADC_result;//Luu tru gia tri lay mau tu ADC module @dat_a3cbq91
#endif

#if defined(USE_SHT10)
    WORD humidity, temperature;//Luu tru gia tri do am, nhiet do lay tu SHT chua qua xu ly @dat_a3cbq91
#endif
#if defined(USE_CONTROL_PUMP)
    volatile BYTE CmdPump;
#endif
#if defined(ROUTER_EMB)
    volatile BYTE counter = CLEAR,EnableSendCmd = DISABLE, EnableSendConfirm = DISABLE;
    volatile BYTE Val[6] = {0,0,0,0,0,0};
    BYTE Addr_MSB,Addr_LSB,Cmd;
#endif

BYTE    i;
WORD TimerCount = 0;//Chu ki gui lenh bang: TimerCount * TIMER_UNIT (s)
WORD _Tpass = 0;

BYTE confirmCmd, pre_confirmCmd;
static union
{
    struct
    {
                WORD MQ6Warning             : 1;//when has smoke, set this flag
                WORD LowPowerWarning        : 1;//when voltage drop-out, set this flag
                WORD EnableGetDataHTE       : 1;//when need to print humi-temp data on terminal, set this flag
                WORD HTE_Data_Ready         : 1;
                WORD PrintNeighborTable     : 1;
                WORD MicrowaveDetecting     : 1;
                WORD PumpAckReceive         : 1;

        #if defined(USE_MQ6)||defined(ENERGY_TRACKING)
            #if defined(USE_MQ6) && defined(ENERGY_TRACKING)
                WORD MQ6orVoltage           : 2;
                WORD Reserve                : 7;
            #else
                WORD CompleteADC            : 1;
                WORD Reserve                : 8;
            #endif
        #else
                WORD Reserve                : 9;
        #endif
    } bits;
    WORD Val;
} WSANFlags;
#define WSAN_FLAGS_INIT       0x0000
//end by dat_a3cbq91

//******************************************************************************
//******************************************************************************
//******************************************************************************
int main(void)
{
    /*******************************create by lab411****************************/
    OSCCON = 0x2001;//current osc HS, new osc FRC

    CLKDIV = 0x0000;//Fcy = Fosc/2

    /************Maping Pins************/
    //unlock registers
    asm volatile ("mov #OSCCONL, w1  \n"
        "mov #0x46, w2     \n"
        "mov #0x57, w3     \n"
        "mov.b w2, [w1]    \n"
        "mov.b w3, [w1]    \n"
        "bclr OSCCON, #6");

    // INT1 >>> RP21
    RPINR0bits.INT1R = 21;

    /*------SPI1------*/
    // SCK1 >> RP26
    RPINR20bits.SCK1R = 26;
    RPOR13bits.RP26R  = 8;
    // SDO1 >> RP19
    RPOR9bits.RP19R   = 7;
    // SDI1 >> RP27
    RPINR20bits.SDI1R = 27;

    /*------SPI2------*/
    // SCK2 >> RP3
    RPINR22bits.SCK2R = 3;
    RPOR1bits.RP3R    = 11;
    //SDO2 >> RP2
    RPOR1bits.RP2R    = 10;
    // SDI2 >> RP4
    RPINR22bits.SDI2R = 4;

    /*------UART1-----*/
    // RX1 >> RP22
    RPINR18bits.U1RXR = 22;
    // TX1 >> RP25
    RPOR12bits.RP25R  = 3;

    /*------UART2-----*/
    // RX2 >> RP24
    RPINR19bits.U2RXR = 24;
    // TX2 >> RP23
    RPOR11bits.RP23R  = 5;

    //lock register
    asm volatile ("mov #OSCCONL, w1  \n"
        "mov #0x46, w2     \n"
        "mov #0x57, w3     \n"
        "mov.b w2, [w1]    \n"
        "mov.b w3, [w1]    \n"
        "bset OSCCON, #6");
    /*********************************end by lab411*****************************/
    CLRWDT();
    ENABLE_WDT();

    NetworkDescriptor = NULL;
    orphanTries = 3;

    // If you are going to send data to a terminal, initialize the UART.
    #if defined(USE_USART) || defined(ROUTER_EMB)|| defined(USE_CONTROL_PUMP) || defined(USE_MicroWaveS)
	ConsoleInit();
        #ifdef USE_DEBUG
            printf("Init UART2\r\n");
        #endif
    #endif

    // Initialize the hardware - must be done before initializing ZigBee.
    HardwareInit();
    #if defined(USE_USART) || defined(ROUTER_EMB)
        printf("Init Hardware\r\n");
    #endif

    // Initialize the ZigBee Stack.
    ZigBeeInit();
    currentPrimitive = NO_PRIMITIVE;
    #if defined(USE_USART) || defined(ROUTER_EMB)
        printf("Zigbee Init\r\n");
    #endif

    // *************************************************************************
    // Perform any other initialization here
    // *************************************************************************

    #if defined(USE_USART) || defined(ROUTER_EMB)
        printf("-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_\r\n");
    /*****************DEFINE SENSOR NODE*****************/
        #if defined(SENSOR1)
            printf("ZigBee Router Sensor1");
        #elif defined(SENSOR2)
            printf("ZigBee Router Sensor2");
        #elif defined(SENSOR3)
            printf("ZigBee Router Sensor3");
        #elif defined(SENSOR4)
            printf("ZigBee Router Sensor4");
        #elif defined(SENSOR5)
            printf("ZigBee Router Sensor5");
        #elif defined(SENSOR6)
            printf("ZigBee Router Sensor6");
        #elif defined(SENSOR7)
            printf("ZigBee Router Sensor7");
        #elif defined(SENSOR8)
            printf("ZigBee Router Sensor8");
        #elif defined(SENSOR9)
            printf("ZigBee Router Sensor9");
        #elif defined(SENSOR10)
            printf("ZigBee Router Sensor10");

        /*****************DEFINE ACTOR NODE*****************/
        #elif defined(ACTOR1)
            printf("ZigBee Router Actor1");
        #elif defined(ACTOR2)
            printf("ZigBee Router Actor2");

        /****************DEFINE ROUTER_EMBOARD****************/
        #elif defined(ROUTER_EMB)
            printf("ZigBee Router_EMB");
        #else
            #error "Router not supported."
        #endif

        printf(" - R&D by WSAN-Lab411\r\n");
        #if (RF_CHIP == MRF24J40)
            printf("Transceiver-MRF24J40\r\n");
        #else
            printf("Transceiver-Unknown\r\n");
        #endif
    #endif

    // Enable interrupts to get everything going.
    RFIE = ENABLE;

    /* Initialize my status flags*/
    WSANFlags.Val = WSAN_FLAGS_INIT;

    /* Start the network anew each time Node is booted up */
    NWKClearNeighborTable();
    #if defined(I_SUPPORT_BINDINGS)
        ClearBindingTable();
    #endif

    /* Clear the Group Table */
    RemoveAllGroups();

    #if defined(I_SUPPORT_BINDINGS)
        RemoveAllBindings(macPIB.macShortAddress);
    #endif

    /* Clearing nwk status flags */
    nwkStatus.flags.Val = 0x00;

    while (1)
    {
        /* Clear the watch dog timer */
        CLRWDT();

        /* Determine which is the next ZigBee Primitive to operate on */
        ZigBeeTasks( &currentPrimitive );

        /* Process the next ZigBee Primitive */
        ProcessZigBeePrimitives();
        
        /* do any non ZigBee related tasks and then go back to ZigBee tasks */
        ProcessNONZigBeeTasks();
    }
}

void ProcessZigBeePrimitives(void)
{
    switch (currentPrimitive)
    {
        case NLME_ROUTE_DISCOVERY_confirm:
//DKCS: Bo cac dong thong bao
            #if defined(USE_USART) || defined(ROUTER_EMB)
                if (!params.NLME_ROUTE_DISCOVERY_confirm.Status)
                {
//                    printf("Route Reply OK\r\n" );
                }
                else
                {
//                    PrintChar( params.NLME_PERMIT_JOINING_confirm.Status );
//                    printf(" Route Reply Failed\r\n" );
                }
            #endif
            currentPrimitive = NO_PRIMITIVE;
            break;

        case NLME_NETWORK_DISCOVERY_confirm:
            currentPrimitive = NO_PRIMITIVE;
            if (!params.NLME_NETWORK_DISCOVERY_confirm.Status)
            {
                if (!params.NLME_NETWORK_DISCOVERY_confirm.NetworkCount)
                {
                    #if defined(USE_USART) || defined(ROUTER_EMB)
                        printf("No networks found. Trying again!\r\n" );
                    #endif
                }
                else
                {
                    // Save the descriptor list pointer so we can destroy it later.
                    NetworkDescriptor = params.NLME_NETWORK_DISCOVERY_confirm.NetworkDescriptor;

                    // Select a network to try to join.  We're not going to be picky right now...
                    currentNetworkDescriptor = NetworkDescriptor;

SubmitJoinRequest:
                    params.NLME_JOIN_request.PANId          = currentNetworkDescriptor->PanID;
                    params.NLME_JOIN_request.JoinAsRouter   = TRUE;
                    params.NLME_JOIN_request.RejoinNetwork  = FALSE;
                    #ifdef ROUTER_EMB
                        params.NLME_JOIN_request.PowerSource    = MAINS_POWERED;
                    #else
                        params.NLME_JOIN_request.PowerSource    = NOT_MAINS_POWERED;
                    #endif
                    params.NLME_JOIN_request.RxOnWhenIdle   = TRUE;
                    params.NLME_JOIN_request.MACSecurity    = FALSE;
                    params.NLME_JOIN_request.ExtendedPANID = currentNetworkDescriptor->ExtendedPANID;

                    currentPrimitive = NLME_JOIN_request;

                    #if defined(USE_USART) || defined(ROUTER_EMB)
                        printf("Network(s) found. Trying to join " );
                        PrintWord(params.NLME_JOIN_request.PANId.Val);
                        printf(" | ");
                        for(i=7;i ^ 0xFF;--i)
                            PrintChar(currentNetworkDescriptor->ExtendedPANID.v[i]);
                        printf("\r\n");
                    #endif
                }
            }
            else
            {
                #if defined(USE_USART) || defined(ROUTER_EMB)
                    PrintChar( params.NLME_NETWORK_DISCOVERY_confirm.Status );
                    printf(" Error finding network. Trying again...\r\n" );
                #endif
            }
            break;

        case NLME_JOIN_confirm:
            if (!params.NLME_JOIN_confirm.Status)
            {
                #if defined(USE_USART) || defined(ROUTER_EMB)
                    printf("Join OK!\r\n" );
                #endif


                // Free the network descriptor list, if it exists. If we joined as an orphan, it will be NULL.
                while (NetworkDescriptor)
                {
                    currentNetworkDescriptor = NetworkDescriptor->next;
                    nfree( NetworkDescriptor );
                    NetworkDescriptor = currentNetworkDescriptor;
                }

                // Start routing capability.
                params.NLME_START_ROUTER_request.BeaconOrder = MAC_PIB_macBeaconOrder;
                params.NLME_START_ROUTER_request.SuperframeOrder = MAC_PIB_macSuperframeOrder;
                params.NLME_START_ROUTER_request.BatteryLifeExtension = FALSE;
                currentPrimitive = NLME_START_ROUTER_request;
            }
            else
            {
                currentPrimitive = NO_PRIMITIVE;
                #if defined(USE_USART) || defined(ROUTER_EMB)
                    printf("Status: ");
                    PrintChar( params.NLME_JOIN_confirm.Status );
                #endif

                // If we were trying as an orphan, see if we have some more orphan attempts.
                if (ZigBeeStatus.flags.bits.bTryOrphanJoin)
                {
                    // If we tried to join as an orphan, we do not have NetworkDescriptor, so we do
                    // not have to free it.
                    #if defined(USE_USART) || defined(ROUTER_EMB)
                        printf(". Could not join as orphan. " );
                    #endif
                    orphanTries--;
                    if (orphanTries)
                    {
                        #if defined(USE_USART) || defined(ROUTER_EMB)
                            printf("Trying again!\r\n" );
                        #endif
                        
                    }
                    else
                    {
                        orphanTries = 3;
                        #if defined(USE_USART) || defined(ROUTER_EMB)
                            printf("Must try as new node!\r\n" );
                        #endif
                        ZigBeeStatus.flags.bits.bTryOrphanJoin = CLEAR;
                    }
                }
                else
                {
                    #if defined(USE_USART) || defined(ROUTER_EMB)
                        printf(". Could not join selected network. " );
                    #endif
                    currentNetworkDescriptor = currentNetworkDescriptor->next;
                    if (currentNetworkDescriptor)
                    {
                        #if defined(USE_USART) || defined(ROUTER_EMB)
                            printf("Trying next discovered network!\r\n" );
                        #endif
                        goto SubmitJoinRequest;
                    }
                    else
                    {
                        //Ran out of descriptors.  Free the network descriptor list, and fall
                        // through to try discovery again.
                        #if defined(USE_USART) || defined(ROUTER_EMB)
                            printf("Cleaning up and retrying discovery!\r\n" );
                        #endif
                        while (NetworkDescriptor)
                        {
                            currentNetworkDescriptor = NetworkDescriptor->next;
                            nfree( NetworkDescriptor );
                            NetworkDescriptor = currentNetworkDescriptor;
                        }
                    }
                }
            }
            break;

        case NLME_START_ROUTER_confirm:
            if (!params.NLME_START_ROUTER_confirm.Status)
            {
                #if defined(USE_USART) || defined(ROUTER_EMB)
                    printf("Router Started! Enabling joins...\r\n" );
                #endif
                params.NLME_PERMIT_JOINING_request.PermitDuration = 0xFF;   // No Timeout
                params.NLME_PERMIT_JOINING_request._updatePayload = TRUE;
                currentPrimitive = NLME_PERMIT_JOINING_request;
            }
            else
            {
                #if defined(USE_USART) || defined(ROUTER_EMB)
                    PrintChar( params.NLME_JOIN_confirm.Status );
                    printf(" Router start Failed. We cannot route frames\r\n" );
                #endif
                currentPrimitive = NLME_START_ROUTER_request;//request again
            }
            break;

        case NLME_PERMIT_JOINING_confirm:
            if (!params.NLME_PERMIT_JOINING_confirm.Status)
            {
                #ifdef ROUTER_EMB
                    printf("Old Network Address is: ");
                    PrintChar(PHYGetShortRAMAddr(0x04));
                    PrintChar(PHYGetShortRAMAddr(0x03));
                    printf(". Old Network Address is:");
                    PrintWord(macPIB.macShortAddress.Val);
                    // Thiet lap lai dia chi mang cua Router_EMB @dat_a3cbq91
                    macPIB.macShortAddress.v[1] = RouterEmboardAddrMSB;
                    macPIB.macShortAddress.v[0] = RouterEmboardAddrLSB;
                    // Set ShortAddr on Transceiver !
                    PHYSetShortRAMAddr(0x03, macPIB.macShortAddress.v[0]);
                    PHYSetShortRAMAddr(0x04, macPIB.macShortAddress.v[1]);
                    printf(". New Network Address is 0x0009\r\n");

                    WSAN_LED	= ON;
                    currentPrimitive = NO_PRIMITIVE;
                #else
                    #if defined(USE_USART)
                        printf("Joining permitted!\r\n");
//DKCS: Cac tham so can in ra
					printf("Time | SPNumber | RDNumber | Pt | LQI | Voltage \r\n");
                    #endif

                    // gui toi bo nhung cho biet no vua gia nhap mang
                    SendOneByte(MAC_LONG_ADDR_BYTE0, JOIN_CONFIRM_CLUSTER);
                    WSAN_LED = ON;
                    #if defined (ENERGY_TRACKING)
                        AD1CON1bits.SAMP = ON;
                    #endif
                #endif
            }
            else
            {
                #if defined(USE_USART) || defined(ROUTER_EMB)
                    PrintChar( params.NLME_PERMIT_JOINING_confirm.Status );
                    printf(" Join permission Failed. We cannot allow joins\r\n" );
                #endif
                currentPrimitive = NO_PRIMITIVE;
            }
            break;

        case NLME_JOIN_indication:
            /* For Zigbee 2006: If a new device with the same old longAddress address
             * joins the PAN, then make sure the old short address is no longer used and is
             * overwritten with the new shortAddress & longAddress combination
             */
            {  /* same long address check block */
                APS_ADDRESS_MAP currentAPSAddress1;
                currentAPSAddress1.shortAddr   =   params.NLME_JOIN_indication.ShortAddress;
                currentAPSAddress1.longAddr    =   params.NLME_JOIN_indication.ExtendedAddress;

                if(LookupAPSAddress(&params.NLME_JOIN_indication.ExtendedAddress) )
                {
                    for( i = 0; i ^ apscMaxAddrMapEntries; i++)
                    {
                        #ifdef USE_EXTERNAL_NVM
                            GetAPSAddress( &currentAPSAddress,  apsAddressMap + i * sizeof(APS_ADDRESS_MAP) );
                        #else
                            GetAPSAddress( &currentAPSAddress,  &apsAddressMap[i] );
                        #endif
                            if (!memcmp( (void *)&currentAPSAddress.longAddr, (void *)&currentAPSAddress1.longAddr, 8 ))
                            {
                                /* overwrite the old with the new short/long address combo  */
                                #ifdef USE_EXTERNAL_NVM
                                    PutAPSAddress( apsAddressMap + i * sizeof(APS_ADDRESS_MAP), &currentAPSAddress1);
                                #else
                                    PutAPSAddress( &apsAddressMap[i], &currentAPSAddress1 );
                                #endif
                            }
                    }   /* end for loop */
                }   /* end if */
            }   /* end address check block */
        
            #ifdef I_SUPPORT_SECURITY
                #ifdef I_AM_TRUST_CENTER
                {
                    BOOL AllowJoin = TRUE;
                    // decide if you allow this device to join
                    if( !AllowJoin )
                    {
                        // no need to set deviceAddress, since it is overlap with NLME_JOIN_indication
                        //params.NLME_LEAVE_request.DeviceAddress = params.NLME_JOIN_indication.ExtendedAddress;
                        params.NLME_LEAVE_request.RemoveChildren = TRUE;
                        currentPrimitive = NLME_LEAVE_request;
                        break;
                    }

                    #ifdef I_SUPPORT_SECURITY_SPEC
                        if( params.NLME_JOIN_indication.secureJoin )
                        {
                            BYTE i;
                            for(i = 0; i < 16; i++)
                            {
                                    KeyVal.v[i] = 0;
                            }
                            params.APSME_TRANSPORT_KEY_request.Key = &KeyVal;
                            params.APSME_TRANSPORT_KEY_request.TransportKeyData.NetworkKey.KeySeqNumber = 0;

                        }
                        else
                        {
                            BYTE i;
                            GetNwkActiveKeyNumber(&i);
                            #ifdef USE_EXTERNAL_NVM
                                currentNetworkKeyInfo = plainSecurityKey[i-1];
                            #else
                                GetNwkKeyInfo(&currentNetworkKeyInfo, (ROM void *)&(NetworkKeyInfo[i-1]));
                            #endif
                            params.APSME_TRANSPORT_KEY_request.Key = &(currentNetworkKeyInfo.NetKey);
                            params.APSME_TRANSPORT_KEY_request.TransportKeyData.NetworkKey.KeySeqNumber = currentNetworkKeyInfo.SeqNumber.v[0];
                        }
                    #else
                        #ifdef PRECONFIGURE_KEY
                        {
                            BYTE i;
                            for(i = 0; i < 16; i++)
                            {
                                KeyVal.v[i] = 0;
                            }
                            params.APSME_TRANSPORT_KEY_request.Key = &KeyVal;
                            params.APSME_TRANSPORT_KEY_request.TransportKeyData.NetworkKey.KeySeqNumber = 0;
                            params.APSME_TRANSPORT_KEY_request._UseSecurity = TRUE;
                        }
                        #else
                            if( params.NLME_JOIN_indication.secureJoin )
                            {
                                BYTE i;
                                for(i = 0; i < 16; i++)
                                {
                                    KeyVal.v[i] = 0;
                                }
                                params.APSME_TRANSPORT_KEY_request.Key = &KeyVal;
                                params.APSME_TRANSPORT_KEY_request.TransportKeyData.NetworkKey.KeySeqNumber = 0;
                                params.APSME_TRANSPORT_KEY_request._UseSecurity = TRUE;
                            }
                            else
                            {
                                BYTE i;
                                GetNwkActiveKeyNumber(&i);
                                #ifdef USE_EXTERNAL_NVM
                                    currentNetworkKeyInfo = plainSecurityKey[i-1];
                                #else
                                    GetNwkKeyInfo(&currentNetworkKeyInfo, (ROM void *)&(NetworkKeyInfo[i-1]));
                                #endif
                                params.APSME_TRANSPORT_KEY_request.Key = &(currentNetworkKeyInfo.NetKey);
                                params.APSME_TRANSPORT_KEY_request.TransportKeyData.NetworkKey.KeySeqNumber = currentNetworkKeyInfo.SeqNumber.v[0];
                                params.APSME_TRANSPORT_KEY_request._UseSecurity = FALSE;
                            }
                        #endif
                    #endif
                    params.APSME_TRANSPORT_KEY_request.KeyType = ID_NetworkKey;
                    params.APSME_TRANSPORT_KEY_request.DestinationAddress = params.NLME_JOIN_indication.ExtendedAddress;
                    params.APSME_TRANSPORT_KEY_request.TransportKeyData.NetworkKey.UseParent = FALSE;
                    currentPrimitive = APSME_TRANSPORT_KEY_request;
                }
                #else
                    #ifdef I_SUPPORT_SECURITY_SPEC
                        params.APSME_UPDATE_DEVICE_request.Status = (params.NLME_JOIN_indication.secureJoin ) ? 0x00 : 0x01;
                    #else
                        #ifdef PRECONFIGURE_KEY
                            params.APSME_UPDATE_DEVICE_request.Status = 0x00;
                        #else
                            params.APSME_UPDATE_DEVICE_request.Status = 0x01;
                        #endif
                    #endif
                    params.APSME_UPDATE_DEVICE_request.DeviceShortAddress = params.NLME_JOIN_indication.ShortAddress;
                    params.APSME_UPDATE_DEVICE_request.DeviceAddress = params.NLME_JOIN_indication.ExtendedAddress;
                    GetTrustCenterAddress(&params.APSME_UPDATE_DEVICE_request.DestAddress);
                    for(i=0; i < 8; i++)
                        params.APSME_UPDATE_DEVICE_request.DestAddress.v[i] = 0xaa;

                    currentPrimitive = APSME_UPDATE_DEVICE_request;
                #endif
            #else
                currentPrimitive = NO_PRIMITIVE;
            #endif
            break;

        case NLME_LEAVE_indication:
            {
                LONG_ADDR myLongAddr;

                GetMACAddress(&myLongAddr);
                if(!memcmppgm2ram( &params.NLME_LEAVE_indication.DeviceAddress, &myLongAddr, 8 ))
                {
                    #if defined(USE_USART) || defined(ROUTER_EMB)
                    printf("We have left the network\r\n" );
                    #endif
                }
                else
                {
                    #if defined(USE_USART) || defined(ROUTER_EMB)
                    printf("Another node has left the network\r\n" );
                    #endif
                }
            }
            #ifdef I_SUPPORT_SECURITY
            {
                SHORT_ADDR	LeavingChildShortAddress;
                if( !APSFromLongToShort(&params.NLME_LEAVE_indication.DeviceAddress) )
                {
                    currentPrimitive = NO_PRIMITIVE;
                    break;
                }
                LeavingChildShortAddress = currentAPSAddress.shortAddr;

                #ifdef I_AM_TRUST_CENTER
                    params.APSME_UPDATE_DEVICE_indication.Status = 0x02;
                    params.APSME_UPDATE_DEVICE_indication.DeviceAddress = params.NLME_LEAVE_indication.DeviceAddress;
                    GetMACAddress(&params.APSME_UPDATE_DEVICE_indication.SrcAddress);
                    params.APSME_UPDATE_DEVICE_indication.DeviceShortAddress = LeavingChildShortAddress;
                    currentPrimitive = APSME_UPDATE_DEVICE_indication;
                    break;
                #else
                    params.APSME_UPDATE_DEVICE_request.Status = 0x02;
                    GetTrustCenterAddress(&params.APSME_UPDATE_DEVICE_request.DestAddress);
                    params.APSME_UPDATE_DEVICE_request.DeviceShortAddress = LeavingChildShortAddress;
                    currentPrimitive = APSME_UPDATE_DEVICE_request;
                    break;
                #endif
            }

            #else
                currentPrimitive = NO_PRIMITIVE;
            #endif
            break;

        case NLME_RESET_confirm:
            #if defined(USE_USART) || defined(ROUTER_EMB)
                printf("ZigBee Stack has been reset\r\n" );
            #endif
            /* For Zigbee 2006 The Specs require that node needs to
             * try joining as an orphan first after each reset,
             * see Mandatory test 3.9
             */
            ZigBeeStatus.flags.bits.bTryOrphanJoin = SET;

            currentPrimitive = NO_PRIMITIVE;
            break;

        case NLME_LEAVE_confirm:
            #if defined(USE_USART) || defined(ROUTER_EMB)
                PrintChar(params.NLME_LEAVE_confirm.Status);
                printf(" Leaving the Zigbee network\r\n" );
            #endif

            currentPrimitive = NO_PRIMITIVE;
            break;

        case APSDE_DATA_indication:
        #ifdef ROUTER_EMB
            {
                currentPrimitive = NO_PRIMITIVE;

                switch (params.APSDE_DATA_indication.DstEndpoint)
                {
                    // ************************************************************************
                    // Place a case for each user defined endpoint.
                    // ************************************************************************
                    case WSAN_Endpoint:
                    {
                        switch(params.APSDE_DATA_indication.ClusterId.Val)
                        {
   
  

                            /* Place other Cluster.ID cases here */
                            case ACTOR_RESPONSE_CLUSTER:
                            {
                                printf("#OK:");
                                PrintChar(APLGet());
                                printf("\r\n");
                            }
                                break;
							case LQI_RESPONSE_CLUSTER:
							{
							printf("LQI do duoc tai node nhan:");
							PrintChar(APLGet());
							printf("\r\n");
							}
							break;

                            default:
                                break;
                        }   /* switch 1*/

                        if( currentPrimitive != APSDE_DATA_request )
                            TxData = TX_DATA_START;
                    }           /* if msg */
                        break;

                    default:
                        break;
                }
                APLDiscardRx();
            }
            break;
        #else
            {
                BYTE data;

                currentPrimitive = NO_PRIMITIVE;

                switch (params.APSDE_DATA_indication.DstEndpoint)
                {
                    case WSAN_Endpoint:
                    {
                        switch (params.APSDE_DATA_indication.ClusterId.Val)
                        {
                            //user clusterID application
                          #if defined(USE_SHT10)
                            case HTE_REQUEST_CLUSTER://gui du lieu nhiet do do am ma emboard yeu cau
                            {
                            if (APLGet() == 0xff) MessageSuccessfulyCount++;  
                            }
                                break;
                            #endif

                            default:
                                /* Catch all place for all none ZDO msgs not processed above */
                                #if defined(USE_USART)
                                    printf("Got message...\r\n");
                                #endif
                                break;
                        }
                        if (currentPrimitive != APSDE_DATA_request)
                        TxData = TX_DATA_START;//reset lai chi so TxData
                    }
                        break;

                    default:
                        break;
                }
                APLDiscardRx();
            }
            break;
        #endif

        case APSDE_DATA_confirm:
//DKCS: Bo cac dong thong bao
            if (params.APSDE_DATA_confirm.Status)
            {
//                #if defined(USE_USART) || defined(ROUTER_EMB)
//                    PrintChar(params.APSDE_DATA_confirm.Status);
//                    printf(" Error sending message\r\n");
//                #endif
            }
            else
            {
			ReceiveDataCout++;
//                #if defined(USE_USART) || defined(ROUTER_EMB)
//                    printf("Sending message OK!\r\n" );
//                #endif
            }
            currentPrimitive = NO_PRIMITIVE;
            break;

        case APSME_ADD_GROUP_confirm:
        case APSME_REMOVE_GROUP_confirm:
        case APSME_REMOVE_ALL_GROUPS_confirm:
            #if defined(USE_USART) || defined(ROUTER_EMB)
                printf("Perform Group Operation\r\n" );
            #endif
            currentPrimitive = NO_PRIMITIVE;
            break;

        /* if nothing to process first check to see if we are in startup seqence */
        case NO_PRIMITIVE:
            if (AllowJoin && (!ZigBeeStatus.flags.bits.bNetworkJoined))
            {
                if (!ZigBeeStatus.flags.bits.bTryingToJoinNetwork)
                {
                    if (ZigBeeStatus.flags.bits.bTryOrphanJoin)
                    {
                        #if defined(USE_USART) || defined(ROUTER_EMB)
                            printf("Trying to join network as an orphan...\r\n" );
                        #endif

                        params.NLME_JOIN_request.ScanDuration     = 8;
                        params.NLME_JOIN_request.ScanChannels.Val = ALLOWED_CHANNELS;
                        params.NLME_JOIN_request.JoinAsRouter     = TRUE;
                        params.NLME_JOIN_request.RejoinNetwork    = 0x01;
                        #ifdef ROUTER_EMB
                            params.NLME_JOIN_request.PowerSource  = MAINS_POWERED;
                        #else
                            params.NLME_JOIN_request.PowerSource  = NOT_MAINS_POWERED;
                        #endif
                        params.NLME_JOIN_request.RxOnWhenIdle     = TRUE;
                        params.NLME_JOIN_request.MACSecurity      = FALSE;

                        params.NLME_JOIN_request.ExtendedPANID    = currentNetworkDescriptor->ExtendedPANID;
                        currentPrimitive = NLME_JOIN_request;
                    }
                    else
                    {
                        #if defined(USE_USART) || defined(ROUTER_EMB)
                            printf("Trying to join network as a new device!\r\n" );
                        #endif
                        params.NLME_NETWORK_DISCOVERY_request.ScanDuration          = 8;
                        params.NLME_NETWORK_DISCOVERY_request.ScanChannels.Val      = ALLOWED_CHANNELS;
                        currentPrimitive = NLME_NETWORK_DISCOVERY_request;
                    }
                }
            }
            else
            {
            #ifdef ROUTER_EMB
            #else
                if(!ZigBeeStatus.flags.bits.bHasBackgroundTasks)
                {
                    // ************************************************************************
                    // Place all processes that can send messages here.  Be sure to call
                    // ZigBeeBlockTx() when currentPrimitive is set to APSDE_DATA_request.
                    // ************************************************************************
//DKCS
#if defined(AODV_POWER_CONTROL)
					if ((SendDataEnable)&&(ZigBeeReady()))
					{
					SendOneByte(0xff,HTE_REQUEST_CLUSTER);
					SendDataCount++;
					SendDataEnable = CLEAR;
					PrintWord(TimerCount);
					printf(" | ");
					PrintWord(SendDataCount);
					printf("     | ");
					PrintWord(ReceiveDataCout);
					printf("     | ");		
					PrintChar(routeDiscoveryTablePointer[rdIndex]->indexforwardPt);
					printf(" | ");
					PrintChar(params.MCPS_DATA_indication.mpduLinkQuality);
					printf("  | ");
					PrintChar(Energy_Level);
					printf("\r\n");
					} 
#endif
                  }
            #endif
            }

            #ifdef ROUTER_EMB
//DKCS: Kiem tra LQI theo TxPower
			if (SetTxPower) 
            {
				PHYSetOutputPower(TxPower);
				printf("Da thiet lap cong suat phat:");
				PrintChar(TxPower);
				printf("\r\n");
				SendOneByte(0xAA, TEST_LQI_CLUSTER);
				SetTxPower = CLEAR;
			}
 
            if(WSANFlags.bits.PrintNeighborTable)
            {
                WSANFlags.bits.PrintNeighborTable = CLEAR;
                #ifdef USE_EXTERNAL_NVM
                    pCurrentNeighborRecord = neighborTable;   //+ (WORD)neighborIndex * (WORD)sizeof(NEIGHBOR_RECORD);
                #else
                    pCurrentNeighborRecord = &(neighborTable[0]);
                #endif
                printf("Short MAC Type Rntlship ");
                for ( i=0; i < MAX_NEIGHBORS; i++ )
                {
                    GetNeighborRecord( &currentNeighborRecord, pCurrentNeighborRecord );
                    if ((currentNeighborRecord.deviceInfo.bits.bInUse))
                    {
                        BYTE z;
                        printf("\r\n");
                        PrintWord(currentNeighborRecord.shortAddr.Val);
                        printf("|");

                        for(z=7; z ^ 0xFF; --z)
                           PrintChar(currentNeighborRecord.longAddr.v[z]);

                        printf("|");
                        switch(currentNeighborRecord.deviceInfo.bits.deviceType)
                        {
                            case 0x00:
                                printf("CRD");
                                break;

                            case 0x01:
                                printf("RTR");
                                break;

                            case 0x02:
                                printf("RFD");
                                break;

                            default:
                                printf("UKWN");
                                break;
                        }

                        printf("|");
                        switch(currentNeighborRecord.deviceInfo.bits.Relationship)
                        {
                            case 0x00:
                                printf("PARENT");
                                break;
                            case 0x01:
                                printf("CHILD ");
                                break;
                            default:
                                printf("UNKWN ");
                                break;
                        }
                    }
                    #ifdef USE_EXTERNAL_NVM
                        pCurrentNeighborRecord += (WORD)sizeof(NEIGHBOR_RECORD);
                    #else
                        pCurrentNeighborRecord++;
                    #endif
                }
                printf("\r\n");
            }
            #endif
            break;

        default:
            #if defined(USE_USART) || defined(ROUTER_EMB)
                PrintChar( currentPrimitive );
                printf(" Unhandled primitive\r\n" );
            #endif
            currentPrimitive = NO_PRIMITIVE;
            break;
    }
}

void ProcessNONZigBeeTasks(void)
{
    // *********************************************************************
    // Place any non-ZigBee related processing here.  Be sure that the code
    // will loop back and execute ZigBeeTasks() in a timely manner.
    // *********************************************************************

}

/*******************************************************************************
HardwareInit

All port directioning and SPI must be initialized before calling ZigBeeInit().

For demonstration purposes, required signals are configured individually.
*******************************************************************************/
void HardwareInit(void)
{
    //Digital pins @ added by dat_a3cbq91
    ANSE = 0x0000;
    ANSG = 0x0000;
    ANSD = 0x0000;

    #if(CLOCK_FREQ < 1000000)
        //SPI1 for RF transceiver
        SPI1CON1 = 0b0000000100111111;      // CLOCK_FREQ as SPI CLOCK
        SPI1STAT = 0x8000;

        //SPI2 for EEPROM
        SPI2CON1 = 0b0000000100111111;      // CLOCK_FREQ as SPI CLOCK
        SPI2STAT = 0x8000;
    #else
        //comment by dat_a3cbq91
        /* SPI1 for RF transceiver */
        SPI1CON1 = 0b0000000100111110;  //PIC24FJ128GA306 is Master, MRF24J40 is Slaver
                                        //Internal SPI clock is enabled
                                        //SDO1 pin is controlled by the module
                                        //Communication is byte-wide (8 bits)
                                        //Input data is sampled at the middle of data output time
                                        //Serial output data changes on transition from active clock state to Idle clock state
                                        //~SS1 pin is not used by the module; pin is controlled by the port function
                                        //Idle state for the clock is a low level; active state is a high level
                                        //Secondary prescale 1:1
                                        //Primary prescale 4:1 -> CLOCK_FREQ/2/4 as SPI CLOCK

        //Enables module and configures SCKx, SDOx, SDIx and ~SSx as serial port pins
        //Interrupt when the last data in the receive buffer is read; as a result, the buffer is empty (SRXMPT bit is set)
        SPI1STAT = 0x8000;

        /* SPI2 for EEPROM */
        //same as configuration SPI1 module
        SPI2CON1 = 0b0000000100111110;
        SPI2STAT = 0x8000;
    #endif

    #ifdef USE_EXTERNAL_NVM
    	EEPROM_nCS	= 1;
    	EEPROM_nCS_TRIS	= 0;
    	IFS2bits.SPI2IF = 1;
    #endif

    PHY_RESETn = 0;
    PHY_RESETn_TRIS = 0;
    PHY_CS = 1;
    PHY_CS_TRIS = 0;

    RFIF = 0;
    RFIE = 1;

    if(RF_INT_PIN == 0)
    {
        RFIF = 1;
    }

    //configure pin which is attached by led is output
    WSAN_LED_TRIS = CLEAR;
    //Clear Led to a initialize state
    WSAN_LED = CLEAR;
    
    #ifndef ROUTER_EMB
     //--------------------------------------------------------------------------
    // This section is required for application-specific hardware initialization
    //--------------------------------------------------------------------------
    #if defined(ENERGY_TRACKING) || defined(USE_MQ6)
        //input analog at AN8
        REF_VOLTAGE_TRIS = SET;
        REF_ANALOG = ENABLE;

        //Enable supply voltage to Zener Diode
        TRISBbits.TRISB10 = 0;
        LATBbits.LATB10 = 1;

        //configure ADC module
        AD1CON1 = 0x0470;//12-bit A/D operation
                        //Output format bit is absolute decimal result, unsigned, right-justified
                        //Internal counter ends sampling and starts conversion
                        //Sampling begins when SAMP bit is manually set
        #if defined(USE_MQ6)
            AD1CON2 = 0x4800;//external positive voltage reference --> for measure signal from smoke sensor
                            //Conversion result is loaded into the buffer ADC1BUF8
                            //Interrupt at the completion of conversion for each sample
                            //Always uses channel input selects for Sample A
        #else
            AD1CON2 = 0x0800;//VDD is voltage reference --> for measure voltage lever of power
        #endif
        AD1CON3 = 0x1F02;//Sample Time = 31Tad, Tad = 3Tcy
        AD1CHS = 0x0008;//channel AN8

        AD1CON1bits.ADON = 1;
        //ADC interrupt
        IFS0bits.AD1IF = 0;
        IEC0bits.AD1IE = 1;
        IPC3bits.AD1IP = 6;
    #endif

    // ----------------- Init SHT -----------------
    #if defined(USE_SHT10)
        // Make RG2, RG3 outputs.
        SCK_TRIS    =   CLEAR;
        DATA_TRIS   =   CLEAR;
    #endif

    // ---------- Init RX interrupt UART ----------
    #if defined(USE_MicroWaveS)
        PIE1bits.RCIE = ON;
        PIR1bits.RCIF = CLEAR;
    #endif
    #endif
}

/*******************************************************************************
User Interrupt Handler

The stack uses some interrupts for its internal processing. Once it is done
checking for its interrupts, the stack calls this function to allow for any
additional interrupt processing.
*******************************************************************************/
//this routine is written by dat_a3cbq91
void _ISR __attribute__((interrupt, auto_psv)) _T3Interrupt(void)
{
    DWORD timer = TIMER_UNIT;
    IFS0bits.T3IF = 0;
	TimerCount++;
    #if defined(ENERGY_TRACKING) || defined(USE_MQ6)
        AD1CON1bits.SAMP = ON;//start sampling for conversion
    #endif

    //re-assigned for PR3:PR2
    PR3 = timer >> 16;
    PR2 = timer & 0xFFFF;

    //if you don't want print neighbor of this node, comment the following
    //WSANFlags.bits.PrintNeighborTable = SET;
}

#if defined(ROUTER_EMB)
void _ISR __attribute__((interrupt, auto_psv)) _U2RXInterrupt(void)
{
    IFS1bits.U2RXIF = CLEAR;
    TxPower = U2RXREG;
	SetTxPower = SET;
}
#else

void _ISR __attribute__((interrupt, auto_psv)) _ADC1Interrupt(void)
{
    IFS0bits.AD1IF = CLEAR;
    //Lay gia tri chuyen doi ADC
    #if defined(USE_MQ6) && defined(ENERGY_TRACKING)
        //lay ket qua chuyen doi ADC
        ADC_result = ADC1BUF8;
        switch(WSANFlags.bits.MQ6orVoltage)
        {
            case MQ6_Mode_ADC:
                Mq6Signal = ADC_result;
                AD1CON2bits.PVCFG = Measure_Voltage;//chuyen sang do dien ap cua nguon
                break;

            case Power_Mode_ADC:
                Energy_Level = ADC_result;
                AD1CON2bits.PVCFG = Measure_MQ6;//chuyen sang do tin hieu tu MQ6
                break;

            default:
                break;
        }
        ++(WSANFlags.bits.MQ6orVoltage);
    #endif

    #if defined(USE_MQ6) && !defined(ENERGY_TRACKING)
        Mq6Signal = ADC1BUF8;
        WSANFlags.bits.CompleteADC = SET;
    #endif

    #if defined(ENERGY_TRACKING) && !defined(USE_MQ6)
        Energy_Level = ADC1BUF8;
        WSANFlags.bits.CompleteADC = SET;
    #endif
}
#endif
/*********************************************************************
 * Function:        void SendOnceByte(BYTE data, WORD ClusterID)
 *
 * PreCondition:    Init OK
 *
 * Input:           data, CluaterID
 *
 * Output:          None
 *
 * Side Effects:
 *
 * Overview:        Gui trang thai ve router-EMB. Cac trang thai co the
 *                  la tat/bat bom, chay rung, canh bao muc nang luong.
 *
 * Note:
 ********************************************************************/

void SendOneByte(BYTE data, WORD ClusterID)
{
    //cau truc ban tin gui tu router-emboard toi cac node khac: DD
    TxBuffer[TxData++] = data;

    params.APSDE_DATA_request.DstAddrMode = APS_ADDRESS_16_BIT;
    // Address of destination node
    params.APSDE_DATA_request.DstAddress.ShortAddr.v[1] = RouterEmboardAddrMSB;
    params.APSDE_DATA_request.DstAddress.ShortAddr.v[0] = RouterEmboardAddrLSB;

    params.APSDE_DATA_request.RadiusCounter = DEFAULT_RADIUS;
//    params.APSDE_DATA_request.DiscoverRoute = ROUTE_DISCOVERY_ENABLE;
    params.APSDE_DATA_request.DiscoverRoute = ROUTE_DISCOVERY_SUPPRESS;

    #ifdef I_SUPPORT_SECURITY
        params.APSDE_DATA_request.TxOptions.Val = SET;
    #else
        params.APSDE_DATA_request.TxOptions.Val = CLEAR;
    #endif

    params.APSDE_DATA_request.TxOptions.bits.acknowledged = SET;

    params.APSDE_DATA_request.SrcEndpoint = WSAN_Endpoint;
    params.APSDE_DATA_request.DstEndpoint = WSAN_Endpoint;
    params.APSDE_DATA_request.ProfileId.Val = MY_PROFILE_ID;
    params.APSDE_DATA_request.ClusterId.Val = ClusterID;

    //make ZigbeeIsReady() false
    ZigBeeBlockTx();

    currentPrimitive = APSDE_DATA_request;
}
