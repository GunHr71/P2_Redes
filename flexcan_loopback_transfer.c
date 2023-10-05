/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_ftm.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_CAN            CAN0
#define EXAMPLE_CAN_CLK_SOURCE (kFLEXCAN_ClkSrc1)
#define EXAMPLE_CAN_CLK_FREQ   CLOCK_GetFreq(kCLOCK_BusClk)
/* Set USE_IMPROVED_TIMING_CONFIG macro to use api to calculates the improved CAN / CAN FD timing values. */
#define USE_IMPROVED_TIMING_CONFIG (1U)
#define RX_MESSAGE_BUFFER_NUM      (9)
#define TX_MESSAGE_BUFFER_NUM      (8)
#define DLC                        (8)

/* Fix MISRA_C-2012 Rule 17.7. */
#define LOG_INFO (void)PRINTF

//FTM
#define FTM_BASEADDR FTM0
#define FTM_CHANNEL  kFTM_Chnl_1
#define PWM_FREQUENCY (24000U)
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#define FTM_CHANNEL_INTERRUPT_ENABLE kFTM_Chnl1InterruptEnable
#define FTM_CHANNEL_FLAG             kFTM_Chnl1Flag

#define RED_LED_GPIO     GPIOB
#define RED_LED_GPIO_PIN 22

#define BLUE_LED_GPIO     GPIOB
#define BLUE_LED_GPIO_PIN 21

#define GREEN_LED_GPIO     GPIOE
#define GREEN_LED_GPIO_PIN 26

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool txComplete = false;
volatile bool rxComplete = false;
flexcan_handle_t flexcanHandle;
flexcan_mb_transfer_t txXfer, rxXfer;
#if (defined(USE_CANFD) && USE_CANFD)
flexcan_fd_frame_t txFrame, rxFrame;
#else
flexcan_frame_t txFrame, rxFrame;
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief FlexCAN Call Back function
 */
void delay(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool ftmIsrFlag          = false;
volatile bool brightnessUp        = true; /* Indicate LED is brighter or dimmer */
volatile uint8_t updatedDutycycle = 10U;

/******************** ***********************************************************
 * Code
 ******************************************************************************/
void delay(void)
{
    volatile uint32_t i = 0U;
    for (i = 0U; i < 40000U; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

void FTM0_IRQHandler(void)
{
    ftmIsrFlag = true;

    if (brightnessUp)
    {
        /* Increase duty cycle until it reach limited value, don't want to go upto 100% duty cycle
         * as channel interrupt will not be set for 100%
         */
        if (++updatedDutycycle >= 99U)
        {
            updatedDutycycle = 99U;
            brightnessUp     = false;
        }
    }
    else
    {
        /* Decrease duty cycle until it reach limited value */
        if (--updatedDutycycle == 1U)
        {
            brightnessUp = true;
        }
    }

    /* Clear interrupt flag.*/
    FTM_ClearStatusFlags(FTM_BASEADDR, FTM_GetStatusFlags(FTM_BASEADDR));

    __DSB();
}

typedef enum{
    RED,
    YELLOW,
    GREEN,
    CYAN,
    BLUE,
    PURPLE,
    WHITE,
    NO_COLOR
}color_t;

static FLEXCAN_CALLBACK(flexcan_callback)
{
    switch (status)
    {
        /* Process FlexCAN Rx event. */
        case kStatus_FLEXCAN_RxIdle:
            if (RX_MESSAGE_BUFFER_NUM == result)
            {
                rxComplete = true;
            }
            break;

        /* Process FlexCAN Tx event. */
        case kStatus_FLEXCAN_TxIdle:
            if (TX_MESSAGE_BUFFER_NUM == result)
            {
                txComplete = true;
            }
            break;

        default:
            break;
    }
}
void RGB_color(color_t color);

/*!
 * @brief Main function
 */
int main(void)
{
    flexcan_config_t flexcanConfig;
    flexcan_rx_mb_config_t mbConfig;
    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam;
    ftm_pwm_level_select_t pwmLevel = kFTM_HighTrue;
    gpio_pin_config_t red_led_config = {
        kGPIO_DigitalOutput,
        0,
    };
    gpio_pin_config_t green_led_config = {
        kGPIO_DigitalOutput,
        0,
    };
    gpio_pin_config_t blue_led_config = {
        kGPIO_DigitalOutput,
        0,
    };

    /* Initialize board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    GPIO_PinInit(RED_LED_GPIO, RED_LED_GPIO_PIN, &red_led_config);
    GPIO_PinInit(BLUE_LED_GPIO, BLUE_LED_GPIO_PIN, &green_led_config);
    GPIO_PinInit(GREEN_LED_GPIO, GREEN_LED_GPIO_PIN, &blue_led_config);

    LOG_INFO("\r\n==FlexCAN loopback example -- Start.==\r\n\r\n");
    //PWM INIT
    /* Fill in the FTM config struct with the default settings */
    FTM_GetDefaultConfig(&ftmInfo);
    /* Calculate the clock division based on the PWM frequency to be obtained */
    ftmInfo.prescale = FTM_CalculateCounterClkDiv(FTM_BASEADDR, PWM_FREQUENCY, FTM_SOURCE_CLOCK);
    /* Initialize FTM module */
    FTM_Init(FTM_BASEADDR, &ftmInfo);

    /* Configure ftm params with frequency 24kHZ */
    ftmParam.chnlNumber            = FTM_CHANNEL;
    ftmParam.level                 = pwmLevel;
    ftmParam.dutyCyclePercent      = updatedDutycycle;
    ftmParam.firstEdgeDelayPercent = 0U;
    ftmParam.enableComplementary   = false;
    ftmParam.enableDeadtime        = false;
    if (kStatus_Success !=
        FTM_SetupPwm(FTM_BASEADDR, &ftmParam, 1U, kFTM_CenterAlignedPwm, PWM_FREQUENCY, FTM_SOURCE_CLOCK))
    {
        PRINTF("\r\nSetup PWM fail, please check the configuration parameters!\r\n");
        return -1;
    }

    FTM_StartTimer(FTM_BASEADDR, kFTM_SystemClock);


    //PWM INIT
    LOG_INFO("\r\n==FlexCAN loopback example -- Start.==\r\n\r\n");

    /* Init FlexCAN module. */
    /*
     * flexcanConfig.clkSrc                 = kFLEXCAN_ClkSrc0;
     * flexcanConfig.baudRate               = 1000000U;
     * flexcanConfig.baudRateFD             = 2000000U;
     * flexcanConfig.maxMbNum               = 16;
     * flexcanConfig.enableLoopBack         = false;
     * flexcanConfig.enableSelfWakeup       = false;
     * flexcanConfig.enableIndividMask      = false;
     * flexcanConfig.disableSelfReception   = false;
     * flexcanConfig.enableListenOnlyMode   = false;
     * flexcanConfig.enableDoze             = false;
     */
    FLEXCAN_GetDefaultConfig(&flexcanConfig);
   // flexcanConfig.enableIndividMask = true;
   // FLEXCAN_SetRxIndividualMask(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, 0x1F);



#if defined(EXAMPLE_CAN_CLK_SOURCE)
    flexcanConfig.clkSrc = EXAMPLE_CAN_CLK_SOURCE;
#endif

    flexcanConfig.enableLoopBack = false;

#if (defined(USE_IMPROVED_TIMING_CONFIG) && USE_IMPROVED_TIMING_CONFIG)
    flexcan_timing_config_t timing_config;
    memset(&timing_config, 0, sizeof(flexcan_timing_config_t));
#if (defined(USE_CANFD) && USE_CANFD)
    if (FLEXCAN_FDCalculateImprovedTimingValues(EXAMPLE_CAN, flexcanConfig.baudRate, flexcanConfig.baudRateFD,
                                                EXAMPLE_CAN_CLK_FREQ, &timing_config))
    {
        /* Update the improved timing configuration*/
        memcpy(&(flexcanConfig.timingConfig), &timing_config, sizeof(flexcan_timing_config_t));
    }
    else
    {
        LOG_INFO("No found Improved Timing Configuration. Just used default configuration\r\n\r\n");
    }
#else
    if (FLEXCAN_CalculateImprovedTimingValues(EXAMPLE_CAN, flexcanConfig.baudRate, EXAMPLE_CAN_CLK_FREQ,
                                              &timing_config))
    {
        /* Update the improved timing configuration*/
        memcpy(&(flexcanConfig.timingConfig), &timing_config, sizeof(flexcan_timing_config_t));
    }
    else
    {
        LOG_INFO("No found Improved Timing Configuration. Just used default configuration\r\n\r\n");
    }
#endif
#endif

#if (defined(USE_CANFD) && USE_CANFD)
    FLEXCAN_FDInit(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ, BYTES_IN_MB, true);
#else
    FLEXCAN_Init(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ);
    FLEXCAN_SetRxMbGlobalMask(EXAMPLE_CAN, 0x000);//TODO mask
#endif

    /* Setup Rx Message Buffer. */
    mbConfig.format = kFLEXCAN_FrameFormatStandard;
    mbConfig.type   = kFLEXCAN_FrameTypeData;
    mbConfig.id     = FLEXCAN_ID_STD(0x04);
#if (defined(USE_CANFD) && USE_CANFD)
    FLEXCAN_SetFDRxMbConfig(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);
#else
    FLEXCAN_SetRxMbConfig(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);
#endif

/* Setup Tx Message Buffer. */
#if (defined(USE_CANFD) && USE_CANFD)
    FLEXCAN_SetFDTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);
#else
    FLEXCAN_SetTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);
#endif

    /* Create FlexCAN handle structure and set call back function. */
    FLEXCAN_TransferCreateHandle(EXAMPLE_CAN, &flexcanHandle, flexcan_callback, NULL);

    /* Start receive data through Rx Message Buffer. */
    rxXfer.mbIdx = (uint8_t)RX_MESSAGE_BUFFER_NUM;
#if (defined(USE_CANFD) && USE_CANFD)
    rxXfer.framefd = &rxFrame;
    (void)FLEXCAN_TransferFDReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxXfer);
#else
    rxXfer.frame = &rxFrame;
    (void)FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxXfer);
#endif

    /* Prepare Tx Frame for sending. */
    txFrame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
    txFrame.type   = (uint8_t)kFLEXCAN_FrameTypeData;
    txFrame.id     = FLEXCAN_ID_STD(0x123);
    txFrame.length = (uint8_t)DLC;
#if (defined(USE_CANFD) && USE_CANFD)
    txFrame.brs = 1U;
#endif
#if (defined(USE_CANFD) && USE_CANFD)
    uint8_t i = 0;
    for (i = 0; i < DWORD_IN_MB; i++)
    {
        txFrame.dataWord[i] = i;
    }
#else
    txFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0xAA) | CAN_WORD0_DATA_BYTE_1(0xAA) | CAN_WORD0_DATA_BYTE_2(0x33) |
                        CAN_WORD0_DATA_BYTE_3(0xAA);
    txFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0xAA) | CAN_WORD1_DATA_BYTE_5(0xAA) | CAN_WORD1_DATA_BYTE_6(0xAA) |
                        CAN_WORD1_DATA_BYTE_7(0xAA);
#endif

    LOG_INFO("Send message from MB%d to MB%d\r\n", TX_MESSAGE_BUFFER_NUM, RX_MESSAGE_BUFFER_NUM);
#if (defined(USE_CANFD) && USE_CANFD)
    for (i = 0; i < DWORD_IN_MB; i++)
    {
        LOG_INFO("tx word%d = 0x%x\r\n", i, txFrame.dataWord[i]);
    }
#else
//    LOG_INFO("tx word0 = 0x%x\r\n", txFrame.dataWord0);
//    LOG_INFO("tx word1 = 0x%x\r\n", txFrame.dataWord1);
//    LOG_INFO("id = 0x%x\r\n", txFrame.id);
#endif

    /* Send data through Tx Message Buffer. */
    txXfer.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
#if (defined(USE_CANFD) && USE_CANFD)
    txXfer.framefd = &txFrame;
    (void)FLEXCAN_TransferFDSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);
#else
    txXfer.frame = &txFrame;
    //(void)FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);
#endif

    /* Waiting for Rx Message finish. */
    while ((!rxComplete))
    {
    };

    LOG_INFO("\r\nReceived message from MB%d\r\n", RX_MESSAGE_BUFFER_NUM);
#if (defined(USE_CANFD) && USE_CANFD)
    for (i = 0; i < DWORD_IN_MB; i++)
    {
        LOG_INFO("rx word%d = 0x%x\r\n", i, rxFrame.dataWord[i]);
    }
#else
    LOG_INFO("rx word0 = 0x%x\r\n", rxFrame.dataWord0);
    LOG_INFO("rx word1 = 0x%x\r\n", rxFrame.dataWord1);
    LOG_INFO("rx id = 0x%x\r\n", rxFrame.id);

#endif

    LOG_INFO("\r\n==FlexCAN loopback example -- Finish.==\r\n");

    while (true)
    {
    	rxComplete = false;
    	FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxXfer);
        while ((!rxComplete))
        {
        };
        LOG_INFO("rx word0 = 0x%x\r\n", rxFrame.dataWord0);
        LOG_INFO("rx word1 = 0x%x\r\n", rxFrame.dataWord1);
        LOG_INFO("rx id = 0x%x\r\n", rxFrame.id);

        if(rxFrame.id >> 18 == 3)
        {
            updatedDutycycle = (rxFrame.dataWord0>>16 ) % 256;

            FTM_UpdateChnlEdgeLevelSelect(FTM_BASEADDR, FTM_CHANNEL, 0U);
            if (kStatus_Success !=
                FTM_UpdatePwmDutycycle(FTM_BASEADDR, FTM_CHANNEL, kFTM_CenterAlignedPwm, (updatedDutycycle%100)))
            {
                PRINTF("Update duty cycle fail, the target duty cycle may out of range!\r\n");
            }
            /* Software trigger to update registers */
            FTM_SetSoftwareTrigger(FTM_BASEADDR, true);
            /* Start channel output with updated dutycycle */
            FTM_UpdateChnlEdgeLevelSelect(FTM_BASEADDR, FTM_CHANNEL, pwmLevel);

            RGB_color(((rxFrame.dataWord0>>24) % 256) % NO_COLOR);
        }


    }
}

void RGB_color(color_t color)
{
	switch(color)
	{
		case RED:
			GPIO_PortClear(RED_LED_GPIO, 1u << RED_LED_GPIO_PIN );
			GPIO_PortSet(GREEN_LED_GPIO, 1u << GREEN_LED_GPIO_PIN );
			GPIO_PortSet(BLUE_LED_GPIO, 1u << BLUE_LED_GPIO_PIN );
		break;
		case YELLOW:
			GPIO_PortClear(RED_LED_GPIO, 1u << RED_LED_GPIO_PIN );
			GPIO_PortClear(GREEN_LED_GPIO, 1u << GREEN_LED_GPIO_PIN );
			GPIO_PortSet(BLUE_LED_GPIO, 1u << BLUE_LED_GPIO_PIN );
		break;
		case GREEN:
			GPIO_PortSet(RED_LED_GPIO, 1u << RED_LED_GPIO_PIN );
			GPIO_PortClear(GREEN_LED_GPIO, 1u << GREEN_LED_GPIO_PIN );
			GPIO_PortSet(BLUE_LED_GPIO, 1u << BLUE_LED_GPIO_PIN );
		break;
		case CYAN:
			GPIO_PortSet(RED_LED_GPIO, 1u << RED_LED_GPIO_PIN );
			GPIO_PortClear(GREEN_LED_GPIO, 1u << GREEN_LED_GPIO_PIN );
			GPIO_PortClear(BLUE_LED_GPIO, 1u << BLUE_LED_GPIO_PIN );
		break;
		case BLUE:
			GPIO_PortSet(RED_LED_GPIO, 1u << RED_LED_GPIO_PIN );
			GPIO_PortSet(GREEN_LED_GPIO, 1u << GREEN_LED_GPIO_PIN );
			GPIO_PortClear(BLUE_LED_GPIO, 1u << BLUE_LED_GPIO_PIN );
		break;
		case PURPLE:
			GPIO_PortClear(RED_LED_GPIO, 1u << RED_LED_GPIO_PIN );
			GPIO_PortSet(GREEN_LED_GPIO, 1u << GREEN_LED_GPIO_PIN );
			GPIO_PortClear(BLUE_LED_GPIO, 1u << BLUE_LED_GPIO_PIN );
		break;
		case WHITE:
			GPIO_PortClear(RED_LED_GPIO, 1u << RED_LED_GPIO_PIN );
			GPIO_PortClear(GREEN_LED_GPIO, 1u << GREEN_LED_GPIO_PIN );
			GPIO_PortClear(BLUE_LED_GPIO, 1u << BLUE_LED_GPIO_PIN );
		break;
		case NO_COLOR:
			GPIO_PortSet(RED_LED_GPIO, 1u << RED_LED_GPIO_PIN );
			GPIO_PortSet(GREEN_LED_GPIO, 1u << GREEN_LED_GPIO_PIN );
			GPIO_PortSet(BLUE_LED_GPIO, 1u << BLUE_LED_GPIO_PIN );
		break;
	}
}
