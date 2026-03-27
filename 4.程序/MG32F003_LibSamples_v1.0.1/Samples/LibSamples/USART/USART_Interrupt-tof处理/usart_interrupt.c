/**
 * @file    usart_interrupt.c
 * @author  MegawinTech Application Team
 * @version V1.0.1
 * @date    14-Nov-2023
 * @brief   This file contains all the system functions
 */

/* Define to prevent recursive inclusion */
#define _USART_INTERRUPT_C_

/* Files include */
#include <stdio.h>
#include "platform.h"
#include "usart_interrupt.h"

/**
  * @addtogroup MG32F003_LibSamples
  * @{
  */

/**
  * @addtogroup USART
  * @{
  */

/**
  * @addtogroup USART_Interrupt
  * @{
  */

/* Private typedef ****************************************************************************************************/

/* Private define *****************************************************************************************************/

/* Private macro ******************************************************************************************************/

/* Private variables **************************************************************************************************/
/* 全局变量定义 */
volatile USART_RxTx_TypeDef USART_RxStruct = {0};
volatile USART_RxTx_TypeDef USART_TxStruct = {0};

typedef struct
{
    uint16_t norm_tof;
    uint16_t norm_peak;
    uint32_t norm_noise;
    uint16_t multshot;
    uint16_t atten_peak;
    uint32_t atten_noise;
    uint16_t ref_tof;
    int32_t  temperature;   // 0.01℃
    int16_t  cal_tof;       // mm
    uint8_t  confidence;
} YC02_FrameData_t;

//== 测试帧
static const uint8_t yc02_test_frame[27] =
{
    0x5A, 0x8E, 0x17,
    0x37, 0x29,
    0x89, 0x6D,
    0xD4, 0x00, 0x00,
    0xB8, 0x01,
    0xDA, 0x14,
    0x09, 0x00, 0x00,
    0x46, 0x26,
    0xAB, 0x0F, 0x00, 0x00,
    0x17, 0x03,
    0x64,
    0x7D
};
/* Private functions **************************************************************************************************/

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void USART_Configure(uint32_t Baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2, ENABLE);

    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate   = Baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits   = USART_StopBits_1;
    USART_InitStruct.USART_Parity     = USART_Parity_No;
    USART_InitStruct.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStruct);

    USART_ITConfig(USART2, USART_IT_PE, ENABLE);
    USART_ITConfig(USART2, USART_IT_ERR, ENABLE);
    
    USART_Cmd(USART2, ENABLE);
}


/* 启动一次 IDLE 帧接收
 * MaxLength = 本帧最多允许接收多少字节，可自定义
 */
void USART_RxIdle_Start(uint16_t MaxLength)
{
    uint16_t i;

    if (MaxLength > USART_RX_BUFFER_SIZE)
    {
        MaxLength = USART_RX_BUFFER_SIZE;
    }

    for (i = 0; i < MaxLength; i++)
    {
        USART_RxStruct.Buffer[i] = 0;
    }

    USART_RxStruct.Length       = MaxLength;
    USART_RxStruct.CurrentCount = 0;
    USART_RxStruct.CompleteFlag = 0;
    USART_RxStruct.OverflowFlag = 0;

    /* 先清一次残留状态 */
    (void)USART_ReceiveData(USART2);
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    USART_ClearFlag(USART2, USART_FLAG_RXNE);

    /* 开启接收和空闲中断 */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
}


/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
//void USART_RxData_Interrupt(uint8_t Length)
//{
//    uint8_t i = 0;

//    for (i = 0; i < Length; i++)
//    {
//        USART_RxStruct.Buffer[i] = 0;
//    }

//    USART_RxStruct.Length = Length;
//    USART_RxStruct.CurrentCount = 0;
//    USART_RxStruct.CompleteFlag = 0;

//    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
//}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
//void USART_TxData_Interrupt(uint8_t *Buffer, uint8_t Length)
//{
//    uint8_t i = 0;

//    for (i = 0; i < Length; i++)
//    {
//        USART_TxStruct.Buffer[i] = Buffer[i];
//    }

//    USART_TxStruct.Length = Length;
//    USART_TxStruct.CurrentCount = 0;
//    USART_TxStruct.CompleteFlag = 0;

//    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
//}

static uint16_t read_le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t read_le24(const uint8_t *p)
{
    return (uint32_t)(p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16));
}

static int32_t read_le32s(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] |
                    ((uint32_t)p[1] << 8) |
                    ((uint32_t)p[2] << 16) |
                    ((uint32_t)p[3] << 24));
}

static uint8_t yc02_checksum(const uint8_t *buf, uint16_t len_without_checksum)
{
    uint16_t i;
    uint32_t sum = 0;

    for (i = 0; i < len_without_checksum; i++)
    {
        sum += buf[i];
    }

    return (uint8_t)(sum & 0xFF);
}

static uint8_t YC02_ParseFrame(const uint8_t *buf, uint16_t len, YC02_FrameData_t *out)
{
    if ((buf == 0) || (out == 0))
    {
        return 0;
    }

    /* 总长度应为27字节 */
    if (len != 27)
    {
        return 0;
    }

    if (buf[0] != 0x5A)
    {
        return 0;
    }

    if (buf[1] != 0x8E)
    {
        return 0;
    }

    if (buf[2] != 0x17)
    {
        return 0;
    }

    if (yc02_checksum(buf, 26) != buf[26])
    {
        return 0;
    }

    out->norm_tof    = read_le16(&buf[3]);   /* 3~4 */
    out->norm_peak   = read_le16(&buf[5]);   /* 5~6 */
    out->norm_noise  = read_le24(&buf[7]);   /* 7~9 */
    out->multshot    = read_le16(&buf[10]);  /* 10~11 */
    out->atten_peak  = read_le16(&buf[12]);  /* 12~13 */
    out->atten_noise = read_le24(&buf[14]);  /* 14~16 */
    out->ref_tof     = read_le16(&buf[17]);  /* 17~18 */
    out->temperature = read_le32s(&buf[19]); /* 19~22 */
    out->cal_tof     = (int16_t)read_le16(&buf[23]); /* 23~24 */
    out->confidence  = buf[25];              /* 25 */

    return 1;
}
/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/

//== 正常解析
void USART_Interrupt_Sample(void)
{
    uint16_t i = 0;
    YC02_FrameData_t frame;

    USART_Configure(460800);

    printf("\r\nTest %s", __FUNCTION__);

    USART_RxStruct.CompleteFlag = 0;
    USART_TxStruct.CompleteFlag = 1;

    USART_RxIdle_Start(30);

    printf("\r\nUSART2 idle rx start, max frame = 30 bytes");

    while (1)
    {
        if (USART_RxStruct.CompleteFlag)
        {
            printf("\r\nFrame done, len = %d", USART_RxStruct.CurrentCount);
            printf("\r\nData(hex): ");

            for (i = 0; i < USART_RxStruct.CurrentCount; i++)
            {
                printf("%02X ", USART_RxStruct.Buffer[i]);
            }

            if (YC02_ParseFrame((uint8_t *)USART_RxStruct.Buffer,
                                USART_RxStruct.CurrentCount,
                                &frame))
            {
                printf("\r\nParse OK");
                printf("\r\nnorm_tof    = %u", frame.norm_tof);
                printf("\r\nnorm_peak   = %u", frame.norm_peak);
                printf("\r\nnorm_noise  = %lu", (unsigned long)frame.norm_noise);
                printf("\r\nmultshot    = %u", frame.multshot);
                printf("\r\natten_peak  = %u", frame.atten_peak);
                printf("\r\natten_noise = %lu", (unsigned long)frame.atten_noise);
                printf("\r\nref_tof     = %u", frame.ref_tof);
                printf("\r\ntemperature = %ld (0.01C)", (long)frame.temperature);
                printf("\r\ntemperature = %ld.%02ld C",
                       (long)(frame.temperature / 100),
                       (long)(frame.temperature >= 0 ? (frame.temperature % 100) : -(frame.temperature % 100)));
                printf("\r\ncal_tof     = %d mm", frame.cal_tof);
                printf("\r\nconfidence  = %u", frame.confidence);
            }
            else
            {
                printf("\r\nParse FAIL");
            }

            printf("\r\n");

            USART_RxIdle_Start(30);
        }
    }
}


/**   
//== 打印测试帧
//== 测试结果应该是
    norm_tof    = 10551
    norm_peak   = 28041
    norm_noise  = 212
    multshot    = 440
    atten_peak  = 5338
    atten_noise = 9
    ref_tof     = 9798
    temperature = 40.11 C
    cal_tof     = 791 mm
    confidence  = 100
**/
//void USART_Interrupt_Sample(void)
//{
//    uint16_t i;
//    YC02_FrameData_t frame;

//    USART_Configure(460800);
//    printf("\r\nTest %s", __FUNCTION__);

//    printf("\r\nTest frame hex: ");
//    for (i = 0; i < sizeof(yc02_test_frame); i++)
//    {
//        printf("%02X ", yc02_test_frame[i]);
//    }
//    printf("\r\n");

//    if (YC02_ParseFrame(yc02_test_frame, sizeof(yc02_test_frame), &frame))
//    {
//        printf("\r\nParse OK");
//        printf("\r\nnorm_tof    = %u", frame.norm_tof);
//        printf("\r\nnorm_peak   = %u", frame.norm_peak);
//        printf("\r\nnorm_noise  = %lu", (unsigned long)frame.norm_noise);
//        printf("\r\nmultshot    = %u", frame.multshot);
//        printf("\r\natten_peak  = %u", frame.atten_peak);
//        printf("\r\natten_noise = %lu", (unsigned long)frame.atten_noise);
//        printf("\r\nref_tof     = %u", frame.ref_tof);
//        printf("\r\ntemperature = %ld.%02ld C",
//               (long)(frame.temperature / 100),
//               (long)((frame.temperature >= 0) ? (frame.temperature % 100) : -(frame.temperature % 100)));
//        printf("\r\ncal_tof     = %d mm", frame.cal_tof);
//        printf("\r\nconfidence  = %u", frame.confidence);
//    }
//    else
//    {
//        printf("\r\nParse FAIL");
//    }

//    while (1)
//    {
//    }
//}

//== 串口接收tof测试
//void USART_Interrupt_Sample(void)
//{
//    uint16_t i = 0;  
//    
//    USART_Configure(460800);
//    
//    printf("\r\nTest %s", __FUNCTION__);

//    USART_RxStruct.CompleteFlag = 0;
//    USART_TxStruct.CompleteFlag = 1;

//    /* 最大帧长 64 字节 */
//    USART_RxIdle_Start(30);

//    printf("\r\nUSART2 idle rx start, max frame = 32 bytes");

//    while (1)
//    {
//        //== 按字符串打印
////          if (USART_RxStruct.CompleteFlag)
////        {
////            USART_RxStruct.Buffer[USART_RxStruct.CurrentCount] = '\0';

////            printf("\r\nFrame done, len = %d", USART_RxStruct.CurrentCount);
////            printf("\r\nData(str): %s\r\n", USART_RxStruct.Buffer);

////            USART_RxIdle_Start(30);
////        }

//          //== 按16进制打印  
//           
//        if (USART_RxStruct.CompleteFlag)
//        {
//            printf("\r\nFrame done, len = %d", USART_RxStruct.CurrentCount);
//            printf("\r\nData(hex): ");

//            for (i = 0; i < USART_RxStruct.CurrentCount; i++)
//            {
//                printf("%02X ", USART_RxStruct.Buffer[i]);
//            }

//            printf("\r\n");

//            /* 重新开启下一帧接收 */
//            USART_RxIdle_Start(30);
//        }
//    }
//}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

