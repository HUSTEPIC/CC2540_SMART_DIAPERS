/******************************************************************************

 @file  SimpleBLEPeripheral_Main.c

 @brief This file contains the main and callback functions for the
        Simple BLE Peripheral sample application.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2010-2016, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: ble_sdk_1.4.2.2
 Release Date: 2016-06-09 06:57:10
 *****************************************************************************/

/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/
/* Hal Drivers */
#include "hal_types.h"
#include "hal_key.h"
#include "hal_timer.h"
#include "hal_drivers.h"
#include "hal_led.h"

/* OSAL */
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "OnBoard.h"
#include "simpleble.h"

/**************************************************************************************************
 * FUNCTIONS
 **************************************************************************************************/

/**************************************************************************************************
 * @fn          main
 *
 * @brief       Start of application.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
int main(void)
{
  /* Initialize hardware */
  HAL_BOARD_INIT();

  // Initialize board I/O
  InitBoard( OB_COLD );

  /* Initialze the HAL driver */
  HalDriverInit();

  /* Initialize NV system */
  osal_snv_init();

#if 1//��һ�δ����˵���� amomcu ���ӵ�  
    // �������ж����Ա�������ݣ� �Ա��������Ӧ�������������Ǵӻ�
    // ע�⣬ �����õ��� osal_snv_xxx �� �����Ǵ���flash��ߵģ� ��ҿ���������ش����˵��
    // ��Ҫע����� osal_snv_read �� osal_snv_write �� ��һ�� ���� osalSnvId_t id
    // ���id�� ���Ǳ�̿��õ��Ǵ� 0x80 �� 0xff, ����Ŀǰ�����п��õĿռ��� 2048 �ֽ�
    // �����С������ osal_snv.c �У� �����º궨�壬 ���ٴ����ȥ���ܿ���
    /*
    // NV page configuration
    #define OSAL_NV_PAGE_SIZE       HAL_FLASH_PAGE_SIZE
    #define OSAL_NV_PAGES_USED      HAL_NV_PAGE_CNT
    #define OSAL_NV_PAGE_BEG        HAL_NV_PAGE_BEG
    #define OSAL_NV_PAGE_END       (OSAL_NV_PAGE_BEG + OSAL_NV_PAGES_USED - 1)
    */
    {
        int8 ret8 = osal_snv_read(0x80, sizeof(SYS_CONFIG), &sys_config);
        // ����ö��ڴ�δ��д������ݣ� ֱ�Ӷ����᷵�� NV_OPER_FAILED ,
        // ������������ص���Ϊ��һ����¼������У� �Ӷ����ò����ĳ�������
        if(NV_OPER_FAILED == ret8)
        {
            simpleBLE_SetAllParaDefault(PARA_ALL_FACTORY);
            simpleBLE_WriteAllDataToFlash();
        } 

        // ִ��  ���ڳ�ʼ��
//        simpleBLE_NPI_init();     
    }
#endif//��һ�δ����˵���� amomcu ���ӵ�  

  /* Initialize LL */

  // ����S1 �����ж��������豸�������豸�� 
  // ��������ڼ� S1 �������£�P0_1==0, ����������
  if(false == Check_startup_peripheral_or_central())
  {   
    sys_config.role = BLE_ROLE_PERIPHERAL;
    simpleBLE_WriteAllDataToFlash();
  }
  else
  {
    sys_config.role = BLE_ROLE_CENTRAL;
    simpleBLE_WriteAllDataToFlash();
  }

  // ��������
  simpleBLE_NPI_init();     

  /* Initialize the operating system */
  osal_init_system(sys_config.role);

  /* Enable interrupts */
  HAL_ENABLE_INTERRUPTS();

  // Final board initialization
  InitBoard( OB_READY );

  #if defined ( POWER_SAVING )
//    osal_pwrmgr_device( PWRMGR_BATTERY );  
    osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
  #endif
    
  /* Start OSAL */
  osal_start_system(); // No Return from here

  return 0;
}

/**************************************************************************************************
                                           CALL-BACKS
**************************************************************************************************/


/*************************************************************************************************
**************************************************************************************************/
