/******************************************************************************
*
* Module: Port
*
* File Name: Port.c
*
* Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
*
* Author: Youssef Hussien
******************************************************************************/

#include "Port.h"
#include "tm4c123gh6pm_registers.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Dio Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
|| (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
  || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Det.h does not match the expected version"
#endif


#endif
STATIC const Port_ConfigChannel * Channel_PORT = NULL_PTR;
STATIC uint8 Port_Status =PORT_NOT_INITIALIZED;

/************************************************************************************
* Service Name: Port_Init
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): ConfigPtr - Pointer to configuration set
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Initializes the Port Driver module
************************************************************************************/
void Port_Init(const Port_ConfigType* ConfigPtr)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* check if the input configuration pointer is not a NULL_PTR */
  if (NULL_PTR == ConfigPtr)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_Init_SID,PORT_E_PARAM_CONFIG);
  }
  else
#endif
  /*variable to save the statues of port initializing*/
  Port_Status=PORT_INITIALIZED;
  /*pointer to the constructor of configured channels*/
  Channel_PORT=ConfigPtr->Channels;
  
  /*pointer to save the port which will be operation will occur on*/
  volatile uint32* PORT_PTR=NULL_PTR;
  /*looping on the configured channels*/
  for(uint8 i=0;i<PORT_CONFIGURED_CHANNLES ;i++)
  {
    /*saving the port address in the pointer depending on the configured channel*/
    switch(Channel_PORT[i].port_num)
    {
    case PORTA:
      PORT_PTR=(volatile uint32*)GPIO_PORTA_BASE_ADDRESS;
      break;
    case PORTB:
      PORT_PTR=(volatile uint32*)GPIO_PORTB_BASE_ADDRESS;
      break;
    case PORTC:
      PORT_PTR=(volatile uint32*)GPIO_PORTC_BASE_ADDRESS;
      break;
    case PORTD:
      PORT_PTR=(volatile uint32*)GPIO_PORTD_BASE_ADDRESS;
      break;
    case PORTE:
      PORT_PTR=(volatile uint32*)GPIO_PORTE_BASE_ADDRESS;
      break;
    case PORTF:
      PORT_PTR=(volatile uint32*)GPIO_PORTF_BASE_ADDRESS;
      break;
    }

    if( ((Channel_PORT[i].port_num == 3) && (Channel_PORT[i].pin_num == 7)) || ((Channel_PORT[i].port_num == 5) && (Channel_PORT[i].pin_num == 0)) ) /* PD7 or PF0 */
    {
      *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_COMMIT_REG_OFFSET) , Channel_PORT[i].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if( (Channel_PORT[i].port_num == 2) && (Channel_PORT[i].pin_num <= 3) ) /* PC0 to PC3 */
    {
      /* Do Nothing ...  this is the JTAG pins */
    }
    else
    {
      /* MISRA */
    }
    
    
    /*checking if the mode is analog*/
    if(Channel_PORT[i].pin_mode==PortConfig_Mode_ADC)
    {
      /*setting the analog mode for the configured channel if it's mode is analog*/
      SET_BIT((*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_ANALOG_MODE_SEL_REG_OFFSET) ), Channel_PORT[i].pin_num);
      /*clearing the digital mode*/
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_DIGITAL_ENABLE_REG_OFFSET) ,Channel_PORT[i].pin_num );
    }
    else
    {
      /*else the rest of modes is digital */
      CLEAR_BIT((*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_ANALOG_MODE_SEL_REG_OFFSET)) ,Channel_PORT[i].pin_num );
      SET_BIT((*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_DIGITAL_ENABLE_REG_OFFSET)) ,Channel_PORT[i].pin_num );
    }
    if(Channel_PORT[i].pin_mode==PortConfig_Mode_DIO)
    {
      /*if the mode is digital the AFSEL for that configured pin will be disabled, no need to clear the PCTL here*/
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_ALT_FUNC_REG_OFFSET) ,Channel_PORT[i].pin_num );
    }
    else
    {
      /*else the AFSEL will be set and the PCTL will be updated in the next swicth case*/
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_ALT_FUNC_REG_OFFSET) ,Channel_PORT[i].pin_num );
    }
    
    switch(Channel_PORT[i].pin_mode)
    {
    case PortConfig_Mode_ADC:
      /* MISRA */
      break;
      
    case PortConfig_Mode_DIO:
      /*clear the PCTL*/
      *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Channel_PORT[i].pin_num * 4));
      break;
      
    case PortConfig_Mode_UART:
      *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_UART << (Channel_PORT[i].pin_num * 4));
      break;
      
    case PortConfig_Mode_SSI:
      if((Channel_PORT[i].port_num==PORTD)&&(Channel_PORT[i].pin_num<=PIN3))
      {
        *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_UART << (Channel_PORT[i].pin_num * 4));
      }
      else
      {
        *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_SSI<< (Channel_PORT[i].pin_num * 4));
      }
      break;
      
    case PortConfig_Mode_I2C:
      *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_I2C<< (Channel_PORT[i].pin_num * 4));
      break;
      
    case PortConfig_Mode_M0PWM:
      if(((Channel_PORT[i].port_num==PORTD)&&(Channel_PORT[i].pin_num<=PIN2))||((Channel_PORT[i].port_num==PORTD)&&(Channel_PORT[i].pin_num<=PIN6))||((Channel_PORT[i].port_num==PORTF)&&(Channel_PORT[i].pin_num<=PIN2)))
      {
        *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_M0FAULT << (Channel_PORT[i].pin_num * 4));
      }
      else
      {
        *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_M0PWM << (Channel_PORT[i].pin_num * 4));
      }
      break;
    case PortConfig_Mode_M1PWM:
      *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_M1PWM << (Channel_PORT[i].pin_num * 4));
      break;
    case PortConfig_Mode_IDX:
      *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_IDX << (Channel_PORT[i].pin_num * 4));
      break;      
    case PortConfig_Mode_TIMER:
      *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_TIMER<< (Channel_PORT[i].pin_num * 4));
      break;
    case PortConfig_Mode_CAN:
      
      if(((Channel_PORT[i].port_num==PORTA)&&((Channel_PORT[i].port_num==PIN0)||(Channel_PORT[i].port_num==PIN1)))||
         ((Channel_PORT[i].port_num==PORTB)&&((Channel_PORT[i].port_num==PIN4)||(Channel_PORT[i].port_num==PIN5)))||
           ((Channel_PORT[i].port_num==PORTE)&&((Channel_PORT[i].port_num==PIN4)||(Channel_PORT[i].port_num==PIN5))))
      {
        *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_CAN<< (Channel_PORT[i].pin_num * 4));
      }
      else if(((Channel_PORT[i].port_num==PORTC)&&((Channel_PORT[i].port_num==PIN6)||(Channel_PORT[i].port_num==PIN7)))||
              ((Channel_PORT[i].port_num==PORTD)&&((Channel_PORT[i].port_num==PIN2)||(Channel_PORT[i].port_num==PIN3)))||
                ((Channel_PORT[i].port_num==PORTF)&&((Channel_PORT[i].port_num==PIN4))))
      {
        *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_USB<< (Channel_PORT[i].pin_num * 4));
      }
      else if (((Channel_PORT[i].port_num==PORTD)&&(Channel_PORT[i].port_num==PIN7))||\
        ((Channel_PORT[i].port_num==PORTF)&&(Channel_PORT[i].port_num==PIN0)))
      {
        *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_NMI<< (Channel_PORT[i].pin_num * 4));
      }
      else
      {
        /* MISRA */
      }
      break;
    }
    if(Channel_PORT[i].direction==OUTPUT)
    {
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_DIR_REG_OFFSET),Channel_PORT[i].pin_num);
      if(Channel_PORT[i].initial_value==STD_HIGH)
      {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_DATA_REG_OFFSET),Channel_PORT[i].pin_num);
      }
      else
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_DATA_REG_OFFSET),Channel_PORT[i].pin_num);
      }
      
    }
    else if (Channel_PORT[i].direction==INPUT)
    {
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_DIR_REG_OFFSET),Channel_PORT[i].pin_num);
      if(Channel_PORT[i].resistor==PULL_UP)
      {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_PULL_UP_REG_OFFSET ),Channel_PORT[i].pin_num);
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_PULL_DOWN_REG_OFFSET ),Channel_PORT[i].pin_num);
      }
      else if(Channel_PORT[i].resistor==PULL_DOWN)
      {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_PULL_DOWN_REG_OFFSET ),Channel_PORT[i].pin_num);
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_PULL_UP_REG_OFFSET ),Channel_PORT[i].pin_num);
      }
      else
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_PULL_UP_REG_OFFSET ),Channel_PORT[i].pin_num);
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_PULL_DOWN_REG_OFFSET ),Channel_PORT[i].pin_num);
      }
    }
    else
    {
      /*do nothing*/
    }
    
  }
  
}




/************************************************************************************
* Service Name: Port_SetPinDirection
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin          - Port Pin ID number
Direction    - Port Pin Direction 
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction
************************************************************************************/
#if(PORT_SET_PIN_DIRECTION==STD_ON)
void Port_SetPinDirection(Port_PinType Pin,Port_PinDirectionType Direction)
{
  boolean error=FALSE;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* Check If Pin Is Correct */
  if (Pin >= PORT_CONFIGURED_CHANNLES)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID,
                    PORT_E_PARAM_PIN);
    error = TRUE ;
  }
  else
  {
    /* MISRA */
  }
  if (Port_Status == PORT_NOT_INITIALIZED)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID,
                    PORT_E_UNINIT);
    error = TRUE ;
  }
  else
  {
    /* MISRA */
  }
  if (Channel_PORT[Pin].direction_changeable == DIRECTION_CHANGE_OFF)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID,
                    PORT_E_DIRECTION_UNCHANGEABLE);
    error = TRUE ;
  }
  else
  {
    /* MISRA */
  }
#endif
  
  if(error==FALSE)
  {
    volatile uint32* PORT_PTR=NULL_PTR;
    switch(Channel_PORT[Pin].port_num)
    {
    case PORTA:
      PORT_PTR=(volatile uint32*)GPIO_PORTA_BASE_ADDRESS;
      break;
    case PORTB:
      PORT_PTR=(volatile uint32*)GPIO_PORTB_BASE_ADDRESS;
      break;
    case PORTC:
      PORT_PTR=(volatile uint32*)GPIO_PORTC_BASE_ADDRESS;
      break;
    case PORTD:
      PORT_PTR=(volatile uint32*)GPIO_PORTD_BASE_ADDRESS;
      break;
    case PORTE:
      PORT_PTR=(volatile uint32*)GPIO_PORTE_BASE_ADDRESS;
      break;
    case PORTF:
      PORT_PTR=(volatile uint32*)GPIO_PORTF_BASE_ADDRESS;
      break;
    }
    if( ((Channel_PORT[Pin].port_num == 3) && (Channel_PORT[Pin].pin_num == 7)) || ((Channel_PORT[Pin].port_num == 5) && (Channel_PORT[Pin].pin_num == 0)) ) /* PD7 or PF0 */
    {
      *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_COMMIT_REG_OFFSET) , Channel_PORT[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if( (Channel_PORT[Pin].port_num == 2) && (Channel_PORT[Pin].pin_num <= 3) ) /* PC0 to PC3 */
    {
      /* Do Nothing ...  this is the JTAG pins */
    }
    else
    {
      /* Do Nothing ... No need to unlock the commit register for this pin */
    }
    if(Direction==OUTPUT)
    {
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_DIR_REG_OFFSET) , Channel_PORT[Pin].pin_num);
    }
    else
    {
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_DIR_REG_OFFSET) , Channel_PORT[Pin].pin_num);
    }
    
  }
  
  
  else
  {
    /* MISRA */
  }
  
  
  
  
}
#endif







/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): None     
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction
************************************************************************************/
void Port_RefreshPortDirection(void)
{
  boolean error=FALSE;
#if(PORT_DEV_ERROR_DETECT == STD_ON)
  {
    if (Port_Status == PORT_NOT_INITIALIZED)
    {
      Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID,
                      PORT_E_UNINIT);
      error = TRUE ;
    }
    else
    {
      /* Do Nothing */
    }
    
  }
#endif
  if(error==FALSE)
  {
    volatile uint32* PORT_PTR=NULL_PTR;
    /*looping on the configured channels*/
    for(uint8 i=0;i<PORT_CONFIGURED_CHANNLES ;i++)
    {
      /*saving the port address in the pointer depending on the configured channel*/
      switch(Channel_PORT[i].port_num)
      {
      case PORTA:
        PORT_PTR=(volatile uint32*)GPIO_PORTA_BASE_ADDRESS;
        break;
      case PORTB:
        PORT_PTR=(volatile uint32*)GPIO_PORTB_BASE_ADDRESS;
        break;
      case PORTC:
        PORT_PTR=(volatile uint32*)GPIO_PORTC_BASE_ADDRESS;
        break;
      case PORTD:
        PORT_PTR=(volatile uint32*)GPIO_PORTD_BASE_ADDRESS;
        break;
      case PORTE:
        PORT_PTR=(volatile uint32*)GPIO_PORTE_BASE_ADDRESS;
        break;
      case PORTF:
        PORT_PTR=(volatile uint32*)GPIO_PORTF_BASE_ADDRESS;
        break;
      }
      if( ((Channel_PORT[i].port_num == 3) && (Channel_PORT[i].pin_num == 7)) || ((Channel_PORT[i].port_num == 5) && (Channel_PORT[i].pin_num == 0)) ) /* PD7 or PF0 */
      {
        *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_COMMIT_REG_OFFSET) , Channel_PORT[i].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
      }
      else if( (Channel_PORT[i].port_num == 2) && (Channel_PORT[i].pin_num <= 3) ) /* PC0 to PC3 */
      {
        /* Do Nothing ...  this is the JTAG pins */
      }
      else
      {
        /* Do Nothing ... No need to unlock the commit register for this pin */
      }
      if(Channel_PORT[i].direction==OUTPUT)
      {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_DIR_REG_OFFSET),Channel_PORT[i].pin_num);
      }
      if(Channel_PORT[i].direction==INPUT)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_DIR_REG_OFFSET),Channel_PORT[i].pin_num);
        
      }
      
      
    }
  }
}










/************************************************************************************
* Service Name: Port_GetVersionInfo
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): None     
* Parameters (inout): None
* Parameters (out): versioninfo     -Pointer to where to store the version information of this module. 
* Return value: None
* Description: Returns the version information of this module. 
************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* Check if input pointer is not Null pointer */
  if(NULL_PTR == versioninfo)
  {
    /* Report to DET  */
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                    Port_SetPinDirection_SID, PORT_E_PARAM_POINTER);
  }
  else
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
  {
    /* Copy the vendor Id */
    versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
    /* Copy the module Id */
    versioninfo->moduleID = (uint16)PORT_MODULE_ID;
    /* Copy Software Major Version */
    versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
    /* Copy Software Minor Version */
    versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
    /* Copy Software Patch Version */
    versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
  }
}

#endif

/************************************************************************************
* Service Name: Port_SetPinMode
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin          - Port Pin ID number
Mode         - New Port Pin mode to be set on port pin. 
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode.
************************************************************************************/
void Port_SetPinMode(Port_PinType Pin,Port_PinModeType Mode)
{
  boolean error=FALSE;
  
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* Check If Pin Is Correct */
  if (Pin >= PORT_CONFIGURED_CHANNLES)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID,
                    PORT_E_PARAM_PIN);
    error = TRUE ;
  }
  else
  {
    /* DO Nothing */
  }
  if (Port_Status == PORT_NOT_INITIALIZED)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID,
                    PORT_E_UNINIT);
    error = TRUE ;
  }
  else
  {
    /* Do Nothing */
  }
  if (Channel_PORT[Pin].mode_changeable == MODE_CHANGE_OFF)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID,
                    PORT_E_DIRECTION_UNCHANGEABLE);
    error = TRUE ;
  }
  else
  {
    /* Do Nothing */
  }
  if(Mode>=PORT_CONFIGURED_MODES)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID,
                    PORT_E_DIRECTION_UNCHANGEABLE);
    error = TRUE ;
  }
  else
  {
    /* Do Nothing */
  }
#endif
  if(error==FALSE)
  {
    volatile uint32* PORT_PTR=NULL_PTR;
    switch(Channel_PORT[Pin].port_num)
    {
    case PORTA:
      PORT_PTR=(volatile uint32*)GPIO_PORTA_BASE_ADDRESS;
      break;
    case PORTB:
      PORT_PTR=(volatile uint32*)GPIO_PORTB_BASE_ADDRESS;
      break;
    case PORTC:
      PORT_PTR=(volatile uint32*)GPIO_PORTC_BASE_ADDRESS;
      break;
    case PORTD:
      PORT_PTR=(volatile uint32*)GPIO_PORTD_BASE_ADDRESS;
      break;
    case PORTE:
      PORT_PTR=(volatile uint32*)GPIO_PORTE_BASE_ADDRESS;
      break;
    case PORTF:
      PORT_PTR=(volatile uint32*)GPIO_PORTF_BASE_ADDRESS;
      break;
    }
    if( ((Channel_PORT[Pin].port_num == 3) && (Channel_PORT[Pin].pin_num == 7)) || ((Channel_PORT[Pin].port_num == 5) && (Channel_PORT[Pin].pin_num == 0)) ) /* PD7 or PF0 */
    {
      *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_COMMIT_REG_OFFSET) , Channel_PORT[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if( (Channel_PORT[Pin].port_num == 2) && (Channel_PORT[Pin].pin_num <= 3) ) /* PC0 to PC3 */
    {
      /* Do Nothing ...  this is the JTAG pins */
    }
    else
    {
      /* Do Nothing ... No need to unlock the commit register for this pin */
    }
    if(Channel_PORT[Pin].pin_mode==PortConfig_Mode_ADC)
    {
      /*setting the analog mode for the configured channel if it's mode is analog*/
      SET_BIT((*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_ANALOG_MODE_SEL_REG_OFFSET) ), Channel_PORT[Pin].pin_num);
      /*clearing the digital mode*/
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_DIGITAL_ENABLE_REG_OFFSET) ,Channel_PORT[Pin].pin_num );
    }
    else
    {
      /*else the rest of modes is digital */
      CLEAR_BIT((*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_ANALOG_MODE_SEL_REG_OFFSET)) ,Channel_PORT[Pin].pin_num );
      SET_BIT((*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_DIGITAL_ENABLE_REG_OFFSET)) ,Channel_PORT[Pin].pin_num );
    }
    if(Channel_PORT[Pin].pin_mode==PortConfig_Mode_DIO)
    {
      /*if the mode is digital the AFSEL for that configured pin will be disabled, no need to clear the PCTL here*/
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_ALT_FUNC_REG_OFFSET) ,Channel_PORT[Pin].pin_num );
    }
    else
    {
      /*else the AFSEL will be set and the PCTL will be updated in the next swicth case*/
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_ALT_FUNC_REG_OFFSET) ,Channel_PORT[Pin].pin_num );
    }
    
    switch(Channel_PORT[Pin].pin_mode)
    {
    case PortConfig_Mode_ADC:
      /*do nothing*/
      break;
      
    case PortConfig_Mode_DIO:
      /*clear the PCTL*/
      *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Channel_PORT[Pin].pin_num * 4));
      break;
      
    case PortConfig_Mode_UART:
      *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_UART << (Channel_PORT[Pin].pin_num * 4));
      break;
      
    case PortConfig_Mode_SSI:
      if((Channel_PORT[Pin].port_num==PORTD)&&(Channel_PORT[Pin].pin_num<=PIN3))
      {
        *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_UART << (Channel_PORT[Pin].pin_num * 4));
      }
      else
      {
        *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_SSI<< (Channel_PORT[Pin].pin_num * 4));
      }
      break;
      
    case PortConfig_Mode_I2C:
      *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_I2C<< (Channel_PORT[Pin].pin_num * 4));
      break;
      
    case PortConfig_Mode_M0PWM:
      if(((Channel_PORT[Pin].port_num==PORTD)&&(Channel_PORT[Pin].pin_num<=PIN2))||((Channel_PORT[Pin].port_num==PORTD)&&(Channel_PORT[Pin].pin_num<=PIN6))||((Channel_PORT[Pin].port_num==PORTF)&&(Channel_PORT[Pin].pin_num<=PIN2)))
      {
        *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_M0FAULT << (Channel_PORT[Pin].pin_num * 4));
      }
      else
      {
        *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_M0PWM << (Channel_PORT[Pin].pin_num * 4));
      }
      break;
    case PortConfig_Mode_M1PWM:
      *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_M1PWM << (Channel_PORT[Pin].pin_num * 4));
      break;
    case PortConfig_Mode_IDX:
      *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_IDX << (Channel_PORT[Pin].pin_num * 4));
      break;      
    case PortConfig_Mode_TIMER:
      *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_TIMER<< (Channel_PORT[Pin].pin_num * 4));
      break;
    case PortConfig_Mode_CAN:
      
      if(((Channel_PORT[Pin].port_num==PORTA)&&((Channel_PORT[Pin].port_num==PIN0)||(Channel_PORT[Pin].port_num==PIN1)))||
         ((Channel_PORT[Pin].port_num==PORTB)&&((Channel_PORT[Pin].port_num==PIN4)||(Channel_PORT[Pin].port_num==PIN5)))||
           ((Channel_PORT[Pin].port_num==PORTE)&&((Channel_PORT[Pin].port_num==PIN4)||(Channel_PORT[Pin].port_num==PIN5))))
      {
        *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_CAN<< (Channel_PORT[Pin].pin_num * 4));
      }
      else if(((Channel_PORT[Pin].port_num==PORTC)&&((Channel_PORT[Pin].port_num==PIN6)||(Channel_PORT[Pin].port_num==PIN7)))||
              ((Channel_PORT[Pin].port_num==PORTD)&&((Channel_PORT[Pin].port_num==PIN2)||(Channel_PORT[Pin].port_num==PIN3)))||
                ((Channel_PORT[Pin].port_num==PORTF)&&((Channel_PORT[Pin].port_num==PIN4))))
      {
        *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_USB<< (Channel_PORT[Pin].pin_num * 4));
      }
      else if (((Channel_PORT[Pin].port_num==PORTD)&&(Channel_PORT[Pin].port_num==PIN7))||\
        ((Channel_PORT[Pin].port_num==PORTF)&&(Channel_PORT[Pin].port_num==PIN0)))
      {
        *(volatile uint32 *)((volatile uint8 *)PORT_PTR + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_NMI<< (Channel_PORT[Pin].pin_num * 4));
      }
      else
      {
        /* MISRA */
      }
      break;
    }
    
    
    
  }
}

