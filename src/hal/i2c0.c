/*
 * Copyright (C) 2016 Ascending Technologies GmbH, Germany
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "i2c0.h"
#include "LPC214x.h"
#include "type.h"
#include "irq.h"

#define BUFSIZE       0x20
#define MAX_TIMEOUT   0x00FFFFFF

#define GET_DEVICE_ID     0x01
#define GET_TEMPERATURE   0x02
#define SET_CONFIGURATION 0x03

#define I2C_IDLE            0
#define I2C_STARTED         1
#define I2C_RESTARTED       2
#define I2C_REPEATED_START  3
#define DATA_ACK            4
#define DATA_NACK           5

#define I2CONSET_I2EN       0x00000040  // I2C Control Set Register
#define I2CONSET_AA         0x00000004
#define I2CONSET_SI         0x00000008
#define I2CONSET_STO        0x00000010
#define I2CONSET_STA        0x00000020

#define I2CONCLR_AAC        0x00000004  // I2C Control clear Register
#define I2CONCLR_SIC        0x00000008
#define I2CONCLR_STAC       0x00000020
#define I2CONCLR_I2ENC      0x00000040

#define I2DAT_I2C           0x00000000  // I2C Data Reg
#define I2ADR_I2C           0x00000000  // I2C Slave Address Reg
#define I2SCLH_SCLH         0x00000080  // I2C SCL Duty Cycle High Reg
#define I2SCLL_SCLL         0x00000080  // I2C SCL Duty Cycle Low Reg

static DWORD I2CMasterState = I2C_IDLE;
static DWORD I2CCmd;
static BYTE I2CMasterBuffer[BUFSIZE];
static DWORD I2CReadLength;
static DWORD I2CWriteLength;
static DWORD RdIndex = 0;
static DWORD WrIndex = 0;

/* 
 From device to device, the I2C communication protocol may vary,
 in the example below, the protocol uses repeated start to read data from or
 write to the device:
 For master read: the sequence is: STA,Addr(W),offset,RE-STA,Addr(w),data...STO
 for master write: the sequence is: STA,Addr(W),length,RE-STA,Addr(r),data...STO
 Thus, in state 8, the address is always WRITE. in state 10, the address could
 be READ or WRITE depending on the I2CCmd.
 */

/*****************************************************************************
 ** Function name:		I2C0MasterHandler
 **
 ** Descriptions:		I2C0 interrupt handler, deal with master mode
 **				only.
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
void I2C0MasterHandler(void) __irq
{
  BYTE StatValue;

  /* this handler deals with master read and master write only */
  StatValue = I20STAT;

  IENABLE;
  switch(StatValue)
  {
    case 0x08: /* A Start condition is issued. */
      I20DAT = I2CMasterBuffer[0];
      I20CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
      I2CMasterState = I2C_STARTED;
      break;

    case 0x10: /* A repeated started is issued */
      if(I2CCmd == GET_DEVICE_ID || I2CCmd == GET_TEMPERATURE)
      {
        I20DAT = I2CMasterBuffer[2];
      }
      I20CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
      I2CMasterState = I2C_RESTARTED;
      break;

    case 0x18: /* Regardless, it's a ACK */
      if(I2CMasterState == I2C_STARTED)
      {
        I20DAT = I2CMasterBuffer[1 + WrIndex];
        WrIndex++;
        I2CMasterState = DATA_ACK;
      }
      I20CONCLR = I2CONCLR_SIC;
      break;

    case 0x28: /* Data byte has been transmitted, regardless ACK or NACK */
    case 0x30:
      if(WrIndex != I2CWriteLength)
      {
        I20DAT = I2CMasterBuffer[1 + WrIndex]; /* this should be the last one */
        WrIndex++;
        if(WrIndex != I2CWriteLength)
        {
          I2CMasterState = DATA_ACK;
        }
        else
        {
          I2CMasterState = DATA_NACK;
          if(I2CReadLength != 0)
          {
            I20CONSET = I2CONSET_STA; /* Set Repeated-start flag */
            I2CMasterState = I2C_REPEATED_START;
          }
        }
      }
      else
      {
        if(I2CReadLength != 0)
        {
          I20CONSET = I2CONSET_STA; /* Set Repeated-start flag */
          I2CMasterState = I2C_REPEATED_START;
        }
        else
        {
          I2CMasterState = DATA_NACK;
        }
      }
      I20CONCLR = I2CONCLR_SIC;
      break;

    case 0x40: /* Master Receive, SLA_R has been sent */
      I20CONCLR = I2CONCLR_SIC;
      break;

    case 0x50: /* Data byte has been received, regardless following ACK or NACK */
    case 0x58:
      I2CMasterBuffer[3 + RdIndex] = I20DAT;
      RdIndex++;
      if(RdIndex != I2CReadLength)
      {
        I2CMasterState = DATA_ACK;
      }
      else
      {
        RdIndex = 0;
        I2CMasterState = DATA_NACK;
      }
      I20CONSET = I2CONSET_AA; /* assert ACK after data is received */
      I20CONCLR = I2CONCLR_SIC;
      break;

    case 0x20: /* regardless, it's a NACK */
    case 0x48:
      I20CONCLR = I2CONCLR_SIC;
      I2CMasterState = DATA_NACK;
      break;

    case 0x38: /* Arbitration lost, in this example, we don't
     deal with multiple master situation */
    default:
      I20CONCLR = I2CONCLR_SIC;
      break;
  }

  IDISABLE;
  VICVectAddr = 0; /* Acknowledge Interrupt */
}

/*****************************************************************************
 ** Function name:		I2CStart
 **
 ** Descriptions:		Create I2C start condition, a timeout
 **				value is set if the I2C never gets started,
 **				and timed out. It's a fatal error.
 **
 ** parameters:			None
 ** Returned value:		true or false, return false if timed out
 **
 *****************************************************************************/
unsigned int I2C0Start(void)
{
  unsigned int timeout = 0;
  unsigned int returnValue = FALSE;

  /*--- Issue a start condition ---*/
  I20CONSET = I2CONSET_STA; /* Set Start flag */

  /*--- Wait until START transmitted ---*/
  while(1)
  {
    if(I2CMasterState == I2C_STARTED)
    {
      returnValue = TRUE;
      break;
    }

    if(timeout >= MAX_TIMEOUT)
    {
      returnValue = FALSE;
      break;
    }
    timeout++;
  }

  return returnValue;
}

/*****************************************************************************
 ** Function name:		I2CStop
 **
 ** Descriptions:		Set the I2C stop condition, if the routine
 **				never exit, it's a fatal bus error.
 **
 ** parameters:			None
 ** Returned value:		true or never return
 **
 *****************************************************************************/
unsigned int I2C0Stop(void)
{
  I20CONSET = I2CONSET_STO; /* Set Stop flag */
  I20CONCLR = I2CONCLR_SIC; /* Clear SI flag */

  /*--- Wait for STOP detected ---*/
  while( I20CONSET & I2CONSET_STO);

  return TRUE;
}

/*****************************************************************************
 ** Function name:		I2CInit
 **
 ** Descriptions:		Initialize I2C controller
 **
 ** parameters:			I2c mode is either MASTER or SLAVE
 ** Returned value:		true or false, return false if the I2C
 **				interrupt handler was not installed correctly
 **
 *****************************************************************************/
void I2C0Init(unsigned int I2cMode)
{
  (void)I2cMode;

  IODIR0 |= 0x0C; /* set port 0.2 and port 0.3 to output, high */
  IOSET0 = 0x0C;

  /*--- Clear flags ---*/
  I20CONCLR = I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC | I2CONCLR_I2ENC;

  /*--- Reset registers ---*/
  I20SCLL = I2SCLL_SCLL;
  I20SCLH = I2SCLH_SCLH;
}

/*****************************************************************************
 ** Function name:		I2CEngine
 **
 ** Descriptions:		The routine to complete a I2C transaction
 **				from start to stop. All the intermitten
 **				steps are handled in the interrupt handler.
 **				Before this routine is called, the read
 **				length, write length, I2C master buffer,
 **				and I2C command fields need to be filled.
 **				see i2cmst.c for more details.
 **
 ** parameters:			None
 ** Returned value:		true or false, return false only if the
 **				start condition can never be generated and
 **				timed out.
 **
 *****************************************************************************/
unsigned int I2C0Engine(void)
{
  I2CMasterState = I2C_IDLE;
  RdIndex = 0;
  WrIndex = 0;

  if(I2C0Start() != TRUE)
  {
    I2C0Stop();
    return FALSE;
  }

  while(1)
  {
    if(I2CMasterState == DATA_NACK)
    {
      I2C0Stop();
      break;
    }
  }

  return TRUE;
}
