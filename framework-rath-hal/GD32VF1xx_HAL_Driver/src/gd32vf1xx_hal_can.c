
#include "gd32vf1xx_hal_can.h"


uint8_t HAL_CAN_init(CAN_TypeDef *CANx, CAN_InitTypeDef *CAN_init) {
  uint32_t timeout = CAN_TIMEOUT;
  
  /* disable sleep mode */
  CLEAR_BITS(CANx->CTL, CAN_CTL_SLPWMOD_MSK);
  
  /* wait sleep leave */
  while(READ_BITS(CANx->STAT, CAN_STAT_SLPWS_MSK)) {
    timeout -= 1;
  }

  /* enable initialize mode */
  SET_BITS(CANx->CTL, CAN_CTL_IWMOD_MSK);

  /* wait ACK */
  while (!READ_BITS(CANx->STAT, CAN_STAT_IWS_MSK)) {
    timeout -= 1;
  }
  
  /* time trigger communication mode */
  if (CAN_init->time_triggered) {
    SET_BITS(CANx->CTL, CAN_CTL_TTC_MSK);
  }
  else {
    CLEAR_BITS(CANx->CTL, CAN_CTL_TTC_MSK);
  }
  /* automatic bus-off managment */
  if (CAN_init->auto_bus_off_recovery) {
    SET_BITS(CANx->CTL, CAN_CTL_ABOR_MSK);
  }
  else {
    CLEAR_BITS(CANx->CTL, CAN_CTL_ABOR_MSK);
  }
  /* automatic wakeup mode */
  if (CAN_init->auto_wake_up) {
    SET_BITS(CANx->CTL, CAN_CTL_AWU_MSK);
  }
  else {
    CLEAR_BITS(CANx->CTL, CAN_CTL_AWU_MSK);
  }
  /* automatic retransmission mode disable*/
  if (CAN_init->auto_retrans) {
    SET_BITS(CANx->CTL, CAN_CTL_ARD_MSK);
  }
  else {
    CLEAR_BITS(CANx->CTL, CAN_CTL_ARD_MSK);
  }
  /* receive fifo overwrite mode */        
  if (CAN_init->rec_fifo_overwrite) {
    SET_BITS(CANx->CTL, CAN_CTL_RFOD_MSK);
  }
  else {
    CLEAR_BITS(CANx->CTL, CAN_CTL_RFOD_MSK);
  } 
  /* transmit fifo order */
  if (CAN_init->trans_fifo_order) {
    SET_BITS(CANx->CTL, CAN_CTL_TFO_MSK);
  }
  else {
    CLEAR_BITS(CANx->CTL, CAN_CTL_TFO_MSK);
  }

  /* set the bit timing register */
  CANx->BT = (READ_BITS(CAN_init->working_mode, CAN_BT_SCMOD_MSK | CAN_BT_LCMOD_MSK)
              | READ_BITS(CAN_init->resync_jump_width, CAN_BT_SJW_MSK)
              | READ_BITS(CAN_init->time_segment_1, CAN_BT_BS1_MSK)
              | READ_BITS(CAN_init->time_segment_2, CAN_BT_BS2_MSK)
              | (READ_BITS(CAN_init->prescaler << CAN_BT_BAUDPSC_POS, CAN_BT_BAUDPSC_MSK) - 1UL));


  CANx->ERR = 0x0;
  
  CANx->TxMailBox[0].TMI = 0x0;
  CANx->TxMailBox[1].TMI = 0x0;
  CANx->TxMailBox[2].TMI = 0x0;
  CANx->TxMailBox[0].TMP = 0x0;
  CANx->TxMailBox[1].TMP = 0x0;
  CANx->TxMailBox[2].TMP = 0x0;
  CANx->TxMailBox[0].TMDATA0 = 0x0;
  CANx->TxMailBox[1].TMDATA0 = 0x0;
  CANx->TxMailBox[2].TMDATA0 = 0x0;
  CANx->TxMailBox[0].TMDATA0 = 0x0;
  CANx->TxMailBox[1].TMDATA0 = 0x0;
  CANx->TxMailBox[2].TMDATA0 = 0x0;

  /* disable initialize mode */
  CANx->CTL &= ~CAN_CTL_IWMOD_MSK;

  timeout = CAN_TIMEOUT;

  /* wait the ACK */
  while(READ_BITS(CANx->STAT, CAN_STAT_IWS_MSK)){
    timeout -= 1;
  }
  
  return HAL_OK;
}


void HAL_CAN_initFilter(CAN_FilterInitTypeDef* CAN_filter_init) {
    uint32_t val = 0U;
    
    /* filter lock disable */
    SET_BITS(CAN0->FCTL, CAN_FCTL_FLD_MSK);

    val = (uint32_t)1UL << (CAN_filter_init->filter_number & 0x1FU);
    /* disable filter */
    CLEAR_BITS(CAN0->FW, val);
    
    /* filter 16 bits */
    if(CAN_filter_init->filter_bits == CAN_FILTERBITS_16BIT){
        /* set filter 16 bits */
        CAN0->FSCFG &= ~(uint32_t)val;
        /* first 16 bits list and first 16 bits mask or first 16 bits list and second 16 bits list */
        
        CAN0->FilterRegister[CAN_filter_init->filter_number].FDATA0 = (0xFFFF0000UL & ((CAN_filter_init->filter_mask_low) & CAN_FILTER_MASK_16BITS) << 16) | \
                (0x0000FFFFUL & ((CAN_filter_init->filter_list_low) & CAN_FILTER_MASK_16BITS));
        /* second 16 bits list and second 16 bits mask or third 16 bits list and fourth 16 bits list */
        CAN0->FilterRegister[CAN_filter_init->filter_number].FDATA1 = (0xFFFF0000UL & ((CAN_filter_init->filter_mask_high) & CAN_FILTER_MASK_16BITS) << 16) | \
                (0x0000FFFFUL & ((CAN_filter_init->filter_mask_high) & CAN_FILTER_MASK_16BITS));
    }
    /* filter 32 bits */
    if(CAN_FILTERBITS_32BIT == CAN_filter_init->filter_bits){
        /* set filter 32 bits */
        CAN0->FSCFG |= (uint32_t)val;
        
        CAN0->FilterRegister[CAN_filter_init->filter_number].FDATA0 = (0xFFFF0000UL & ((CAN_filter_init->filter_list_high) & CAN_FILTER_MASK_16BITS) << 16) | \
                (0x0000FFFFUL & ((CAN_filter_init->filter_list_low) & CAN_FILTER_MASK_16BITS));
        
        CAN0->FilterRegister[CAN_filter_init->filter_number].FDATA1 = (0xFFFF0000UL & ((CAN_filter_init->filter_mask_high) & CAN_FILTER_MASK_16BITS) << 16) | \
                (0x0000FFFFUL & ((CAN_filter_init->filter_mask_low) & CAN_FILTER_MASK_16BITS));
    }
    
    /* filter mode */
    if(CAN_filter_init->filter_mode == CAN_FILTER_MODE_MASK){
        /* mask mode */
        CAN0->FMCFG &= ~(uint32_t)val;
    }else{
        /* list mode */
        CAN0->FMCFG |= (uint32_t)val;
    }
    
    /* filter FIFO */
    if(CAN_FIFO0 == (CAN_filter_init->filter_fifo_number)){
        /* FIFO0 */
        CAN0->FAFIFO &= ~(uint32_t)val;
    }else{
        /* FIFO1 */
        CAN0->FAFIFO |= (uint32_t)val;
    }
    
    /* filter working */
    if(ENABLE == CAN_filter_init->filter_enable){
        
        CAN0->FW |= (uint32_t)val;
    }
    
    /* filter lock enable */
    CAN0->FCTL &= ~CAN_FCTL_FLD_MSK;
}


uint8_t HAL_CAN_transmit(CAN_TypeDef *CANx, CAN_MessageTypeDef* message, uint64_t timeout) {
    uint8_t mailbox_number = CAN_MAILBOX0;

    /* select one empty mailbox */
    if (READ_BITS(CANx->TSTAT, CAN_TSTAT_TME0_MSK)) {
        mailbox_number = CAN_MAILBOX0;
    }
    else if (READ_BITS(CANx->TSTAT, CAN_TSTAT_TME1_MSK)) {
        mailbox_number = CAN_MAILBOX1;
    }
    else if (READ_BITS(CANx->TSTAT, CAN_TSTAT_TME2_MSK)) {
        mailbox_number = CAN_MAILBOX2;
    }
    else {
        mailbox_number = CAN_NOMAILBOX;
        return CAN_NOMAILBOX;
    }
    
    CLEAR_BITS(CANx->TxMailBox[mailbox_number].TMI, CAN_TMI0_TEN_MSK);
    if (message->frame_format == CAN_FRAME_FORMAT_STANDARD) {
        /* set transmit mailbox standard identifier */
        CANx->TxMailBox[mailbox_number].TMI |= (((message->sfid << CAN_TMI0_SFID_EFID_POS) & CAN_TMI0_SFID_EFID_MSK) | \
                                                message->frame_type);
    }
    else {
        /* set transmit mailbox extended identifier */
        CANx->TxMailBox[mailbox_number].TMI |= (((message->efid << CAN_TMI0_EFID_POS) & (CAN_TMI0_SFID_EFID_MSK | CAN_TMI0_EFID_MSK)) | \
                                                message->frame_format | \
                                                message->frame_type);
    }
    /* set the data length */
    CANx->TxMailBox[mailbox_number].TMP &= ~CAN_TMP0_DLENC_MSK;
    CANx->TxMailBox[mailbox_number].TMP |= message->data_length;
    /* set the data */
    CANx->TxMailBox[mailbox_number].TMDATA0 = ((message->data[3] << CAN_TMDATA00_DB3_POS) & CAN_TMDATA00_DB3_MSK) | \
                                              ((message->data[2] << CAN_TMDATA00_DB2_POS) & CAN_TMDATA00_DB2_MSK) | \
                                              ((message->data[1] << CAN_TMDATA00_DB1_POS) & CAN_TMDATA00_DB1_MSK) | \
                                              ((message->data[0] << CAN_TMDATA00_DB0_POS) & CAN_TMDATA00_DB0_MSK);
    CANx->TxMailBox[mailbox_number].TMDATA1 = ((message->data[7] << CAN_TMDATA10_DB7_POS) & CAN_TMDATA10_DB7_MSK) | \
                                              ((message->data[6] << CAN_TMDATA10_DB6_POS) & CAN_TMDATA10_DB6_MSK) | \
                                              ((message->data[5] << CAN_TMDATA10_DB5_POS) & CAN_TMDATA10_DB5_MSK) | \
                                              ((message->data[4] << CAN_TMDATA10_DB4_POS) & CAN_TMDATA10_DB4_MSK);
                                              
    /* enable transmission */
    CANx->TxMailBox[mailbox_number].TMI |= CAN_TMI0_TEN_MSK;

    return mailbox_number;
}


uint8_t HAL_CAN_getReceiveFIFOLength(CAN_TypeDef *CANx, uint8_t fifo_number) {
  if (fifo_number == CAN_FIFO0) {
    /* FIFO0 */
    return READ_BITS(CANx->RFIFO0, CAN_RFIFO0_RFL0_MSK);
  }
  else if(fifo_number == CAN_FIFO1) {
    /* FIFO1 */
    return READ_BITS(CANx->RFIFO1, CAN_RFIFO1_RFL1_MSK);
  }
  /* illegal parameters */
  return HAL_ERROR;
}


uint8_t HAL_CAN_receive(CAN_TypeDef *CANx, uint8_t fifo_number, CAN_MessageTypeDef* message, uint64_t timeout) {
  /* get the frame format */
  message->frame_format = READ_BITS(CANx->FIFOMailBox[fifo_number].RFIFOMI, CAN_RFIFOMI0_FF_MSK);
  if (message->frame_format == CAN_FRAME_FORMAT_STANDARD) {
      /* get standard identifier */
      message->sfid = (uint32_t)(READ_BITS(CANx->FIFOMailBox[fifo_number].RFIFOMI, CAN_RFIFOMI0_SFID_EFID_MSK));
  }
  else {
      /* get extended identifier */
      message->efid = (uint32_t)(READ_BITS(CANx->FIFOMailBox[fifo_number].RFIFOMI, CAN_RFIFOMI0_SFID_EFID_MSK | CAN_RFIFOMI0_EFID_MSK));
  }
  
  /* get frame type */
  message->frame_type = (uint8_t)(CAN_RFIFOMI0_FT_MSK & CANx->FIFOMailBox[fifo_number].RFIFOMI);        
  /* filtering index */
  message->filter_index = (uint8_t)(READ_BITS(CANx->FIFOMailBox[fifo_number].RFIFOMP, CAN_RFIFOMI1_FT_MSK));
  /* get recevie data length */
  message->data_length = (uint8_t)(READ_BITS(CANx->FIFOMailBox[fifo_number].RFIFOMP, CAN_RFIFOMP0_DLENC_MSK));
  
  /* receive data */
  message -> data[0] = (uint8_t)(READ_BITS(CANx->FIFOMailBox[fifo_number].RFIFOMDATA0, CAN_TMDATA00_DB0_MSK) >> CAN_TMDATA00_DB0_POS);
  message -> data[1] = (uint8_t)(READ_BITS(CANx->FIFOMailBox[fifo_number].RFIFOMDATA0, CAN_TMDATA00_DB1_MSK) >> CAN_TMDATA00_DB1_POS);
  message -> data[2] = (uint8_t)(READ_BITS(CANx->FIFOMailBox[fifo_number].RFIFOMDATA0, CAN_TMDATA00_DB2_MSK) >> CAN_TMDATA00_DB2_POS);
  message -> data[3] = (uint8_t)(READ_BITS(CANx->FIFOMailBox[fifo_number].RFIFOMDATA0, CAN_TMDATA00_DB3_MSK) >> CAN_TMDATA00_DB3_POS);
  message -> data[4] = (uint8_t)(READ_BITS(CANx->FIFOMailBox[fifo_number].RFIFOMDATA1, CAN_TMDATA10_DB4_MSK) >> CAN_TMDATA10_DB4_POS);
  message -> data[5] = (uint8_t)(READ_BITS(CANx->FIFOMailBox[fifo_number].RFIFOMDATA1, CAN_TMDATA10_DB5_MSK) >> CAN_TMDATA10_DB5_POS);
  message -> data[6] = (uint8_t)(READ_BITS(CANx->FIFOMailBox[fifo_number].RFIFOMDATA1, CAN_TMDATA10_DB6_MSK) >> CAN_TMDATA10_DB6_POS);
  message -> data[7] = (uint8_t)(READ_BITS(CANx->FIFOMailBox[fifo_number].RFIFOMDATA1, CAN_TMDATA10_DB7_MSK) >> CAN_TMDATA10_DB7_POS);
  
  /* release FIFO */
  if(CAN_FIFO0 == fifo_number){
      CANx->RFIFO0 |= CAN_RFIFO0_RFD0_MSK;
  }else{
      CANx->RFIFO1 |= CAN_RFIFO1_RFD1_MSK;
  }

  return HAL_OK;
}
