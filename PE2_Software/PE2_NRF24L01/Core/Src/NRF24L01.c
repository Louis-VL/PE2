/*
 * NRF24L01.c
 *
 *  Created on: May 18, 2023
 *      Author: louis
 */



#include "stm32f3xx_hal.h"
#include "NRF24L01.h"

extern SPI_HandleTypeDef hspi2;
#define NRF24_SPI &hspi2

#define NRF24_CE_PORT	GPIOA
#define NRF24_CE_PIN	GPIO_PIN_8

#define NRF24_CS_PORT	GPIOB
#define NRF24_CS_PIN	GPIO_PIN_12


void CS_Select(void){
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_RESET);
}

void CS_UnSelect(void){
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_SET);
}

void CE_Enable(void){
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

void CE_Disable(void){
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}

//write a single byte to a particular register
void nrf24_WriteReg(uint8_t Reg, uint8_t Data){
	uint8_t buf[2];
	buf[0] = Reg|1<<5;
	buf[1] = Data;

	//Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, buf, 2, 1000);

	//Pull the CS HIGH to release the device
	CS_UnSelect();
}


//write multiple bytes to a particular register
void nrf24_WriteRegMulti(uint8_t Reg, uint8_t *data, int size){
	uint8_t buf[2];
	buf[0] = Reg | 1<<5;
//	buf[1] = Data;

	//Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, buf, 1, 100);
	HAL_SPI_Transmit(NRF24_SPI, data, size, 1000);

	//Pull the CS HIGH to release the device
	CS_UnSelect();
}


//Read multiple bytes from the register
uint8_t nrf24_ReadReg (uint8_t Reg){
	uint8_t data =0;
	//Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, &data, 1, 100);

	//Pull the CS HIGH to release the device
	CS_UnSelect();

	return data;
}


//send command to the NRF24
void nrfsendCmd(uint8_t cmd){
	//Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

	//Pull the CS HIGH to release the device
	CS_UnSelect();
}


void NRF24_Init(void){
	//disable the chip before configuration
	CE_Disable();


	nrf24_WriteReg(CONFIG, 0); 		//Will be configure later

	nrf24_WriteReg(EN_AA, 0); 		//No auto ACK

	nrf24_WriteReg(EN_RXADDR, 0);	//Not enabling any data pipe right now

	nrf24_WriteReg(SETUP_AW, 0x03);	//5 Bytes for the TX_RX address

	nrf24_WriteReg(SETUP_RETR, 0);	//No retransmission

	nrf24_WriteReg(RF_CH, 0);		//Will be setup during TX or RX

	nrf24_WriteReg(RF_SETUP, 0x0E);	//Power = 0db, data rate = 2Mbps zie datasheet for other options

	//enable the chip after configuring the device
	CE_Enable();
}


//Set up the TX mode
void NRF24_TxMode (uint8_t *Address, uint8_t channel){
	//disable the chip before configuring the device
	CE_Disable();
	CS_UnSelect();

	nrf24_WriteReg(RF_CH, channel); //select the channel, zie datasheet for channel

	nrf24_WriteRegMulti(TX_ADDR, Address, 5);	//Write the TX adddress

	//power up the device
	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1<<1);
	nrf24_WriteReg(CONFIG, config);

	//enable the chip after configuring the device
	CE_Enable();
}


//transmit the data
uint8_t NRF24_Transmit(uint8_t *data){

	uint8_t cmdtosend = 0;

	//select the device
	CS_Select();

	//payload command
	cmdtosend = W_TX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

	//send the payload
	HAL_SPI_Transmit(NRF24_SPI, data,32,1000);

	//Unselect the device
	CS_UnSelect();

	HAL_Delay(1);

	uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);

	if ((fifostatus&(1<<4)) && ((fifostatus&(1<<3))))
		{
			cmdtosend = FLUSH_TX;
			nrfsendCmd(cmdtosend);

			return 1;
		}

		return 0;
}



void NRF24_RxMode(uint8_t *Address, uint8_t channel){

	//disable the chip before configuring the device
	CE_Disable();

	nrf24_WriteReg(RF_CH, channel); 	//select the channel

	//select data pipe 1
	uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);
	en_rxaddr = en_rxaddr | ( 1<<2);
	nrf24_WriteReg(EN_RXADDR, en_rxaddr);	//select data pipe 1

	nrf24_WriteRegMulti(RX_ADDR_P1, Address, 5);	//Write the TX address

	nrf24_WriteReg(RX_PW_P1, 32); //32 bytes payload size

	//power up the device
	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1<<1) | (1<<0);
	nrf24_WriteReg (CONFIG, config);

	//Enable the chip after configuring the device
	CE_Enable();
}


uint8_t IsDataAvailable(int pipenum){
	uint8_t status = nrf24_ReadReg(STATUS);

	if ((status&(1<<6))&&(status&(pipenum<<1)))
	{
			nrf24_WriteReg(STATUS, (1<<6));

			return 1;
	}
	return 0;
}

void NRF24_Receive (uint8_t *data){
	uint8_t cmdtosend = 0;

	//select the device
	CS_Select();

	//payload command
	cmdtosend = R_RX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

	//send the payload
	HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);

	//unselect the device
	CS_UnSelect();

	HAL_Delay(1);

	cmdtosend = FLUSH_RX;
	nrfsendCmd(cmdtosend);

}
