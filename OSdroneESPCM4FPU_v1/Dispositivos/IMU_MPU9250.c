/*
 * IMU_MPU9250.c
 *
 *  Created on: 31/12/2016
 *      Author: Ruben
 */

#include "IMU_MPU9250.h"

bool Iniciar_IMU_MPU9250(I2C_Handle I2C, tpIMU9250 IMU92520){
	I2C_Transaction I2C_Transmision;
	uint8_t bufferEscritura[6] ={IMU_MPU9250_PWR_MGMT_1, 0, 0, 0, 0, 0}; //Ponemos a cero el registro Reg_Power_Managent_1 para arrancar;
	bool TransmisionOK;

	I2C_Transmision.slaveAddress = IMU92520.Direccion_IMU;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 2;
	I2C_Transmision.readBuf = NULL;
	I2C_Transmision.readCount = 0;

	TransmisionOK = I2C_transfer(I2C, &I2C_Transmision);

	bufferEscritura[0] = IMU_MPU9250_SMPLRT_DIV;
	bufferEscritura[1] = IMU92520.SMPLRT_DIV;
	bufferEscritura[2] = IMU92520.DLPF_CFG_GYRO;
	bufferEscritura[3] = IMU92520.Ganancia_Gyro << 3;
	if (IMU92520.DLPF_CFG_GYRO == -1){
		bufferEscritura[3] |= 0b00000011;
	}
	bufferEscritura[4] = IMU92520.Ganancia_Acel << 3;
	bufferEscritura[5] = IMU92520.DLPF_CFG_ACEL;
	if (IMU92520.DLPF_CFG_ACEL == -1){
		bufferEscritura[3] |= 0b00000001;
	}

	I2C_Transmision.writeCount = 5;
	TransmisionOK |= I2C_transfer(I2C, &I2C_Transmision);


	bufferEscritura[0] = IMU_MPU9250_INT_PIN_CFG;
	bufferEscritura[1] = 0x02;

	I2C_Transmision.writeCount = 2;
	TransmisionOK |= I2C_transfer(I2C, &I2C_Transmision);

	//brujula

	bufferEscritura[0] = IMU_MPU9250_MAG_CTL1;
	bufferEscritura[1] = 0x16;

	I2C_Transmision.writeCount = 2;
	I2C_Transmision.slaveAddress = IMU92520.Direccion_MAG;
	TransmisionOK |= I2C_transfer(I2C, &I2C_Transmision);

	return(TransmisionOK);
}

bool Leer_IMU_MPU9250(I2C_Handle I2C, tpIMU9250 IMU92520, tpLecturasIMU *LecturasIMU){
	I2C_Transaction I2C_Transmision;

	uint8_t bufferEscritura[] = {IMU_MPU6050_ACCEL_XOUT_H};
	uint8_t bufferLectura[14];
	bool TransmisionOK;

	I2C_Transmision.slaveAddress = IMU92520.Direccion_IMU;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 1;
	I2C_Transmision.readBuf = bufferLectura;
//	I2C_Transmision.readBuf = &(Lecturas_IMU->Reg.x_acel_h);
	I2C_Transmision.readCount = 14;

	TransmisionOK = I2C_transfer(I2C, &I2C_Transmision);

	LecturasIMU->x_acel = bufferLectura[1]<<8 | bufferLectura[0];
	LecturasIMU->y_acel = bufferLectura[3]<<8 | bufferLectura[2];
	LecturasIMU->z_acel = bufferLectura[5]<<8 | bufferLectura[4];

	LecturasIMU->temp = bufferLectura[7]<<8 | bufferLectura[6];

	LecturasIMU->x_vel = bufferLectura[9]<<8 | bufferLectura[8];
	LecturasIMU->y_vel = bufferLectura[11]<<8 | bufferLectura[10];
	LecturasIMU->z_vel = bufferLectura[13]<<8 | bufferLectura[12];

	TransmisionOK |= I2C_transfer(I2C, &I2C_Transmision);
	return(TransmisionOK);
}
