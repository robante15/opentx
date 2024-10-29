/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "i2c_driver.h"
#include "opentx.h"
#include "board.h"

/**
 * @brief Initializes the I2C peripheral and configures the associated GPIO pins.
 *
 * This function performs the following steps:
 * 1. Deinitializes the I2C peripheral to reset its state.
 * 2. Configures the I2C peripheral with specific settings such as timing, mode, acknowledgment, and filters.
 * 3. Enables the I2C peripheral.
 * 4. Configures the GPIO pins for I2C SCL and SDA with alternate function, speed, mode, output type, and pull-up/pull-down settings.
 *
 * The function uses predefined macros and constants for the I2C peripheral, GPIO pins, and their configurations.
 */
void i2cInit()
{
  I2C_DeInit(I2C);

  I2C_InitTypeDef I2C_InitStructure;
  I2C_InitStructure.I2C_Timing = I2C_TIMING;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Disable;
  I2C_InitStructure.I2C_DigitalFilter = 0x00;
  I2C_Init(I2C, &I2C_InitStructure);
  I2C_Cmd(I2C, ENABLE);

  GPIO_PinAFConfig(I2C_GPIO, I2C_SCL_GPIO_PinSource, I2C_GPIO_AF);
  GPIO_PinAFConfig(I2C_GPIO, I2C_SDA_GPIO_PinSource, I2C_GPIO_AF);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = I2C_SCL_GPIO_PIN | I2C_SDA_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(I2C_GPIO, &GPIO_InitStructure);
}

#define I2C_TIMEOUT_MAX 1000
/**
 * @brief Waits for a specific I2C event to occur.
 *
 * This function waits until the specified I2C event occurs or a timeout is reached.
 *
 * @param event The I2C event to wait for.
 * @return true if the event occurred before the timeout, false otherwise.
 */
bool I2C_WaitEvent(uint32_t event)
{
  uint32_t timeout = I2C_TIMEOUT_MAX;
  while (!I2C_GetFlagStatus(I2C, event))
  {
    if ((timeout--) == 0)
      return false;
  }
  return true;
}

/**
 * @brief Waits for a specified I2C event to be cleared.
 *
 * This function continuously checks the status of a specified I2C event flag
 * and waits until it is cleared or a timeout occurs.
 *
 * @param event The I2C event flag to wait for.
 * @return true if the event was cleared before the timeout, false otherwise.
 */
bool I2C_WaitEventCleared(uint32_t event)
{
  uint32_t timeout = I2C_TIMEOUT_MAX;
  while (I2C_GetFlagStatus(I2C, event))
  {
    if ((timeout--) == 0)
      return false;
  }
  return true;
}

/**
 * @brief Reads a block of data from an I2C device.
 *
 * This function initiates an I2C read operation to retrieve a block of data from a specified device.
 * It handles the necessary I2C communication steps, including sending the device address, memory address,
 * and reading the requested number of bytes.
 *
 * @param deviceAddress The I2C address of the target device.
 * @param pBuffer Pointer to the buffer where the read data will be stored.
 * @param memAddr The memory address within the device from which to start reading.
 * @param memAddrSize The size of the memory address in bytes (e.g., 1 or 2 bytes).
 * @param numBytesToRead The number of bytes to read from the device.
 * @return true if the read operation was successful, false otherwise.
 */
bool I2C_ReadBlock(uint8_t deviceAddress, uint8_t *pBuffer, uint16_t memAddr, uint8_t memAddrSize, uint16_t numBytesToRead)
{
  // Espera que el bus I2C esté libre
  if (!I2C_WaitEventCleared(I2C_FLAG_BUSY))
    return false;

  // Inicia comunicación y envía la dirección del dispositivo en modo escritura para la fase de envío de memoria
  I2C_TransferHandling(I2C, deviceAddress, memAddrSize, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

  // Enviar la dirección de memoria (si se requiere)
  if (memAddrSize > 0)
  {
    for (int i = 0; i < memAddrSize; i++)
    {
      uint8_t addrByte = (memAddr >> (8 * (memAddrSize - 1 - i))) & 0xFF;
      if (!I2C_WaitEvent(I2C_FLAG_TXIS)) // Espera a que el transmisor esté listo
        return false;
      I2C_SendData(I2C, addrByte); // Envía cada byte de la dirección de memoria
    }
  }

  // Espera a que la transmisión de la dirección esté completa
  if (!I2C_WaitEvent(I2C_FLAG_TC))
    return false;

  // Configura el I2C para leer y solicita `numBytesToRead` bytes del dispositivo
  I2C_TransferHandling(I2C, deviceAddress, numBytesToRead, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

  // Leer los datos en un bucle
  while (numBytesToRead > 0)
  {
    if (!I2C_WaitEvent(I2C_FLAG_RXNE)) // Espera a que los datos estén listos
      return false;

    *pBuffer++ = I2C_ReceiveData(I2C); // Lee cada byte y avanza el puntero
    numBytesToRead--;
  }

  // Espera la señal de fin de transferencia y detención
  if (!I2C_WaitEvent(I2C_FLAG_STOPF))
    return false;

  return true; // Lectura exitosa
}

/**
 * @brief Writes a block of data to an I2C device.
 *
 * This function writes a specified number of bytes from a buffer to a given
 * memory address on an I2C device. It handles the I2C communication protocol,
 * including waiting for the bus to be free, sending the device address and
 * memory address, and writing the data bytes.
 *
 * @param deviceAddress The address of the I2C device.
 * @param pBuffer Pointer to the buffer containing the data to be written.
 * @param memAddr The memory address on the I2C device where the data will be written.
 * @param memAddrSize The size of the memory address in bytes (e.g., 1 or 2 bytes).
 * @param numBytesToWrite The number of bytes to write from the buffer.
 * @return true if the write operation was successful, false otherwise.
 */
bool I2C_WriteBlock(uint8_t deviceAddress, uint8_t *pBuffer, uint16_t memAddr, uint8_t memAddrSize, uint8_t numBytesToWrite)
{
  // Espera que el bus I2C esté libre
  if (!I2C_WaitEventCleared(I2C_FLAG_BUSY))
    return false;

  // Inicia la transferencia y envía la dirección del dispositivo en modo escritura para el envío de la dirección de memoria
  I2C_TransferHandling(I2C, deviceAddress, memAddrSize, I2C_Reload_Mode, I2C_Generate_Start_Write);

  if (!I2C_WaitEvent(I2C_FLAG_TXIS))
    return false;

  // Envía cada byte de la dirección de memoria, si se especifica `memAddrSize`
  if (memAddrSize > 0)
  {
    for (int i = 0; i < memAddrSize; i++)
    {
      uint8_t addrByte = (memAddr >> (8 * (memAddrSize - 1 - i))) & 0xFF;
      if (!I2C_WaitEvent(I2C_FLAG_TXIS)) // Espera a que el transmisor esté listo
        return false;
      I2C_SendData(I2C, addrByte); // Envía cada byte de la dirección de memoria
    }
  }

  // Espera a que la transmisión de la dirección esté completa antes de enviar los datos
  if (!I2C_WaitEvent(I2C_FLAG_TCR))
    return false;

  // Configura el periférico para enviar los datos con modo `AutoEnd`
  I2C_TransferHandling(I2C, deviceAddress, numBytesToWrite, I2C_AutoEnd_Mode, I2C_No_StartStop);

  // Escribe los datos desde el buffer, byte por byte
  while (numBytesToWrite--)
  {
    if (!I2C_WaitEvent(I2C_FLAG_TXIS)) // Espera a que el transmisor esté listo para el siguiente byte
      return false;

    I2C_SendData(I2C, *pBuffer++); // Envía el byte actual y avanza el puntero del buffer
  }

  // Espera la señal de fin de transmisión
  if (!I2C_WaitEvent(I2C_FLAG_STOPF))
    return false;

  return true; // Escritura exitosa
}

/**
 * @brief Writes data to an I2C device in pages.
 *
 * This function writes data to an I2C device, handling the data in pages of a specified size.
 * It ensures that data is written correctly even if the memory address is not aligned with the page size.
 *
 * @param deviceAddress The I2C address of the device to write to.
 * @param buffer Pointer to the data buffer to be written.
 * @param memAddr The starting memory address in the device where the data will be written.
 * @param size The total size of the data to be written.
 * @param pageSize The size of each page to be written.
 * @return true if the data was written successfully, false otherwise.
 */
bool I2C_WriteInPages(uint8_t deviceAddress, uint8_t *buffer, size_t memAddr, size_t size, uint8_t pageSize)
{
  uint8_t offset = memAddr % pageSize;
  uint8_t count = pageSize - offset;

  if (size < count)
  {
    count = size;
  }

  while (size > 0)
  {
    if (!I2C_WriteBlock(deviceAddress, buffer, memAddr, 2, count))
    {
      i2cInit(); // Reinitialize if there's an error
      continue;
    }

    // Actualiza los punteros y contadores
    memAddr += count;
    buffer += count;
    size -= count;

    // Recalcula el tamaño para la próxima página
    count = (size < pageSize) ? size : pageSize;
  }
  return true;
}