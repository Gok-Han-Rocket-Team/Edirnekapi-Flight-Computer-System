#include "lora.h"
#include "main.h"



static void send_command(uint8_t header, uint8_t addresses, uint8_t dataLength, uint8_t *data) {
    uint8_t command[12];
    command[0] = header;
    command[1] = addresses;
    command[2] = dataLength;
    for (int i = 0; i < 9; i++) {
        command[3 + i] = data[i];
    }

    HAL_UART_Transmit(&huart4, command, 12, 100);
}

void lora_configure(lorastruct *config)
{
	uint8_t data[9];

    //default values of lora
    config->netId = (uint8_t) 0x00;
    config->serialParity = LORA_PARITY_8N1;
    config->ambientNoise = LORA_RSSI_AMBIENT_NOISE_DISABLE;
    config->RSSI = LORA_RSSI_DISABLE;
    config->transmissonMode = LORA_TRANSMISSION_TRANSPARENT;
    config->repeater = LORA_REPEATER_DISABLE;
    config->LBT = LORA_LBT_DISABLE;
    config->worMode = LORA_WOR_TRANSMITTER;
    config->worCycle = LORA_WOR_4000;

    // Lora address
    data[0] = config->loraAddress.address8[0];
    data[1] = config->loraAddress.address8[1];

    // Lora netid
    data[2] = config->netId;

    // Lora baud rate, parite, air rate
    data[3] = config->baudRate | config->serialParity | config->airRate;

    // packet size, ambient noise, reserve ve power
    data[4] = config->packetSize | config->ambientNoise | LORA_STATUS_LOG_DISABLE | config->power;

    // channel
    //frequency range restriction
    if(config->channel > 83)
    	config->channel = 83;
    else if(config->channel < 0)
    	config->channel = 0;

    data[5] = config->channel;

    // RSSI, transmission mode, repeater, LBT, worTransceiver ve worCycle
    data[6] = config->RSSI | config->transmissonMode | config->repeater | config->LBT | config->worMode | config->worCycle;

    // key
    data[7] = config->loraKey.key8[0];
    data[8] = config->loraKey.key8[1];

    send_command(0xC0, 0x00, 0x09, data);
}





//send command







