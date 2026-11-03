#include "dh11_temp.h"

#define DHT_PIN PD5

void DHT_start(){
	DDRD |= (1<<DHT_PIN); // Configurar como salida
	PORTD &= ~(1<<DHT_PIN); // Enviar señal de inicio (bajo)
	_delay_ms(18); // Esperar al menos 18 ms
	PORTD |= (1<<DHT_PIN); // Liberar la línea
	_delay_us(40); // Esperar 40 µs
}

uint8_t DHT_response() {
	DDRD &= ~(1<<DHT_PIN); // Configurar como entrada
	_delay_us(40);
	if (!(PIND & (1<<DHT_PIN))) { // Esperar respuesta baja del sensor
		_delay_us(80);
		if ((PIND & (1<<DHT_PIN))) { // Esperar respuesta alta del sensor
			_delay_us(80);
			return 1; // Respuesta válida
		}
	}
	return 0; // Sin respuesta
}

uint8_t obtener_temperatura_dht(void) {
	uint8_t dht_data[5]; // Array para almacenar los 5 bytes del sensor

	// Iniciar comunicación y verificar si el sensor responde
	DHT_start();
	if (DHT_response()) {
		
		// Leer secuencialmente los 5 bytes que envía el DHT
		dht_data[0] = DHT_read(); // Humedad entera
		dht_data[1] = DHT_read(); // Humedad decimal
		dht_data[2] = DHT_read(); // Temperatura entera 
		dht_data[3] = DHT_read(); // Temperatura decimal
		dht_data[4] = DHT_read(); // Checksum para verificar errores

		// Verificar el Checksum
		// Asegura que el cable no haya tenido ruido y la lectura sea real
		uint8_t suma = dht_data[0] + dht_data[1] + dht_data[2] + dht_data[3];
		if (suma == dht_data[4]) {
			return dht_data[2]; // Retorna con éxito el byte de temperatura entera
		}
	}
	
	// Si el sensor no respondió o el checksum falló, retorna un código de error arbitrario
	return 99;
}