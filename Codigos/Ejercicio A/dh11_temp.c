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

uint8_t DHT_read(void) {
	uint8_t resultado = 0;
	for (uint8_t j = 0; j < 8; j++) {
		// Esperar a que el pin de datos se ponga en ALTO (fin del pulso bajo de sincronización de 50us)
		while (!(PIND & (1 << DHT_PIN)));
		
		// El pin está en ALTO. Esperamos 40 microsegundos para muestrear
		_delay_us(40);
		
		if (!(PIND & (1 << DHT_PIN))) {
			// Si el pin ya bajó antes de los 40us, el pulso alto duró poco (~26-28us) -> Es un '0'
			resultado <<= 1;
			} else {
			// Si el pin sigue en ALTO después de 40us, el pulso es largo (~70us) -> Es un '1'
			resultado = (resultado << 1) | 0x01;
			
			// Esperar a que el pin vuelva a BAJO para no engancharse en el siguiente bit
			while (PIND & (1 << DHT_PIN));
		}
	}
	return resultado;
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