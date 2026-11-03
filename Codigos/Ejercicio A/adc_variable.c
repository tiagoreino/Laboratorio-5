#include "adc_variable.h"

//inicializar ADC
void ADC_init(uint8_t preescaler)
{
	// Referencia AVcc
	ADMUX = (1 << REFS0);

	if(preescaler == 2)
	{
		ADCSRA = (1 << ADEN)  |
		(0 << ADPS2) |
		(0 << ADPS1) |
		(0 << ADPS0);	
	}
	else if (preescaler == 4)
	{
		ADCSRA = (1 << ADEN)  |
		(0 << ADPS2) |
		(1 << ADPS1) |
		(0 << ADPS0);
	}
	else if (preescaler == 8)
	{
		ADCSRA = (1 << ADEN)  |
		(0 << ADPS2) |
		(1 << ADPS1) |
		(1 << ADPS0);
	}
	else if (preescaler == 16)
	{
		ADCSRA = (1 << ADEN)  |
		(1 << ADPS2) |
		(0 << ADPS1) |
		(0 << ADPS0);
	}
	else if (preescaler == 32)
	{
		ADCSRA = (1 << ADEN)  |
		(1 << ADPS2) |
		(0 << ADPS1) |
		(1 << ADPS0);
	}
	else if (preescaler == 64)
	{
		ADCSRA = (1 << ADEN)  |
		(1 << ADPS2) |
		(1 << ADPS1) |
		(0 << ADPS0);
	}
	else if (preescaler == 128)
	{
		ADCSRA = (1 << ADEN)  |
		(1 << ADPS2) |
		(1 << ADPS1) |
		(1 << ADPS0);
	}
	else
	{
		ADCSRA = (0 << ADEN);
	}
}

// LEER ADC
uint16_t ADC_leer(uint8_t canal)
{
	// Seleccionar referencia AVcc y canal ADC
	ADMUX = (1 << REFS0) | (canal & 0x0F);

	// Iniciar conversión
	ADCSRA |= (1 << ADSC);

	// Esperar hasta que termine
	while (ADCSRA & (1 << ADSC));

	return ADC;
}