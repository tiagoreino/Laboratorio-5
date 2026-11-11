//A0: salida del LM358. D2: contacto NC de S1, con pull-up interno
//-> Puerto USB serie: 115200 baudios, 8N1 8 bits de datos sin paridad y un bits de parada
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#if F_CPU != 16000000UL
#error "Este programa está configurado para un reloj de 16 MHz."
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdio.h>

#define TAM_COLA 32
#define PERIODO_MS 10
#define ANTIRREBOTE_MS 30

//Estructura de datos que se enviará por UART
typedef struct {
    uint32_t ms, secuencia;
    uint16_t adc;
    char tipo;
} Registro;

static volatile Registro cola[TAM_COLA];
static volatile uint8_t entrada, salida, sincronizar;

//El temporizador y el ADC comparten estos datos dentro de sus ISR. 
static volatile uint32_t tiempo_adc;
static uint32_t ms, secuencia;
static uint8_t divisor, nivel_anterior, nivel_estable, lecturas_iguales;

static void inicializar(void);
static void encolar(char tipo, uint32_t tiempo, uint16_t adc);
static int enviar_caracter(char c, FILE *flujo);
static FILE uart = FDEV_SETUP_STREAM(enviar_caracter, NULL, _FDEV_SETUP_WRITE); //Tipo de datos FILE encargados de enviar caracteres por UART, el parámetro NULL es porque no se recibe ningún dato para esta instrucción.

//-> Comados declarados con letras para enviar por UART y establecer una mejor comunicación entre Arduino y Monitor
/* Trama terminadas en LF (Salto de Línea):
 * H,LAB5,2           Arranque del medidor.
 * S,ms,secuencia,adc Muestra. ms se toma al iniciar la conversión.
 * P,ms,secuencia     Presión validada.
 * Y,ms,secuencia     Comando 'A' aceptado con S1 suelto.
 * N,ms,secuencia     Comando 'A' rechazado: S1 no estaba suelto y estable.
 * La secuencia es común a todas las tramas fechadas y avanza aunque se llene la cola. El PC usa los huecos para detectar pérdidas.
 */
/*
	H	Header (cabecera)	Identifica el arranque del medidor
	S	Sample (muestra)	Transmite una lectura ADC
	P	Pulsación	Informa de una pulsación validada de S1
	Y	Yes (sí)	Confirma que se aceptó la sincronización
	N	No	Informa que se rechazó la sincronización
*/
int main(void) {
    inicializar();
 //El LF inicial separa un posible mensaje truncado al reiniciar. 
    puts("\nH,LAB5,2");

    while(1) {
        if (UCSR0A & _BV(RXC0)) { //AND a nivel de bits, condicional usado para comprobar mediante A (caracter recibido) la sincronización de Python con el Arduino 
            uint8_t estado_uart = UCSR0A; //Se guarda el valor del registro UCSR0A para conservar banderas de error antes de leer el registro UDR0
            uint8_t comando = UDR0; //Registro que almacena los datos recibidos y transmitidos
            if (comando == 'A' && !(estado_uart & (_BV(FE0) | _BV(DOR0) | _BV(UPE0)))) { // Comprobar comando y banderas de error guardadas 
                sincronizar = 1;
            }
        }

        Registro registro;
        uint8_t disponible = 0;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // Con ATOMIC evitamos problemas de concurrencia a la hora de leer nuestros datos
            if (salida != entrada) {
                registro = cola[salida];
                salida = (salida + 1) & (TAM_COLA - 1);
                disponible = 1;
            }
        }
        if (disponible) { //Instrucción encargada de formar la cadena de caracteres y enviarla por UART, el printf se encarga del envio por UART
            printf("%c,%lu,%lu", registro.tipo, (unsigned long)registro.ms, (unsigned long)registro.secuencia);
            if (registro.tipo == 'S') {
                printf(",%u", registro.adc);
            }
            putchar('\n'); //Salto de línea
        }
    }
}

static void inicializar(void) {
    DDRD &= ~_BV(PD2);
    PORTD |= _BV(PD2);
    nivel_anterior = nivel_estable = !!(PIND & _BV(PD2)); //Convertir a booleano la lectura de PD2

    //U2X y UBRR=16: ajuste de Ard. UNO para 115200 a 16 MHz.
    UCSR0A = _BV(U2X0); // Se establece un 1 en este bits para activar el modo de doble velocidad de comunicación, se mantiene el modo asíncrono
	//Registro USART Baud Rate Register 0 - Parte alta y Parte baja - Los utilizamos para establecer los baudios de transmisión (Baudios = Fcpu / (8*(UBRR0+1)) -> (Baudios = 16MHz / (8*(16+1)) = 117647
    UBRR0H = 0;  
    UBRR0L = 16;//
    UCSR0B = _BV(TXEN0) | _BV(RXEN0); //Habilitar recepción y transmisión de datos
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);//Definen configuracion de la comunicacion . Paridad, modo asíncrono ...
    stdout = &uart;// flujo principal que especifica que las instrucciones printf() , puts() , putchar() envien directamente los datos por UART

  // Configuracion del ADC en A0, referencia AVcc - 5v , reloj 125 kHz. Descartar arranque.
    ADMUX = _BV(REFS0);
    DIDR0 = _BV(ADC0D);//Se deshabilita el pin digital del A0 para evitar consumos adicionales y evitar errores en la medición
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); //Habilitacion de la medicion y definicion del preescaler (111 -> 128) -> Fadc = Fcpu/128
    ADCSRA |= _BV(ADSC);//Inicia la conversión ADC
    while (ADCSRA & _BV(ADSC)) {}
    ADCSRA |= _BV(ADIF) | _BV(ADIE);//Termina la medición , limpiamos banderas y habilitamos interrupción.

  // Timer0 CTC: 250 cuentas de 4 us por interrupción de 1 ms. 
    TCCR0A = _BV(WGM01); //Modo CTC
    OCR0A = 249;
    TIMSK0 = _BV(OCIE0A); //Habilitar interrupción
    TCCR0B = _BV(CS01) | _BV(CS00); //Selecionar el preescaler -> (011 -> 64) Frecuencia_Timer = 16MHz/64 -> 250kHz -> 1/250KHz = 4us -> T_interrupción =250*4us = 1ms 
    sei();
}

//ISR de Timer0 e ISR de ADC. Consumidor
static void encolar(char tipo, uint32_t tiempo, uint16_t adc) {
    uint32_t numero = secuencia++;
    uint8_t siguiente = (entrada + 1) & (TAM_COLA - 1);
    if (siguiente != salida) {
        cola[entrada].tipo = tipo;
        cola[entrada].ms = tiempo;
        cola[entrada].secuencia = numero;
        cola[entrada].adc = adc;
        entrada = siguiente;
    }
}

static int enviar_caracter(char c, FILE *flujo) {
    (void)flujo;
    //La espera sucede en main con las interrupciones habilitadas.
    while (!(UCSR0A & _BV(UDRE0))) {}
    UDR0 = c;
    return 0;
}

ISR(TIMER0_COMPA_vect) {
    ++ms;
//Condiciones para aplicar el antirrebote cuando presionamos el botón, este botón es normal cerrado por tanto el programa lee el cambio cuando su PD2 pase a HIGH
    uint8_t nivel = !!(PIND & _BV(PD2));
    if (nivel != nivel_anterior) {
        nivel_anterior = nivel;
        lecturas_iguales = 0;
    }
    if (lecturas_iguales < ANTIRREBOTE_MS) {
        ++lecturas_iguales;
    }
    if (lecturas_iguales == ANTIRREBOTE_MS && nivel_estable != nivel) {
        nivel_estable = nivel;
        if (nivel) {
            encolar('P', ms, 0); //Si se cumple el antirrebote de 30ms se envía por UART el cambio de estado y en que ms ocurrió
        }
    }

    if (sincronizar) { //Instrucción para cuando se recibe el comando de sincronizar
        sincronizar = 0;
        char respuesta = !nivel && lecturas_iguales == ANTIRREBOTE_MS ? 'Y' : 'N';
        encolar(respuesta, ms, 0);
    }

    if (++divisor == PERIODO_MS) { //Instruccion que inicia la conversion ADC cada 10 ms
        divisor = 0;
        tiempo_adc = ms;
        ADCSRA |= _BV(ADSC);
    }
}

ISR(ADC_vect) {
    encolar('S', tiempo_adc, ADC);
}

