#include<avr.io.h>
#include<util/delay.h>

#define	 ZERO       0x77 
#define	 ONE        0x41
#define	 TWO        0x3B
#define	 THREE      0x6B
#define	 FOUR       0x4D
#define	 FIVE       0x6E
#define	 SIX        0x7E
#define	 SEVEN      0x43
#define	 EIGHT      0x7F
#define	 NINE       0x6F
#define	 DOT        0x80

void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
/* Wait for completion of previous write */
while(EECR & (1<<EEWE));
/* Set up address and data registers */
EEAR = uiAddress;
EEDR = ucData;
/* Write logical one to EEMWE */
EECR |= (1<<EEMWE);
/* Start eeprom write by setting EEWE */
EECR |= (1<<EEWE);
}

unsigned char EEPROM_read(unsigned int uiAddress)
{
/* Wait for completion of previous write */
while(EECR & (1<<EEWE));
/* Set up address register */
EEAR = uiAddress;
/* Start eeprom read by writing EERE */
EECR |= (1<<EERE);
/* Return data from data register */
return EEDR;
}

public void set_digit(int dig)
{
	PORTB=0x00;
	switch(dig)
	{
		case 1: PORTB=ONE;
		break;
		case 2: PORTB=TWO;
		break;
		case 3: PORTB=THREE;
		break;
		case 4: PORTB=FOUR;
		break;
		case 5: PORTB=FIVE;
		break;
		case 6: PORTB=SIX;
		break;
		case 7: PORTB=SEVEN;
		break;
		case 8: PORTB=EIGHT;
		break;
		case 9: PORTB=NINE;
		break;
	}
}

main()
{
	DDRB=0xff;
	DDRD=0x00;
	int add = 0b00000101;	//Address to store the number of seconds in the delay part
	int tim = 5;
	int temp_tim = 0;
	/*
	*C0 - Output to motor - OP
	*C1 to C4 - BCD to 7seg IC - OP
	*C5 - LED1 - indicate operation - OP
	*C6 - LED2 - indicate power - OP
	*D0 - Trip switch input - IP
	*D2 - Up button - INT
	*D3 - Down button - INT
	*PORTB - 7seg display
	*/
	
	//Reading time data from EEPROM
	temp_tim = EEPROM_read(add);
	if((temp_tim>0)&&(temp_tim<10))	//Verify valid number
	{
		tim = temp_tim;
	}
	else
	{
		EEPROM_write(add, tim);
	}
	
	set_digit(tim);
	
	MCUCR|=(1<<ISC11);   //Falling edge on INT1 triggers interrupt.
	GICR|=(1<<INT1);  //Enable INT1 interrupt
	MCUCR|=(1<<ISC01);   //Falling edge on INT0 triggers interrupt.
	GICR|=(1<<INT0);  //Enable INT0 interrupt
	sei();
	
	while(1)
	{
		
		
		//Showing the EEPROM time on the 7seg display
		//Output goes to a BCD to 7seg display
		//Or directly to the digits
		if((PINA&0b00000001)==0b00000001)
		{
			PORTC&=0xff;
			_delay_ms(5*1000);
			PORTC|=0x00;
			_delay_ms(tim*1000);
		}
		
		
	}
}

ISR(INT0_vect)
{
   //CPU Jumps here automatically when INT1 pin detect a falling edge
   tim--;
   EEPROM_write(add, tim);
   set_digit(tim);
   
}

ISR(INT1_vect)
{
   tim++;
   EEPROM_write(add, tim);
   set_digit(tim);
}