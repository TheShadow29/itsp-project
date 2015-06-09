/* Arduino USB Keyboard HID demo
*
* 0 is modifier, 1 is reserved, 2-7 machao
*
*/

/* Author: Darran Hunt
 * Released into the public domain.
 */

uint8_t buf[8];	/* Keyboard report buffer */
uint8_t clik[8] = {0};
uint8_t no[8] = {0};

#define KEY_LEFT_CTRL	0x01
#define KEY_LEFT_SHIFT	0x02
#define KEY_RIGHT_CTRL	0x10
#define KEY_RIGHT_SHIFT	0x20

#define W 0x52
#define A 0x04
#define S 0x51
#define D 0x07
#define CTRL 0xE0
#define SHIFT 0xE1
#define TAB 0x2B
#define SPACE 0x2C 
#define ESC 0x29
#define K 0x0E
#define G 0x0A
#define B 0x05
#define E 0x08
#define ONE 0x1E
#define TWO 0x1F
#define THREE 0x20
#define FOUR 0x21
#define FIVE 0x22
#define R 0x15

bool digitalPinReading[18  ];
bool previousReading[5] = {1};
int gunChanged = 0;
bool previousSPACE = 1;
bool previousG = 1;
int analogPinReading[2];

void setup() 
{
    Serial.begin(9600);
    clik[2] = 0x28;
    pinMode(0, INPUT);
    digitalWrite(0, HIGH);
    for(int i = 2; i < 14; ++i)
    {
      pinMode(i,INPUT);
      digitalWrite(i, HIGH);
    }
    digitalWrite(A0, HIGH);
    digitalWrite(A1, HIGH);
    digitalWrite(A2, HIGH);  
    digitalWrite(A3, HIGH);
    digitalWrite(A4, HIGH);
    digitalWrite(A5, HIGH);
}

void loop()
{
        for(int i = 0; i < 8; ++i)
        buf[i] = 0;
        
        digitalPinReading[0] = digitalRead(0);
        for(int i = 2; i < 14; ++i)
        {
          digitalPinReading[i] = digitalRead(i);
        }
        analogPinReading[0] = analogRead(A0);
        analogPinReading[1] = analogRead(A1);
        digitalPinReading[14] = digitalRead(A2);
        digitalPinReading[15] = digitalRead(A3);
        digitalPinReading[16] = digitalRead(A4);
        digitalPinReading[17] = digitalRead(A5);
        int index = 2; //stores the index of the current free keyboard
        
        if(!gunChanged)
        {
              
              if(analogPinReading[1] < 200 && index < 8)
              {
              buf[index] = W;
              index++;
              }
              if(analogPinReading[1] > 800 && index < 8)
              {
              buf[index] = S;
              index++;
              }
              if(analogPinReading[0] < 200 && index < 8)
              {
              buf[index] = D;
              index++;
              }
              if(analogPinReading[0] > 800 && index < 8)
              {
              buf[index] = A;
              index++;
              }
              if(digitalPinReading[15] == LOW && index < 8)
              {
              buf[index] = CTRL;
              index++;
              }
              if(digitalPinReading[3] == LOW && index < 8)
              {
              buf[index] = SHIFT;
              index++;
              }
              if(digitalPinReading[6] == LOW && index < 8)
              {
              buf[index] = SPACE;
              index++;
              }
              if(digitalPinReading[14] == LOW && index < 8)
              {
              buf[index] = TAB;
              index++;
              }
              if(digitalPinReading[7] == !previousG && index < 8)
              {
              buf[index] = G;
              index++;
              }
              if(digitalPinReading[17] == LOW && index < 8)
              {
              buf[index] = K;
              index++;
              }
              if(digitalPinReading[0] == LOW && index < 8)
              {
              buf[index] = B;
              index++;
              }
              if(digitalPinReading[16] == LOW && index < 8)
              {
              buf[index] = E;
              index++;
              }
              if(digitalPinReading[2] == LOW && index < 8)
              {
              buf[index] = ESC;
              index++;
              }
              gunChanged = 0;
              if(digitalPinReading[8] == LOW && previousReading[0] == HIGH && index < 8)
              {
              buf[index] = ONE;
              index++;
              gunChanged = 1;
              }
              if(digitalPinReading[9] == LOW && previousReading[1] == HIGH && index < 8)
              {
              buf[index] = TWO;
              index++;
              gunChanged = 1;
              }
              if(digitalPinReading[10] == LOW && previousReading[2] == HIGH && index < 8)
              {
              buf[index] = THREE;
              index++;
              gunChanged = 1;
              }
              if(digitalPinReading[11] == LOW && previousReading[3] == HIGH && index < 8)
              {
              buf[index] = FOUR;
              index++;
              gunChanged = 1;
              }
              if(digitalPinReading[12] == LOW && previousReading[4] == HIGH && index < 8)
              {
              buf[index] = FIVE;
              index++;
              gunChanged = 1;
              }
              if(digitalPinReading[13] == LOW && index < 8)
              {
              buf[index] = R;
              index++;
              }              
              Serial.write(buf, 8);	// Send keypress
              
              for(int i = 0; i < 5; ++i)
              previousReading[i] = digitalPinReading[i+8];
              
              previousSPACE = digitalPinReading[6];
              previousG = digitalPinReading[7];
        }
        else 
        {
              Serial.write(clik, 8);
              gunChanged = 0;
        }
    
}
