#include <stdint.h>
#include <math.h>
#include <DueTimer.h>

#define BUFFER_SIZE 1024
#define FC 2000
#define TC_US (1000000/FC)
#define THRESHOLD_HIGH 3 
#define THRESHOLD_LOW -3
#define MAX_CROSS 10

#define OFF_VOLTAGE 1589 //mV
#define DGT_OFF_VOLTAGE (OFF_VOLTAGE * 4095 + (3300/2))/3300 // Digit

#define OFF_CURRENT 1563 //mV 
#define DGT_OFF_CURRENT (OFF_CURRENT * 4095 + (3300/2))/3300 // Digit

#define GAIN_VLTG 3519.5/976 // Compensazione Guadagno Circuito per Tensione
#define GAIN_AMPR 5228.9/947 // Compensazione Guadagno Circuito per Corrente

#define GAIN_TRV  100 // Compensazione Guadagno Trasduttore per Tensione
#define GAIN_TRI  2 // Compensazione Guadagno Trasduttore per Corrente

int voltage = A0;
int current = A1;
int16_t voltages1[BUFFER_SIZE];
int16_t currents1[BUFFER_SIZE];
int16_t voltages2[BUFFER_SIZE];
int16_t currents2[BUFFER_SIZE];
int16_t *pVoltageW = voltages1;
int16_t *pCurrentR = NULL;
int16_t *pVoltageR = NULL;
int16_t *pCurrentW = currents1;

double trial=0;
int returnIndex[MAX_CROSS];
int ArrayPositionIndex = 0;
int threshold;
int nCross = 0; 
int NumberPointTotal;

double power(int16_t *voltage, int16_t *current, int16_t size){
  int64_t result=0;

  for(int k = 0;k < size; k++){
    result += voltage[k]*current[k];
  }
  double power= (1.0*result* GAIN_VLTG * GAIN_AMPR * GAIN_TRV *GAIN_TRI)/size;
  return (power*(3.3*3.3)/(4095*4095));
}
void triggerHighLevel(int16_t *pSamples) { //sto cercando un campione sopra soglia successivo a quello sotto
    while (ArrayPositionIndex < BUFFER_SIZE) {
        if (pSamples[ArrayPositionIndex] > threshold) {
            returnIndex[nCross] = ArrayPositionIndex;  //ho salvato qualcosa di potenzialmente sbagliato
            nCross++;
            threshold = THRESHOLD_LOW;
            return;
        }
        ArrayPositionIndex++;
    }
}

int trigger(int16_t *pSamples) {
  ArrayPositionIndex = nCross = 0;
  threshold = THRESHOLD_LOW;
  while ((ArrayPositionIndex < BUFFER_SIZE) && (nCross < MAX_CROSS)) {
      if (pSamples[ArrayPositionIndex] < threshold) {
          threshold = THRESHOLD_HIGH;
          triggerHighLevel(pSamples); // Non definito nel codice fornito    
      }
      ArrayPositionIndex++;
    }
    //Serial.println(nCross);

    return NumberPointTotal = returnIndex[nCross - 1] - returnIndex[0];
}

void acquire(){                //Del Professore: vietato toccare le sacre scritture
    //leggi il valore dal sensore

  *pVoltageW++ = analogRead(voltage) - DGT_OFF_VOLTAGE;
  *pCurrentW++ = analogRead(current) - DGT_OFF_CURRENT;

  if(pVoltageW == &voltages1[BUFFER_SIZE])
  {
    pVoltageR = voltages1;
    pCurrentR = currents1;
    //ping pong
    pVoltageW= voltages2;
    pCurrentW = currents2;
  }else if(pVoltageW == &voltages2[BUFFER_SIZE]){
    pVoltageR = voltages2;
    pCurrentR = currents2;
    //ping pong
    pVoltageW= voltages1;
    pCurrentW = currents1;
  }     
  
}

double rms(int16_t* samples, int len){
  double sumV=0;
  for(int j=0;j<len;j++){
    sumV = sumV + (samples[j]*samples[j]);
  }

  return (sqrt((sumV)/len)*(3300)/(4095)); //Convertire da digit in mVolt
}


void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  Timer3.attachInterrupt(acquire);
  Timer3.start(TC_US); // chiama ogni 0.1ms 
}

void loop() {
  if (pVoltageR != NULL){
    int size = trigger(pVoltageR);
    if (nCross < 2) {
      Serial.println("non sincronizzato!");
      size = BUFFER_SIZE;
    }
    
    double voltageValue = rms(pVoltageR, size)* GAIN_VLTG * GAIN_TRV;
    double currentValue = rms(pCurrentR, size)* GAIN_AMPR * GAIN_TRI;
    //Serial.println(size);
    Serial.print("\nValore Tensione (rms): ");
    Serial.print(voltageValue);
    Serial.print(" mV\n");
    
    Serial.print("Valore Corrente (rms): ");
    Serial.print(currentValue);
    Serial.print(" mA\n");
    
    double pwr= power(pVoltageR, pCurrentR, size);   
    double apparentPwr= ((double)voltageValue/1000 * (double)currentValue/1000);
    double pwrFactor = pwr/apparentPwr;
    double reactivePwr= sqrt(apparentPwr*apparentPwr-pwr*pwr);
    Serial.print("Potenza Attiva: ");
    Serial.print(pwr);
    Serial.print(" W\n");
    Serial.print("Potenza Apparente: ");
    Serial.print(apparentPwr);
    Serial.print(" VA\n");
    Serial.print("Fattore di Potenza: ");
    Serial.print(pwrFactor,6);
    Serial.print("\nPotenza non Attiva: ");
    Serial.print(reactivePwr);
    Serial.print(" VAR\n");
    pCurrentR = NULL;
    pVoltageR = NULL;
  }
}
