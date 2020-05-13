
//#include <Controllino.h>
#include "ModbusRtu.h"
#include "afl_call.h"

#define CONTROLLINO_A1 1
#define CONTROLLINO_D6 6
#define CONTROLLINO_D0 0
#define CONTROLLINO_D1 1

#define SalidaRelay  CONTROLLINO_D0 //pin que controla la salida de rele PID

// data array for modbus network sharing
uint16_t au16data[32],data8024[10];
uint8_t u8state;
char bufferIN[64];
int i,j;
String RxString;
int estado=0,minutos,segundos,minutosp,segundosp,mmres,ssres,mmt,sst,sta,stc;
int terminar_ciclo=0,tiempo_espera,espera_plancha,tempe_z1,tempe_z2,duty1,duty2;
float temperatura1,temperatura2,ep1=0,e1=0,ep2=0,e2=0;
float kpa1=0,kia1=0,kda1=0,kpp1=0,kip1=0,kdp1=0,kpa2=0,kia2=0,kda2=0,kpp2=0,kip2=0,kdp2=0,YN,YNp;
int pid1,pid2,tiempo_alto1,tiempo_bajo1,incre,in1,in2,in3,in4,in5,in6,in7,in8;

Modbus master(0,0, CONTROLLINO_D6); // this is master and RS-232 or USB-FTDI

modbus_t telegram[6];
unsigned long u32wait,tiempopid1,reloj,dutyalto1,dutybajo1;





void setup() {
  
  pinMode(CONTROLLINO_A1, INPUT);
  
  pinMode(CONTROLLINO_D6, OUTPUT);
  pinMode(CONTROLLINO_D0, OUTPUT);
  pinMode(CONTROLLINO_D1, OUTPUT);
  digitalWrite(SalidaRelay, LOW);
  digitalWrite(CONTROLLINO_D1,LOW);

  //**** PREPARACION DE TELEGRAMAS
  
  // para lectura
    telegram[0].u8id = 1; // slave address
    telegram[0].u8fct = 3; // function code (this one is registers read)
    telegram[0].u16RegAdd = 0; // start address in slave
    telegram[0].u16CoilsNo = 16; // number of elements (coils or registers) to read
    telegram[0].au16reg = au16data; // pointer to a memory array in the Arduino
  // para escritura
    telegram[1].u8id = 1; // slave address
    telegram[1].u8fct = 16; // function code (this one is registers read)
    telegram[1].u16RegAdd = 19; // start address in slave
    telegram[1].u16CoilsNo = 10; // number of elements (coils or registers) to read
    telegram[1].au16reg = au16data+19; // pointer to a memory array in the Arduino
  // para lectura
    telegram[2].u8id = 1; // slave address
    telegram[2].u8fct = 3; // function code (this one is registers read)
    telegram[2].u16RegAdd = 20; // start address in slave
    telegram[2].u16CoilsNo = 12; // number of elements (coils or registers) to read
    telegram[2].au16reg = au16data+20; // pointer to a memory array in the Arduino
 // para escritura
    telegram[3].u8id = 1; // slave address
    telegram[3].u8fct = 16; // function code (this one is registers read)
    telegram[3].u16RegAdd = 28; // start address in slave
    telegram[3].u16CoilsNo = 3; // number of elements (coils or registers) to read
    telegram[3].au16reg = au16data+28; // pointer to a memory array in the Arduino
// lectura 8024 ADAM
    telegram[4].u8id = 2; // slave address
    telegram[4].u8fct = 2; // function code (this one is registers read)
    telegram[4].u16RegAdd = 0; // start address in slave
    telegram[4].u16CoilsNo = 8; // number of elements (coils or registers) to read
    telegram[4].au16reg = data8024; // pointer to a memory array in the Arduino  
// escritura 8024 ADAM
    telegram[5].u8id = 0X02; // slave address
    telegram[5].u8fct = 0X05; // function para escribir un solo rele.
    telegram[5].u16RegAdd = 0X0001; // empezar a mandar los datos desde el registro data8024[1];
    telegram[5].u16CoilsNo = 0X0001; // numero de elementos a escribir, en este caso [1]
    telegram[5].au16reg = data8024+1; // posicionamiento en el lugar de memoria 1 ([0],[1])  
//data8024[0]=0x0001; ENCIENDE RELE 1
//data8024[1]=0x0000; //ENCIENDE RELE 2
  master.begin( 19200 ); // baud-rate at 19200
  master.setTimeOut( 2000 ); // if there is no answer in 2000 ms, roll over
  u32wait = millis();
  tiempopid1= millis()+1000;
  dutyalto1= millis()+1000;
  dutybajo1= millis()+1000;
  reloj= millis()+1000;
  u8state = 0;
  startForkserver(0);
}

void loop() {

switch( u8state ) {
  case 0: 
    if (millis() > u32wait) u8state++; // wait state
    break;
  
  case 1:
    master.poll(); // check incoming messages
    
   if (master.getState() == COM_IDLE){
    master.query( telegram[0] ); // send query (only once)
    u8state++;      
    u32wait=millis();
    tempe_z1=au16data[1];
    tempe_z2=au16data[2];
    kpa1=au16data[6];
    kia1=(au16data[7]/10);
    kda1=au16data[8];
    duty1=au16data[9];
    duty2=au16data[12];
    minutos=au16data[14];
    segundos=au16data[15];
    }    
    break;

  case 2:
    master.poll(); // check incoming messages
    
    if (master.getState() == COM_IDLE) // COM_IDLE= desocupado o fin de recepcion de respuesta a telegrama
    {      
        u8state++;
        u32wait=millis();       
    }
    break;

  case 3:
   master.poll(); // check incoming messages
      
   if (master.getState() == COM_IDLE){
    master.query( telegram[2] ); // send query (only once)
    u8state++;      
    u32wait=millis();
    estado=au16data[29];
    tiempo_espera=au16data[30];

  }     
    break;   

    case 4:
    master.poll(); // check incoming messages
    
    if (master.getState() == COM_IDLE) // COM_IDLE= desocupado o fin de recepcion de respuesta a telegrama
    {      
        u8state++;
        u32wait=millis();       
    }
    break;
    

  case 5:
   master.poll(); // check incoming messages
      
   if (master.getState() == COM_IDLE){

      
    master.query( telegram[1] ); // send query (only once)
    u8state++; 
  }     
    break;

   case 6:
    master.poll(); // check incoming messages
    
    if (master.getState() == COM_IDLE) // COM_IDLE= desocupado o fin de recepcion de respuesta a telegrama
    {      
        u8state++;
        u32wait=millis();       
    }
    break;
    

  case 7:
   master.poll(); // check incoming messages
   if(terminar_ciclo==1){   
   if (master.getState() == COM_IDLE){
    
      au16data[29]=0;
      terminar_ciclo=0;
      
     
      
    master.query( telegram[3] ); // send query (only once)
    }
     
  } 
  u8state++;    
    break;  

 case 8:
    master.poll(); // check incoming messages
    
    if (master.getState() == COM_IDLE) // COM_IDLE= desocupado o fin de recepcion de respuesta a telegrama
    {      
        u8state++;
        u32wait=millis();       
    }
    break;
    

  case 9:
   master.poll(); // check incoming messages

        
   if (master.getState() == COM_IDLE){
    
        master.query( telegram[4] ); // send query (only once)
        u32wait=millis(); 
    }

  u8state=0;    
    break;  

 /*case 10:
    master.poll(); // check incoming messages
    
    if (master.getState() == COM_IDLE) // COM_IDLE= desocupado o fin de recepcion de respuesta a telegrama
    {      
        u8state++;
        u32wait=millis();       
    }
    break;
    
      case 11:
      if(millis()-50>= u32wait){ 
        u8state++;     
      }
      break;
      
   case 12:            
      bitSet(UCSR0A,TXC0); //seteo de bandera de buffer vacío
      digitalWrite(CONTROLLINO_D6, HIGH); //habilita el transceiver RS485 para escritura
      // AQUI UTILIZO EL COMANDO #02 PARA QUE EL MODULO ADAM ME DEVUELVA TODAS LAS LECTURAS DE LAS ENTRADAS  SIN NECESIDAD DE EJECUTAR DOS VECES LA LECTURA
      Serial.write("#02\r"); //escribe el comando "\r" es equivalente a "carrier return" o chr(13) o vbcr 
      while (bitRead(UCSR0A,TXC0)==0){
       //espera a que todos los bytes hayan sido transmitidos
       // ESTA PARTE DE LA LIBRERIA ES BLOQUEANTE PERO TARDA MUY POCO TIEMPO
      } 
      digitalWrite(CONTROLLINO_D6, LOW); //habilita el transceiver RS485 para lectura luego de finalizada la transmisión
      
      while(RxString.length()) // AQUI SE LIMPIA EL OBJETO STRING DE CUALQUIER CADENA RECIBIDA ANTERIORMENTE
      {
        RxString.remove(0);
      }
       
     
      i= Serial.readBytes(bufferIN,58);  //lee 58 bytes DEL PUERTO SERIAL y de no llegar esa cantidad sale luego de timeout
                                         // AQUI SE LEE 58 BYTES PORQUE ESA ES LA LONGITUD DE LA TRAMA DE RESPUESTA DEL MODULO ADAM
                                         // ESTE MODULO SIEMPRE TRANSMITE 58 BYTES YA SEA QUE HAYA HABILITADO UNA O VARIAS ENTRADAS ANALOGAS
                                         // IMPORTANTE:
                                         // VERIFICAR QUE EL MODULO ADAM ESTE CONFIGURADO PARA LEER 2 TERMOCUPLAS
                                         // EN LAS PRUEBAS QUE YO REALICE EN LA OFICIAN HAILITE DOS Y LUEGO SOLO UNA NO RECUERDO COMO LO DEJE
      for (j=1;j<8;j++){
       RxString+=bufferIN[j];
      }
      temperatura1=RxString.toFloat();
      //au16data[19]=(int)(temperatura1); //copia la variable del Adam hacia modbus
      au16data[19]=(int)(temperatura1*10);

      
       while(RxString.length()) // AQUI SE LIMPIA EL OBJETO STRING DE CUALQUIER CADENA RECIBIDA ANTERIORMENTE
      {
        RxString.remove(0);
      }

      for (j=8;j<16;j++){ // AQUI VERIFICAR LOS INDICES QUE PUSE NO TENGO A MANO EL MODULO ADAM PERO CREO SON CORRECTOS
                          // SE PUEDE USAR EL PROGRAMA QUE HICE EN VISUAL BASIC PARA VERIFICAR LOS INDICES EXACTOS DONDE SE ENCUENTRA LA SEGUNDA LECTURA DE TEMPERATURA
       RxString+=bufferIN[j];
      }
      temperatura2=RxString.toFloat();
      //au16data[20]=(int)(temperatura2);
      //au16data[20]=(int)(temperatura2*10);
      
      // COMO YA HEMOS FINALIAZADO LA LECTURA/ESCRITURA POR MODBUS Y/O EL MODULO ADAM, ES CORRECTO RETORNAR AL ESATOD CERO REALIZANDO UNA PEQUEÑA
      // PAUSA, PARA LO CUAL ACTUALIZAMOS EL LA VARIABLE USADA COMO TIMER Y BRINCAMOS AL ESTADO 0
      u8state=0;      
      u32wait=millis();  // IMPORTANTE: SIEMPRE QUE VAMOS HACER UNA PAUSA LA VARIABLE USADA COMO TIMER DEBE REFRESCARSE CON EL VALOR ACTUAL DE millis()
      
   break;*/
    
}       //------cambia el valor de las variables negativas a positivas de reloj
        mmt=minutos-mmres;
        if(mmt<0){
          mmt=mmt+60;
          }
        sst=segundos-ssres;
        if(sst<0){
          mmt=mmt-1;
          sst=sst+60;
        }  
        au16data[27]=mmt;
        au16data[28]=sst;
  if((mmt==0)&&(sst==0)){
  terminar_ciclo=1;
  mmres=0;
  ssres=0;  
  }
        //----------------------------


if((minutosp!=minutos)||(segundosp=!segundos)){
minutosp=minutos;
segundosp=segundos;
mmres=0;
ssres=0; 
espera_plancha=0;   
}

if(estado==0){
  mmres=0;
  ssres=0;
  espera_plancha=0;
  }
        
if(estado==1){//---------------------------------------------------------------
if(espera_plancha<tiempo_espera){  
        if(millis()> reloj){ //incrementa cada segundo
        espera_plancha++;
         reloj = millis() + 1000;
        }}

if(espera_plancha>=tiempo_espera){
  espera_plancha=tiempo_espera+1;
if(mmres<minutos){
        if(millis()> reloj){ //incrementa cada segundo
        ssres++;
         reloj = millis() + 1000;
        }
        if(ssres>=60){
          ssres=0;
          mmres++;
        }
}else{ if(mmres==minutos){
          if(ssres<=segundos){
            if(millis()> reloj){ //incrementa cada segundo
            ssres++;
            reloj = millis() + 1000;
            }
            if(ssres>=60){
              ssres=0;
              mmres++;
             }         
          }else{
            ssres=0;
            mmres=0;
            }
        }
  
      }}
      }//-------------------------------------------------------------




if(millis()> tiempopid1){ //tiempo del duty
e1=tempe_z1-temperatura1;
kia1=(kia1*(e1+ep1))+kip1;
kda1=(kda1*(e1-ep1))-kdp1;
pid1=(kpa1*e1)+kia1+kda1;
YN=pid1;
incre=0;

ep1=e1;
kip1=kia1;
kdp1=kda1;
tiempopid1 = millis() + duty1;}




if(YN>=duty1){
  YN=duty1;}
if(YN<=0){
  YN=0;}
  


au16data[21]=YN;

//YNp=YN;

//tiempo_bajo1=duty1-YN;
//tiempo_alto1=YN;

// P2IM comments it out because it's comsuning too many input bytes from fuzzer
if(incre>=YN){
  //digitalWrite(SalidaRelay, LOW);
  incre++;
  //delay(1);
  }else{
  //digitalWrite(SalidaRelay, HIGH);
  incre++;
  //delay(1);
  }

in1=(~data8024[0]) & 0B00000001;
/*
if(in1==0B00000001){
  digitalWrite(CONTROLLINO_D1,HIGH);
  }else{digitalWrite(CONTROLLINO_D1,LOW);}
*/


}  
