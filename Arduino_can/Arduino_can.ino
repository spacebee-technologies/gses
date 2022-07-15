/*
 * Ejemplo CAN
*/
#include <SPI.h>
#include <mcp2515.h>
#include <stdint.h>

struct can_frame canMsg1; //Para enviar
struct can_frame canMsg;  //Para recibir

MCP2515 mcp2515(10);

//PARA CANOPEN
#define CANopen_nodeid 0x02
#define CAN_send_timeout     10                             //Tiempo de espera en ms para verificar el envio correcto de mensaje por can
#define Boot_up 1     //1 Si se quiere enviar mensaje Boot-up luego de pasar al estado pre-operacional  o 0 para desactivar

#define CANopen_SDO_mode_client 0
#define CANopen_SDO_mode_server 1
#define CANopen_SDO_timeout     10                         //Tiempo de espera en ms para recibir respuesta de servidor
#define CANopen_SDO_command_writing_request_4byte     0x23 //(rx) Solicitud de escritura de 4 bytes
#define CANopen_SDO_command_writing_request_2byte     0x2B //(rx) Solicitud de escritura de 2 bytes
#define CANopen_SDO_command_writing_request_1byte     0x2F //(rx) Solicitud de escritura de 1 byte
#define CANopen_SDO_command_write_Confirmation        0x60 //(tx) Confirmacion de escritura a cliente (maestro) desde servidor (esclavo)
#define CANopen_SDO_command_read_request              0x40 //(rx) Solicitud de lectura (upload) - solicitud de parametro a servidor (esclavo)
#define CANopen_SDO_command_response_parameter_4byte  0x43 //(tx) Respuesta envio de parametro a cliente (4 bytes)
#define CANopen_SDO_command_response_parameter_2byte  0x4B //(tx) Respuesta envio de parametro a cliente (2 bytes)
#define CANopen_SDO_command_response_parameter_1byte  0x4F //(tx) Respuesta envio de parametro a cliente (1 byte)
#define CANopen_SDO_command_transfer_aborted          0x80 //(tx) Transferencia de datos abortada, envio mensaje de error a cliente (No sopoerado en esta libreria)

typedef enum
    {
        CANopen_INITIALIZATION,             //Estado de inicializacion, luego de configurarse se pasa al siguiente estado
        CANopen_PRE_OPERATIONAL,            //Estado pre operacional (solo responde a comandos NMT, SDO, SYNC, Time stamp o Hearbeat)
        CANopen_OPERATIONAL,                //Estado operacional (se activa el soporte de PDOs ademas de los comandos anteriores)
        CANopen_STOPPED,                    //Estado parado, deja inactivos todos los objetos de comunicación posibles (no se pueden enviar ni recibir PDOs ni SDOs), excepto a los comandos NMT que pueden cambiar a cualquiera de los estados restantes y Heartbeat. Es decir, un nodo solo puede realizar vigilancia de nodos o latidos, pero no puede recibir ni transmitir mensajes.
    } CANopen_STATES;                       //Enumaracion de los estados posibles de la maquina de estado CANopen

volatile static CANopen_STATES state = CANopen_INITIALIZATION; //Variable para guardar el estado de la aplicaciÃ³n
    

typedef struct diccionario32 {                        //Diccionario para datos de 4 bytes
        uint16_t Index;
        uint8_t  Subindex;
        uint8_t  Attribute;
        uint32_t Data;
        } diccionario32;

typedef struct diccionario16 {                        //Diccionario para datos de 2 bytes
        uint16_t Index;
        uint8_t  Subindex;
        uint8_t  Attribute;
        uint16_t Data;
        } diccionario16;

typedef struct diccionario8 {                        //Diccionario para datos de 1 byte
            uint16_t Index;
            uint8_t  Subindex;
            uint8_t  Attribute;
            uint8_t Data;
            } diccionario8;  
        
diccionario32 dictionary32[]={
    //Index     Subindex     Attribute        Data
    { 0x00,         0,           0,          0x0000},   //Posicion actual servor
    { 0x00,         1,           0,          0x0000},   
    { 0X01,         0,           0,          0x0000}    //Setpoin posicion servo
};      

diccionario16 dictionary16[]={
    //Index     Subindex     Attribute       Data
    { 0x00,         0,           0,          0x00},
    { 0x00,         1,           0,          0x00},
    { 0X01,         0,           0,          0x00}
};  

diccionario8 dictionary8[]={
    //Index     Subindex     Attribute      Data
    { 0x00,         0,           0,          0},
    { 0x00,         1,           0,          0},
    { 0X01,         0,           0,          0}
};  

/*========================================================================
  Funcion: CANopen_BootUp
  Descripcion: Envia mensaje de inicio BootUp por MCAN
  Sin parametro de entrada
  Retorna:   true  = si se envio correctamente
             false = si no se pudo enviar
  ========================================================================*/
bool CANopen_BootUp(void){
    //Manda los datos
    canMsg1.can_id  = 0x700 + CANopen_nodeid;
    for(int i=0; i<8; i++){
      canMsg1.data[i]=0;
      }
    mcp2515.sendMessage(&canMsg1);
    return true;
}

/*========================================================================
  Funcion: CANopen_STOP
  Descripcion: Pasa al estado stop de la maquina de estados de CANopen
  Sin parametro de entrada
  No retorna nada
  ========================================================================*/
void CANopen_STOP(void){
   state = CANopen_STOPPED;
}

/*========================================================================
  Funcion: CANopen_Write_Dictionary
  Descripcion: Escribe dato en el diccionario de objetos
  Parametro de entrada:  uint16_t index     = indice donde se desea escribir
                         uint8_t subindex   = subindice donde se desea escribir
                         uint32_t data      = dato a escribir
                         uint8_t dictionary = diccionario al cual se desea escribir (32, 16, 8)
  Retorna:               True si se puedo escribir
                         False si no se puedo escribir
  Importante:            Esta funcion implementa seccion critica
  ========================================================================*/
bool CANopen_Write_Dictionary(uint16_t index, uint8_t subindex, uint32_t data, uint8_t dictionary){
    bool retorno = false;
    if(dictionary==32){
        uint16_t  tamanio_dic=sizeof(dictionary32)/8;       //Obtengo numeros de elementos del diccionario
        for(uint16_t i=0; i<tamanio_dic;i++){               //Recorro el diccionario
                                                            //Busco index y subindex indicados
            if(dictionary32[i].Index==index && dictionary32[i].Subindex==subindex){
                if(dictionary32[i].Attribute==0){           //Si el atributo permite escritura
                    dictionary32[i].Data=data;              //Escribo dato en diccionario
                    retorno=true;
                }
            }
        }
    }
    
    if(dictionary==16){
        uint16_t  tamanio_dic=sizeof(dictionary16)/6;       //Obtengo numeros de elementos del diccionario
        for(uint16_t i=0; i<tamanio_dic;i++){               //Recorro el diccionario
                                                            //Busco index y subindex indicados
            if(dictionary16[i].Index==index && dictionary16[i].Subindex==subindex){
                if(dictionary16[i].Attribute==0){           //Si el atributo permite escritura
                    dictionary16[i].Data=(uint16_t)data;    //Escribo dato en diccionario
                    retorno=true;
                }
            }
        }
    }
    if(dictionary==8){
        uint16_t  tamanio_dic=sizeof(dictionary8)/5;        //Obtengo numeros de elementos del diccionario
        for(uint16_t i=0; i<tamanio_dic;i++){               //Recorro el diccionario
                                                            //Busco index y subindex indicados
            if(dictionary8[i].Index==index && dictionary8[i].Subindex==subindex){
                if(dictionary8[i].Attribute==0){            //Si el atributo permite escritura
                    dictionary8[i].Data=(uint8_t)data;      //Escribo dato en diccionario
                    retorno=true;
                }
            }
        }
    }
    return retorno;
}

/*========================================================================
  Funcion: CANopen_Read_Dictionary
  Descripcion: Lee dato en el diccionario de objetos
  Parametro de entrada:  uint16_t index     = indice donde se desea leer
                         uint8_t subindex   = subindice donde se desea leer
                         uint32_t* data     = puntero a la variable donde se quiere guardar al dato
                         uint8_t dictionary = diccionario al cual se desea escribir (32, 16, 8)
  Retorna:               True si se puedo leer
                         False si no se puedo leer
  Importante:            Esta funcion implementa seccion critica
  ========================================================================*/
bool CANopen_Read_Dictionary(uint16_t index, uint8_t subindex, uint32_t *data, uint8_t dictionary){
    bool retorno = false;
    if(dictionary==32){
        uint16_t  tamanio_dic=sizeof(dictionary32)/8;       //Obtengo numeros de elementos del diccionario
        for(uint16_t i=0; i<tamanio_dic;i++){               //Recorro el diccionario
                                                            //Busco index y subindex indicados
            if(dictionary32[i].Index==index && dictionary32[i].Subindex==subindex){
                if(dictionary32[i].Attribute==0){           //Si el atributo permite leer
                    *data=dictionary32[i].Data;             //Escribo dato en diccionario
                    retorno=true;
                }
            }
        }
    }
    
    if(dictionary==16){
        uint16_t  tamanio_dic=sizeof(dictionary16)/6;       //Obtengo numeros de elementos del diccionario
        for(uint16_t i=0; i<tamanio_dic;i++){               //Recorro el diccionario
                                                            //Busco index y subindex indicados
            if(dictionary16[i].Index==index && dictionary16[i].Subindex==subindex){
                if(dictionary16[i].Attribute==0){           //Si el atributo permite leer
                    *data=(uint32_t)dictionary16[i].Data;   //Escribo dato en diccionario
                    retorno=true;
                }
            }
        }
    }
    
    if(dictionary==8){
        uint16_t  tamanio_dic=sizeof(dictionary8)/5;        //Obtengo numeros de elementos del diccionario
        for(uint16_t i=0; i<tamanio_dic;i++){               //Recorro el diccionario
                                                            //Busco index y subindex indicados
            if(dictionary8[i].Index==index && dictionary8[i].Subindex==subindex){
                if(dictionary8[i].Attribute==0){            //Si el atributo permite leer
                    *data=(uint32_t)dictionary8[i].Data;    //Escribo dato en diccionario
                    retorno=true;
                }
            }
        }
    }
    return retorno;
}

/*========================================================================
  Funcion: CANopen_init
  Descripcion: Establece la configuracion del periferico can y cambia el estado de la maquina de estado CANopen
  Sin parametro de entrada
  Rertorna: uint8_t   0 = resultado ok
                      1 = Error al mandar mensaje Boot_Up
  ========================================================================*/
uint8_t CANopen_init(void){
    state = CANopen_PRE_OPERATIONAL;                                //Cambio estado de la maquina de estado de CANopen
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS);
    mcp2515.setNormalMode();
    if(Boot_up == 1){                                       //Si esta activo el Boot_up
        if(CANopen_BootUp()==false){ return 1; }            //Envio mensaje de inicio y verifico errores
    }
    return 0;                                               //Retorno OK
}

/*========================================================================
  Funcion: CANopen_SDO_Expedited_Write
  Descripcion: Envia un mensaje SDO
  Parametro de entrada:     uint8_t node_id  = id del nodo con el cual se desea comunicar si esta en modo cliente o el propio id del dispositivo si se esta en modo servidor
                            uint8_t command  = Describe la operacion del comando SDO (lectura/escritura), puede ser:
                                                0x22 = (rx) Solicitud de escritura > 4 bytes (no soportado en esta libreria)
                                                0x23 = (rx) Solicitud de escritura de 4 bytes
                                                0x2B = (rx) Solicitud de escritura de 2 bytes
                                                0x2F = (rx) Solicitud de escritura de 1 byte
                                                0x60 = (tx) Confirmacion de escritura a cliente (maestro) desde servidor (esclavo)
                                                0x40 = (rx) Solicitud de lectura (upload) - solicitud de parametro a servidor (esclavo)
                                                0x43 = (tx) Respuesta envio de parametro a cliente (4 bytes)
                                                0x4B = (tx) Respuesta envio de parametro a cliente (2 bytes)
                                                0x4F = (tx) Respuesta envio de parametro a cliente (1 byte)
                                                0x80 = (tx) Transferencia de datos abortada, envio mensaje de error a cliente (No sopoerado en esta libreria)
                            uint16_t index   = Indice del diccionario al que se desea acceder (2 bytes)
                            uint8_t subindex = Subindice deldiccionario al que se desea acceder (1 byte)
                            uint8_t data     = Dato a escribir en el diccionario, si es peticion de informacion data tiene que contener todos ceros (4 bytes) y luego se pasan los datos recibidos a esta variable
                            uint8_t mode     = Modo cliente (maestro) = 0  Modo servidor(esclavo) = 1
  Retorna:                  0  = si se envio y recibio correctamente
                            2  = si no se obtuvo respuesta del servidor (solo funciona con mensajes SDO en modo cliente)
                            3  = si la respuesta del servidor no corresponde con el mensaje enviado (solo funciona con mensajes SDO en modo cliente)
                            4  = si la maquina de estados no paso la inicializacion
  Funcionamiento: Se debe llamar cuando se quiere mandar un mensaje SDO CANopen por MCAN
                  Funcionamiento interno cada vez que se llamaa esta funcion:
                  Genera el header a enviar por MCAN con la forma de un mensaje SDO y lo envia por MCAN.
                  Si esta en modo cliente, espera a recibir un mensaje can con la confirmacion de escritura sobre diccionario o una recepcion de dato (segun el comando seleccionado).
                  Si esta en modo servidor(esclavo), finaliza la funcion sin esperar confirmacion, es decir solo envia el dato.
========================================================================*/
uint8_t CANopen_SDO_Expedited_Write(uint8_t node_id, uint8_t command, uint16_t index, uint8_t subindex,  uint8_t *data, uint8_t mode){
    if(state==CANopen_PRE_OPERATIONAL || state==CANopen_OPERATIONAL){         //Si el estado del CANopen permite mensajes SDO
        static uint8_t message[8] = {0}; 
        message[0]=command;                                                  //Byte 0 (comando SDO)
        message[1]=index & 0xFF;                                             //Byte 1 Index LSB (byte menos significativo)
        message[2]=index >> 8;                                               //Byte 2 Index MSB (byte mas significativo)
        message[3]=subindex;                                                 //Byte 3 Subindex
        message[4]=data[0];                                                  //Byte 4 = byte data 0
        message[5]=data[1];                                                  //Byte 4 = byte data 1
        message[6]=data[2];                                                  //Byte 4 = byte data 2
        message[7]=data[3];                                                  //Byte 4 = byte data 3
        uint32_t id =0x000;

        if(mode==CANopen_SDO_mode_client){
           id = 0x600 + node_id;
        }else{
           id = 0x580 + node_id;
        }
        
        //Envio mensaje por can
		canMsg1.can_dlc = 8;
        for(int i=0; i<8; i++){
          canMsg1.data[i]=message[i];
          }
        mcp2515.sendMessage(&canMsg1);
        
        //Si esta en modo cliente, espero respuesta desde el servidor
        if(mode==CANopen_SDO_mode_client){
          uint8_t interacciones=0;
          uint8_t retornar=9;                                                //Inicializo en un valor no utilizado
          //Bucle infinito con timeout para verificar la recepcion en periferico can  
          while(true){
            if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {         //Si recibi mensaje por can
                //Verifico que el mensaje recibido sea el correcto
                if(canMsg1.can_id!=0x580 + node_id){                         //Si la respuesta no es del servidor o no es un mensaje sdo tx 
                    retornar = 3;
                    break;                                                   //Salgo del while
                }
                if(canMsg1.data[0]==0x60||canMsg1.data[0]==0x43||canMsg1.data[0]==0x4B||canMsg1.data[0]==0x4F){ //Si la respuesta es un comando de respuesta
                    if(canMsg1.data[0]==0x60){                               //Si el mensaje es de confirmacion de escritura
                        retornar = 0;
                        break;                                               //Salgo del while
                    }else{                                                   //Sino verifico que el diccionario de respuesta corresponda con el enviado
                        if((canMsg1.data[1]==(index & 0xFF)) && (canMsg1.data[2]==(index >> 8))){
                            //paso el dato recibido a la variable apuntada en la funcion
                            data[0]=canMsg1.data[4];
                            data[1]=canMsg1.data[5];
                            data[2]=canMsg1.data[6];
                            data[3]=canMsg1.data[7];
                            retornar = 0;
                            break;                                          //Salgo del while
                        }else{
                            retornar = 3;
                            break;                                          //Salgo del while
                        }
                    }
                }else{                                                      //Si la respuesta no es un comando de respuesta
                    retornar = 3;
                    break;                                                  //Salgo del while
                }
            }else{
                interacciones++;
              }
              
            if(interacciones*1>=CANopen_SDO_timeout){
                    retornar = 2;
                    break;                                                  //Salgo del while true
                }
            delay(1);
           }
         return retornar;                                                   //Retorno desde la funcion
        }
        return 0;                                                           //Retorno desde la funcion
    }else{
        return 4;                                                           //Retorno desde la funcion
    }
}


/*========================================================================
  Funcion: CANopen_SDO_Expedited_Read
  Descripcion: Recibe un mensaje can y comprueba que sea un mensaje SDO
  Parametro de entrada:     
  Retorna:                  0 = Mensaje SDO recibido y es escritura sobre diccionario
                            3 = Si el mensaje recibido no es SDO Rx
                            4 = Mensaje SDO recibido y parametros enviados correctamente
                            5 = Mensaje SDO recibido pero no es para este dispositivo (id diferente)
                            6 = Mensaje SDO recibido y parametros enviados incorrectamente
                            7 = Maquina de estado no esta en preoperation o superior
  Funcionamiento: Se debe llamar continuamente a esta funcion desde alguna tarea u funcion para verificar constantemente la llega de un nuevo mensaje CANopen
                  Funcionamiento interno cada vez que se llamaa esta funcion:
                  En caso de recibir correctamente un mensaje SDO CANopen, se anliza si es de escritura o lectura sobre el diccionario ya sea dato de 32 bits.
                  Si es de escritura, guarda el dato recibido en el diccionario correspondiente en el index y subindex indicado en el mensaje
                  Si es de lectura, se lee el valor del diccionario correspondiente en el index y subindex indicado en el mensaje CANopen y luego se envia por CANopen mediante mensaje SDO con la funcion CANopen_SDO_Expedited_Write()
  ========================================================================*/
uint8_t CANopen_SDO_Expedited_Read(uint16_t *index, uint8_t *subindex){
  
    uint8_t retornar=7;                                             //Inicializo en un valor no utilizado
    if(state==CANopen_PRE_OPERATIONAL || state==CANopen_OPERATIONAL){
      if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {        //Si recibi mensaje por can
        //Verifico que el mensaje recibido sea el correcto
        if(canMsg.can_id>=0x601 && canMsg.can_id<=0x67F){             //Si el mensaje tiene cobid SDO Rx
            if(canMsg.can_id==(0x600+CANopen_nodeid)){                //El mensaje esta dedicado a este nodo
                
                if(canMsg.data[0]==0x40){                             //Si el comando SDO es lectura
                    *index=(canMsg.data[2]<<8)+canMsg.data[1];         //Paso index
                    *subindex=canMsg.data[3];                         //Paso subindex
                    retornar = 4;                                     //Indico que se retorna Mensaje SDO recibido y es necesario enviar parametros del diccionario
                }
  
                if(canMsg.data[0]==0x43){                            //Si el comando SDO es escritura de 4 bytes
                    *index=(canMsg.data[2]<<8)+canMsg.data[1];        //Paso index
                    *subindex=canMsg.data[3];                         //Paso subindex
                    uint32_t data=(canMsg.data[7]<<24)+(canMsg.data[6]<<16)+(canMsg.data[5]<<8)+canMsg.data[4];
                    CANopen_Write_Dictionary(*index, *subindex, data, 32); //Escribo diccionario de objetos
                    Serial.print(canMsg.data[4], HEX);
                    Serial.print(" ");
                    Serial.print(canMsg.data[5], HEX);
                    Serial.print(" ");
                    Serial.print(canMsg.data[6], HEX);
                    Serial.print(" ");
                    Serial.println(canMsg.data[7], HEX);
                    retornar = 0;                                     //Indico que se retorna ok
                }
                
                if(canMsg.data[0]==0x4B){                             //Si el comando SDO es escritura de 2 bytes
                    *index=(canMsg.data[2]<<8)+canMsg.data[1];        //Paso index
                    *subindex=canMsg.data[3];                         //Paso subindex
                    uint16_t data=(canMsg.data[5]<<8)+canMsg.data[4];
                    CANopen_Write_Dictionary(*index, *subindex, data, 16);//Escribo diccionario de objetos
                    retornar = 0;                                     //Indico que se retorna ok
                }
    
                if(canMsg.data[0]==0x4F){                             //Si el comando SDO es escritura de 1 byte
                    *index=(canMsg.data[2]<<8)+canMsg.data[1];        //Paso index
                    *subindex=canMsg.data[3];                         //Paso subindex
                    uint8_t data=canMsg.data[4];
                    CANopen_Write_Dictionary(*index, *subindex, data, 16); //Escribo diccionario de objetos
                    retornar = 0;                                     //Indico que se retorna ok
                }
            }else{
                retornar = 5;                                         //Indico que se retorna que el mensaje sdo es para otro nodo id
            }
        }else{
            retornar = 3;                                             //Indico que se retorna que no es mensaje con cobid SDO Rx
        }
  
        if(retornar==0){                                              //Si el comando recibido era de escritura y se escribio con exito, Envio mensaje de confirmacion
          uint8_t data_ff[3]={0};
          CANopen_SDO_Expedited_Write(CANopen_nodeid, 0x60, *index, *subindex,  data_ff, CANopen_SDO_mode_server);
        }
  
        if(retornar==4){                                               //Si el comando recibido era de lectura, envio dato desde el diccionario
          uint32_t dataa;                                              //Variable para guardar el dato del diccionario
          
          if(canMsg.data[0]==0x23){
              if(CANopen_Read_Dictionary(*index,*subindex, &dataa, 32)==true){ //Si existe el dato en el diccionario
                  uint8_t data_byte[3]={0};  
                  //En el diccionario se guarda el dato siendo primero el byte mas siginificativo. Para transmitir por CANopen es necesario transmitir primero el byte menos significativo (por eso se espeja el dato)
                  data_byte[0]=(uint8_t)(dataa & 0x0F);
                  data_byte[1]=(uint8_t)((dataa>>8) & 0x0F);
                  data_byte[2]=(uint8_t)((dataa>>16) & 0x0F);
                  data_byte[3]=(uint8_t)((dataa>>24) & 0x0F);
                  if(CANopen_SDO_Expedited_Write(CANopen_nodeid, 0x60, *index, *subindex,  data_byte, CANopen_SDO_mode_server)!=0){   //Si no se envio correctamente el dato
                      retornar=6;
                  }
              }else{
                  retornar=6;
              }
          } 
  
          if(canMsg.data[0]==0x2B){
              if(CANopen_Read_Dictionary(*index,*subindex, &dataa, 16)==true){ //Si existe el dato en el diccionario
                  uint8_t data_byte[3]={0};  
                  //En el diccionario se guarda el dato siendo primero el byte mas siginificativo. Para transmitir por CANopen es necesario transmitir primero el byte menos significativo (por eso se espeja el dato)
                  data_byte[0]=(uint8_t)(dataa & 0x0F);
                  data_byte[1]=(uint8_t)((dataa>>8) & 0x0F);
                  if(CANopen_SDO_Expedited_Write(CANopen_nodeid, 0x60, *index, *subindex,  data_byte, CANopen_SDO_mode_server)!=0){   //Si no se envio correctamente el dato
                      retornar=6;
                  }
              }else{
                  retornar=6;
              }
          }   
  
          if(canMsg.data[0]==0x2F){
              if(CANopen_Read_Dictionary(*index,*subindex, &dataa, 8)==true){  //Si existe el dato en el diccionario
                  uint8_t data_byte[3]={0};  
                  //En el diccionario se guarda el dato siendo primero el byte mas siginificativo. Para transmitir por CANopen es necesario transmitir primero el byte menos significativo (por eso se espeja el dato)
                  data_byte[0]=(uint8_t)(dataa & 0x0F);
                  if(CANopen_SDO_Expedited_Write(CANopen_nodeid, 0x60, *index, *subindex,  data_byte, CANopen_SDO_mode_server)!=0){   //Si no se envio correctamente el dato
                      retornar=6;
                  }
              }else{
                  retornar=6;
              }
          }                           
          
      }
      
      return retornar;                                                //Retorno desde la funcion
    }
  } 
  return retornar;                                                //Retorno desde la funcion
}

//Fin CANOPEN

void setup() {
  canMsg1.can_id  = 0x01;
  canMsg1.can_dlc = 1;
  canMsg1.data[0] = 0x00;

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();

  //Manda los datos
  canMsg1.data[0] = 0x01; canMsg1.data[1] = 0x10; canMsg1.data[2] = 0x21; canMsg1.data[3] = 0x00;
  mcp2515.sendMessage(&canMsg1);

  /*
  //Inicializo CANopen
  uint8_t resultado=CANopen_init();
  if (resultado == 0){ Uart1_println("CANopen was initialized and is in pre-operational mode"); }
  if (resultado == 1){ Uart1_println("Error al mandar mensaje Boot_Up");  CANopen_STOP(); }
   */
}

void loop() {
 //Leo dato CAN
 if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print(canMsg.data[0], HEX);
    Serial.print(" ");
    Serial.print(canMsg.data[1], HEX);
    Serial.print(" ");
    Serial.print(canMsg.data[2], HEX);
    Serial.print(" ");
    Serial.print(canMsg.data[3], HEX);
    Serial.print(" ");
    Serial.println(" ");
  }

  /* 
  //CANopen
  //verifico si hay mensaje SDO CANopen
  uint16_t index=0x0000;
  uint8_t  subindex=0x00;
  uint8_t res = CANopen_SDO_Expedited_Read(uint16_t *index, uint8_t *subindex);

  if(res==0){
      Serial.println("Mensaje SDO recibido yse escribio dato sobre diccionario");
  }else{
      if(res==4){
        Serial.println("Mensaje SDO recibido y dato de diccioanrio enviado correctamente");
      }else{
          Serial.println("Error");
      }
  }

  //Obtengo valor del diccionario
  int8_t data[3]={0};
  uint16_t index2=0x0001;
  uint8_t subindex2=0x00;
  bool res = CANopen_Read_Dictionary(index2, subindex2, data, 32);
  
  if(res == true){
      //Serial.println("Lectura correcta");
      //El dato leido se almacena en data[]
  }else{
      //Serial.println("Error al leer diccionario");
  }
  */

}
