#include <SPI.h> //Para usar a bibliotecade comunicação SPI
#include <mcp2515.h> //Para usar a biblioteca de comunicação CAN
MCP2515 mcp2515(10); //Pino de entrada que será o CS (Chip Select) da comunicação SPI entre Arduino e o Controlador CAN

const int pinoAcelerador = 7; // acelerador pino 7
const int pinoFreio = 6; // freio pino 6
const int pinoVolante = 9; // volante pino 9
const int pinoFeedBack = 8; // feedback pino 8

struct can_frame canMsg; //Declara uma estrutura de dados com base numa estrutura predefinida na biblioteca do Controlador CAN

const int rodaDianteira = 0x02; // id arbitrario para roda dianteira
const int rodaTraseira = 0x03; // id arbitrario para roda traseira
const int controladoraIMUTraseira = 0x04 // id arbitrario para controladora IMU
const int controladoraIMUDianteira = 0x05 // id arbitrario para controladora IMU
const int controleDirecao = 0x01 // id arbitrario para essa propria controladora
//-----------------------------------------------StabilityController-------------------------------------------

    // state, limite de ajuste, realiza no Motor de feedback uma força contraia AjustfeedbackControl()
    class StabilityController(//controle de estabilidade
      public:
        float torqueRodaDianteiraEsquerda;
        float torqueRodaDianteiraDireita;
        
        float torqueRodaTraseiraDireta;
        float torqueRodaTraseiraEsquerda;
    
        float anguloRodaDianteiraDireita;
        float anguloRodaDianteiraEsquerda;
        
        float ajustesTorqueFrontal;
        float ajustesTorqueTraseiro;

        float ajusteVolante;
    
        float estadoTraseiro;
        float estadoDianteiro;

        float estadoAtualComAjustesTorqueFrontal() // utiliza a variavel estado, e faz ajustes
        float estadoAtualComAjustesTorqueTraseiro() // utiliza a variavel estado, e faz ajustes
       
        AjustfeedbackControl(); // Atua no Motor de feedback com base no estado para aumentar estabilidade

      private:
       float limitesDeAjustes; 
      );
    
    void StabilityController::AjustfeedbackControl(){
      // AnguloRodaDianteiraDireita
      // AnguloRodaDianteiraEsquerda
      // Realiza calculos com as os Angulos das rodas para gerar a força de feedback
      // Possibilitando quem está condozindo o volate a perceber o ambiente em que as rodas estão
      // A força que o Volante exercerá será constante, não aumentando com relação ao angulo final do feedback
      ajusteVolante = anguloRodaDianteiraDireita - anguloRodaDianteiraEsquerda - limitesDeAjustes; 
      analogWrite( pinoFeedBack, ajustes ); // ajuste de feedback
    }
    
    float StabilityController::estadoAtualComAjustesTorqueFrontal(){
      // A partir do torque real Frontal recebido pela IMU, o sistema realiza calculos de ajuste
      // No torque que será transmitido de volta para as rodas frontais
      return  torqueRodaDianteiraEsquerda - torqueRodaDianteiraDireita; 
    }
    float StabilityController::estadoAtualComAjustesTorqueTraseiro(){
      // A partir do torque real Traeseiro recebido pela IMU, o sistema realiza calculos de ajuste
      // No torque que será transmitido de volta para as rodas Traseiras
       return estadoTraseiro - limitesDeAjustes;
    }
    float StabilityController::estadoAtualComAjustesAngulo(){
      // A partir do Angulo retornado pela IMU das rodas dianteiras
      // O sistema realiza ajustes no angulo enviado de volta para as rodas Dianteiras
      // Para garantir a estabilidade do angulo das rodas 
       return anguloRodaDianteiraDireita - anguloRodaDianteiraEsquerda - limitesDeAjustes; 
    }
        
//-----------------------------------------------SteeringController-------------------------------------------
    
    // recebe de StabilityController o limete de ajuste para ajustar os valores que serão enviados
    // tera 2 funções, sendRodaDianteira e sendRodaTraseira, formado pelo calculo das leituras dos sensores com o valor do ajuste
    class SteeringController(//controle de direção
    
      public:
        float acelerador;
        float freio;
        float volante;
        float estadoAjustadoTorqueDianteiro;
        float estadoAjustadoTorqueTraseiro;
        float estadoAjustadoAngulo;
        
        float sendRodaDianteiraTorque();
        float sendRodaDianteiraAngulo();
        float sendRodaTraseiraTorque();

       private:
        int distanciaRodaDianteira;
        int distanciaRodaTraseira;
        int pesoDoCarro;
      );  
      
      //precisa devolver o angulo e o torque para as rodas Dianteiras
      float SteeringController::sendRodaDianteiraTorque(){
        //antes precisa tratar as variaveis 
        return ((aceleracador - freio)*pesoDoCarro*distanciaRodaDianteira) - estadoAjustadoTorqueFrontal;
      }

      //precisa devolver o angulo e o torque para as rodas Traseiras
      float SteeringController::sendRodaTraseiraTorque(){
        return ((aceleracador - freio)*pesoDoCarro*distanciaRodaTraseira) - estadoAjustadoTorqueTraseiro;
      }
      
      float SteeringController::sendRodaAngulo(){
        return volante - estadoAjustadoAngulo;
      }

      
      
//-----------------------------------------------------------------------------------------------------------------
SteeringController *steeringController;
StabilityController *stabilityController;
//-----------------------------------------------------------------------------------------------------------------


//aqui vai o id e a mensagem, precisa enviar uma mensagem para rodaDianteira e rodaTraseira
void sendCam(int id, float torqueFrontal, float torqueTraseiro, int angulo ){ 
  
  // Se houver dado disponivel na Serial para ser lido
  while (Serial.available() > 0)
  {   
    canMsg.can_id  = id;                //id arbitrario do dispositivo
    canMsg.can_dlc = 6;                 //CAN data length = 1 (pode ser no máximo 8 bytes)
    canMsg.data[0] = torqueFrontal;             
    canMsg.data[1] = torqueFrontal;             
    canMsg.data[2] = torqueTraseiro;               
    canMsg.data[3] = torqueTraseiro;               
    canMsg.data[4] = angulo;            
    canMsg.data[5] = angulo;            
    canMsg.data[6] = 0x00;              //CAN data6 = 0
    canMsg.data[7] = 0x00;              //CAN data7 = 0
    //Enviar a Mensagem CAN - Transceiver colocará essa mensagem no barramento CAN
    mcp2515.sendMessage(&canMsg);
  }
}

// Recebe o ID do modulo que a mensagem será enviada
// Precisa ler dados da IMU
byte readCamDianteira(){
  
  // Lê o dado de um dispositivo especifico mensagem específica
   
    if (canMsg.can_id == controladoraIMUDianteira) // aqui podemos setar diferentes ids para ler mensagens
    {          
      
      canMsg.can_id  = id;                 //id arbitrario do dispositivo
      canMsg.can_dlc = 6;                  //CAN data length = 1 (pode ser no máximo 8 bytes)
      stabilityController->torqueRodaDianteiraDireta  = canMsg.data[1]*256+canMsg.data[0];
      stabilityController->torqueRodaDianteiraEsquerda = canMsg.data[3]*256+canMsg.data[2];
      stabilityController->anguloRodaDianteiraDireita = canMsg.data[5]*256+canMsg.data[4];
      stabilityController->anguloRodaDianteiraEsquerda = canMsg.data[7]*256+canMsg.data[6];
                   
    }
  
  
}

byte readCamTraseira(){
  
  // Lê o dado de um dispositivo especifico mensagem específica
   
    if (canMsg.can_id == controladoraIMUTraseira) // aqui podemos setar diferentes ids para ler mensagens
    {          
      
      canMsg.can_id  = id;                 //id arbitrario do dispositivo
      canMsg.can_dlc = 6;                  //CAN data length = 1 (pode ser no máximo 8 bytes)
      stabilityController->torqueRodaTraseiraDireta = canMsg.data[1]*256+canMsg.data[0];
      stabilityController->torqueRodaTraseiraEsquerda = canMsg.data[3]*256+canMsg.data[2];
                   
    }
  
  
}

void setup() {
  
  pinMode(pinoAcelerador, INPUT);  // Define pinoAcelerador como entrada
  pinMode(pinoFreio,      INPUT);  //Define pinoFreio como entrada
  pinMode(pinoVolante,    INPUT);  //Define pinoVolante como entrada
  pinMode(pinoFeedBack,   OUTPUT); //Define pinoFeedBack  como saida
  
  //Inicia a comunicação Serial
  Serial.begin(9600);
  //Inicia a comunicação SPI
  SPI.begin();
  //Reset do Controlador CAN atraves da SPI do Arduino
  mcp2515.reset();
  //Configura a velocidade de comunicação CAN para 500KBPS com um Clock de 8MHz. Clock do cristal do Controlador MCP2515
  mcp2515.setBitrate(CAN_500KBPS,MCP_8MHZ);
  //Modos de Operação do Controlador CAN: 1. Configuration mode 2. Normal mode 3. Sleep mode 4. Listen-Only mode 5. Loopback mode
  //Configura o modo normal
  mcp2515.setNormalMode();

}

void loop() {


  steeringController-> acelerador = analogRead(pinoAcelerador);
  steeringController-> freio      = analogRead(pinoFreio)
  steeringController-> volante    = analogRead(pinoVolante);

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK){
    readCamTraseira(controladoraIMUTraseira);
    readCamDianteira(controladoraIMUDianteira);
    stabilityController-> AjustfeedbackControl();
    steeringController-> estadoAjustadoTorqueDianteiro = stabilityController-> estadoAtualComAjustesTorqueFrontal() ;
    steeringController-> estadoAjustadoTorqueTraseiro = stabilityController-> estadoAtualComAjustesTorqueTraseiro() ;
    steeringController-> estadoAjustadoAngulo = stabilityController-> estadoAtualComAjustesAngulo() ;
  }
  
  sendCam(controleDirecao, 
          steeringController->sendRodaDianteiraTorque(), 
          steeringController->sendRodaTraseiraTorque(), 
          steeringController->sendRodaAngulo()
  );
  
  
}
