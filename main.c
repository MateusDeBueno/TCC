//
// Included Files
//
#include "F28x_Project.h"

static float m1 = 350; //duty cycle
static float m2 = 350; //duty cycle
static float ireal[4]; //corrente do L1
static float Vneg[4]; //tensao de Co2
static float Vpos[4]; //tensao de Co1

static float ireal_med; //corrente com media movel para a proteção

static float Voreal[4]; //tensão de saida
static float Voreal_med; //tensão com media movel para a proteção
static float iref[3];
static float e_i[3]; //erro de corrente
static float e_v[3]; //erro de tensao
static float u[3]; //saida do duty cycle
static float Vref = 100; //tensão de referencia
static float GPWM = 500;
static float deltaV[2];
static float deltaV_med; //diferença de tensão com media movel para a proteção
static float deltaD[2];
static bool protDelta = false;
static bool protIL1 = false;
static bool protVo = false;
static bool bloqueia_chaves = false;
static int rotina;
static float debug = 0;


static float i_manual = 1; //debugar corrente

static float Voreal_prot[5]; //tensão de saida pra protecao
static float ireal_prot[5]; //corrente do L1
static float deltaV_prot[5]; //diferença de tensao pra protecao


// constantes do controlador de tensao total
//  d1*z + d2
//  --------
//   z - 1
float d1 = 0.002085;
float d2 = -0.0019968;

// constantes do controlador de tensao diferencial
// d1*z + d2
// --------
// z - 1
float f1 = -0.000387580679356681;
float f2 = 0.000384270164913587;
static int cont_vdif ; // contador da malha de tensao diferencial

//
// Globals
//
Uint32 EPwm1TimerIntCount;
Uint16 EPwm1_DB_Direction;
Uint32 ResultVdif = 0;
Uint32 ResultVo = 0;
Uint32 ResultIL1 = 0;
Uint32 ResultVpos = 0;
Uint32 ResultVneg = 0;

//
// Function Prototypes
//
//funções que vamos utilizar
void InitEPwm1Example(void);
void ConfigureADC(void);
void SetupADCSoftware(void);
inline void ADC_StartConversion(void);
inline void Control(void);
inline void Protecao(void);
void DAC_Initialize(void);
void DAC_WriteBoth(double voltage_a, double voltage_b);
void aquisicao(void);

__interrupt void epwm1_isr(void);

//
// Main
//
void main(void)
{
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
      InitSysCtrl();

//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to its default state.
//
//    InitGpio();

//
// enable PWM1, PWM2 and PWM3
//
    CpuSysRegs.PCLKCR2.bit.EPWM1=1;

//
// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
// These functions are in the F2837xD_EPwm.c file
//
    ConfigureADC();
    SetupADCSoftware();
    EALLOW;
    DAC_Initialize();
    GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;
    EDIS;


    InitEPwm1Gpio();


//
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;

//
// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
    InitPieCtrl();

//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;



//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
    InitPieVectTable();

//
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
//
    EALLOW; // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM1_INT = &epwm1_isr;
    //PieVectTable.EPWM2_INT = &epwm2_isr;



    EDIS;   // This is needed to disable write to EALLOW protected registers

//
// Step 4. Initialize the Device Peripherals:
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =0;
    EDIS;

    InitEPwm1Example();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =1;
    EDIS;

//
// Step 5. User specific code, enable interrupts:
// Initialize counters:
//
    EPwm1TimerIntCount = 0;

//
// Enable CPU INT3 which is connected to EPWM1-3 INT:
//
    IER |= M_INT3;

//
// Enable EPWM INTn in the PIE: Group 3 interrupt 1
//
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

//
// Enable global Interrupts and higher priority real-time debug events:
//
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


//
// Step 6. IDLE loop. Just sit and loop forever (optional):
//
    for(;;)
    {
        asm ("          NOP");
    }
}

//
// epwm1_isr - EPWM1 ISR
//
__interrupt void epwm1_isr(void)
{
    GpioDataRegs.GPBSET.bit.GPIO61 = 1; // Seta pino para verificação do tempo de cálculo no osciloscópio
    rotina = rotina + 1;
    //pega os valores do ADC
    ADC_StartConversion();

    aquisicao(); //salva os valores lidos nos adc em vetores, serve para melhor visualizacao dos valores reais lidos (verificar ruidos)

    Protecao();

    if (bloqueia_chaves == false){
        Control();
    }
    else{
        m1 = 0;
        m2 = 500;
    }

    EPwm1Regs.CMPA.bit.CMPA = m1;      //Vo2 aumenta - m1
    EPwm1Regs.CMPB.bit.CMPB = 500-m2;  //Vo1 aumenta - m2


    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    //isso serve para fechar a interrupção
    EPwm1Regs.ETCLR.bit.INT = 1; //permite que aconteca outras interrupções

}

//
// InitEPwm1Example - Initialize EPWM1 configuration
//
void InitEPwm1Example()
{
    EPwm1Regs.TBPRD = 500;                        // Set timer period
    EPwm1Regs.TBPHS.bit.TBPHS = 0;                // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                     // Clear counter

    //
    // Setup TBCLK
    //
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count up/down
    EPwm1Regs.TBCTL.bit.PHSEN = 0;                  // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;           //

    //
    // Setup compare
    //
    EPwm1Regs.CMPA.bit.CMPA = 0;                    //inicia com dutycicle = 0%
    EPwm1Regs.CMPB.bit.CMPB = 500;                  //
    //
    // Set actions
    //
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;            //
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;              //
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;            //ePWM1B deve ser 500-x, sendo x o valor de interesse
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;              //


    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;
    EPwm1Regs.DBRED.bit.DBRED = 0;
    EPwm1Regs.DBFED.bit.DBFED = 0;

    //
    // Interrupt where we will change the Deadband
    //
    //toda vez que passar pelo zero, ativara a interrupção
    EPwm1Regs.ETSEL.bit.INTSEL = 3;              // entra na interrupção quando a portadora for igual a zero
    EPwm1Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;          // Generate INT on 1rd event
}

void ConfigureADC(void)
{
    EALLOW;

    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 2; //set ADCCLK divider to /2
    AdcbRegs.ADCCTL2.bit.PRESCALE = 2; //set ADCCLK divider to /2
    AdccRegs.ADCCTL2.bit.PRESCALE = 2; //set ADCCLK divider to /2

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADCs
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}

void SetupADCSoftware(void)
{
    Uint16 acqps;

    //
    //determine minimum acquisition window (in SYSCLKS) based on resolution
    //
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }


    EALLOW;

    //ADC A
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 1;  //SOC0 will convert pin A1
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigar no epwm1a

    //ADC B
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 3;  //SOC0 will convert pin B3
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigar no epwm1a
    //ADC C
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert pin C2
    AdccRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigar no epwm1a

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    EDIS;
}

inline void ADC_StartConversion(void){
    //
    //convert, wait for completion, and store results
    //start conversions immediately via software, ADCA
    //
    AdcaRegs.ADCSOCFRC1.all = 0x0001; //SOC0

    //
    //start conversions immediately via software, ADCB
    //
    AdcbRegs.ADCSOCFRC1.all = 0x0001; //SOC0

    //
    //start conversions immediately via software, ADCB
    //
    AdccRegs.ADCSOCFRC1.all = 0x0001; //SOC0

    //
    //wait for ADCA to complete, then acknowledge flag
    //
    while(AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0);
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    //
    //store results
    //ResultIL1
    ResultIL1 = AdcaResultRegs.ADCRESULT0; //A
    ResultVpos = AdcbResultRegs.ADCRESULT0; //B
    //ResultVdif = AdcbResultRegs.ADCRESULT0; //B
    ResultVneg = AdccResultRegs.ADCRESULT0; //C
    //ResultVo = AdccResultRegs.ADCRESULT0; //C

    //ireal[0]=ResultIL1*0.0191 - 51.253;
    ireal[0]=ResultIL1*0.0136 + 0.03564;

    if (ireal[0]<0){
        ireal[0]=0;
    }
    ireal_prot[0]=ireal[0]; //sera utilizado na protecao

    Vneg[0]=ResultVneg*0.0683-9.3828;
    Vpos[0]=ResultVpos*0.0659-7.9169;

    Voreal[0]=Vpos[0]+Vneg[0];
    deltaV[0]=Vpos[0]-Vneg[0];

    deltaV_prot[0] = deltaV[0]; //sera utilizado na protecao
    Voreal_prot[0] = Voreal[0]; //sera utilizado na protecao


    //Voreal[0]=0.1461*ResultVo + 8.5996;
    //if (Voreal[0]<0){Voreal[0]=0;}
    //Voreal_prot[0]=Voreal[0]; //sera utilizado na protecao

    //deltaV[0] = 0.015*ResultVdif - 26.728;
    //deltaV[0] = -0.0149*ResultVdif + 29.979;
    //ero = 0.0149*ResultVdif - 31.247;
    //deltaV_prot[0]=deltaV[0]; //sera utilizado na protecao

    //escrever o valor numa saida DAC, somente para debugar
    //DAC_WriteBoth(1,0.5); //primeiro 30, segundo 70
}




inline void Control(void){
    // ============================================================== //
    // --------------     Calculo dos controladores    ---------------
    // ============================================================== //

    //==CONTROLADOR DE TENSÃO TOTAL==//

    //calculo do erro de tensao
    e_v[0] = Vref - Voreal[0];

    //calculo do contrador de tensão total
    u[0] = d1*e_v[0] + d2*e_v[1] + u[1];

    //saturação da razão cíclica
    if (u[0]<= 0){
        u[0] = 0;
    }
    if (u[0] >= .9){
        u[0] = .9;
    }

    //atualizacao das variaveis da malha de tensão total
    u[2] = u[1];
    u[1] = u[0];
    e_v[2] = e_v[1];
    e_v[1] = e_v[0];

    //==CONTROLADOR DA TENSAO DIFERENCIAL==//

    deltaV[0] = deltaV[0] - debug; //pra calibrar certinho em um ponto somente


    //calculo do controle da tensao diferencial
    //deltaD[0] = 1.9616*deltaD[1]-0.9616*deltaD[2]+0.001595*deltaV[1]-0.00159292*deltaV[2];

    cont_vdif +=1;
    if ( cont_vdif ==25){
        // calculo do erro de tensao diferencial
        deltaV[0] = deltaV[0] - debug;
        // calculo do controle da tensao diferencial
        deltaD [0] = f1*deltaV [0] + f2*deltaV [1] + deltaD [1];
        if ( deltaD [0] > 0.1){deltaD [0] = 0.1;}
        if ( deltaD [0] < -0.1){deltaD [0] = -0.1;}
        //atualizacao das variaveis
        deltaD[2] = deltaD[1];
        deltaD[1] = deltaD[0];
        deltaV[2] = deltaV[1];
        deltaV[1] = deltaV[0];
        cont_vdif =0;
    }


    m1 = (u[0] - deltaD[0]) * GPWM; //ganho do PWM, compensar internamente o ganho da triangular
    m2 = (u[0] + deltaD[0]) * GPWM; //ganho do PWM, compensar internamente o ganho da triangular

    //saturações da razão ciclica
    if (m1<= 0){
        m1 = 0;
    }
    if (m1 >= 450){
        m1 = 450;
    }
    if (m2<= 0){
        m2 = 0;
    }
    if (m2 >= 450){
        m2 = 450;
    }
}

inline void Protecao(void){
    // ============================================================== //
    // --------------         Proteção        ------------------------
    // ============================================================== //
    //atualização das variaveis da protecao
    deltaV_prot[4] = deltaV_prot[3];
    deltaV_prot[3] = deltaV_prot[2];
    deltaV_prot[2] = deltaV_prot[1];
    deltaV_prot[1] = deltaV_prot[0];

    Voreal_prot[4] = Voreal_prot[3];
    Voreal_prot[3] = Voreal_prot[2];
    Voreal_prot[2] = Voreal_prot[1];
    Voreal_prot[1] = Voreal_prot[0];

    ireal_prot[4] = ireal_prot[3];
    ireal_prot[3] = ireal_prot[2];
    ireal_prot[2] = ireal_prot[1];
    ireal_prot[1] = ireal_prot[0];

    //media movel para logica de protecao
    deltaV_med = (deltaV_prot[0]+deltaV_prot[1]+deltaV_prot[2]+deltaV_prot[3]+deltaV_prot[4])/5;
    Voreal_med = (Voreal_prot[0]+Voreal_prot[1]+Voreal_prot[2]+Voreal_prot[3]+Voreal_prot[4])/5;
    ireal_med = (ireal_prot[0]+ireal_prot[1]+ireal_prot[2]+ireal_prot[3]+ireal_prot[4])/5;

    //proteção de diferença na tensão dos capacitores
    if (deltaV_med > 60 || deltaV_med < -60){
        //protDelta = true;
    }

    //proteção da corrente de entrada muito alta
    if (ireal_med > 25){
        protIL1 = true;
    }

    //proteção de tensao muito alta na saida
    if (Voreal_med > 440){
        protVo = true;
    }

    //abre as chaves se der problema
    if (protDelta == true || protIL1 == true || protVo == true){
        EPwm1Regs.CMPB.bit.CMPB = 500;   //duty cycle = 0 quando é acionado a proteção
        EPwm1Regs.CMPA.bit.CMPA = 0;     //
        bloqueia_chaves = true;
    }
}


void DAC_Initialize(){
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; // Enable DACA
    DacaRegs.DACCTL.bit.DACREFSEL = 1;  // Reference is the positive rail supply (3V3)
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; // Enable DACB
    DacbRegs.DACCTL.bit.DACREFSEL = 1;  // Reference is the positive rail supply (3V3)
}

void DAC_WriteBoth(double voltage_a, double voltage_b){
    if(voltage_a > 3) voltage_a = 3;
        else if(voltage_a < 0) voltage_a = 0;

    if(voltage_b > 3) voltage_b = 3;
        else if(voltage_b < 0) voltage_b = 0;

    DacaRegs.DACVALS.bit.DACVALS = voltage_a*1365;
    DacbRegs.DACVALS.bit.DACVALS = voltage_b*1365;
}

void aquisicao()
{
    //serve para observar no gráfico como a variavel esta se comportando
    static int vResultIL1[500];
    static int contador = 0, contador2 = 0;
    contador++;
    if(contador>=1){
        if(contador2>=500){contador2=0;}
        vResultIL1[contador2] = ResultIL1;
        contador2++;
        contador=0;
    }
    static int vResultVpos[500];
    static int contador3 = 0, contador4 = 0;
    contador3++;
    if(contador3>=1){
        if(contador4>=500){contador4=0;}
        vResultVpos[contador4] = ResultVpos;
        contador4++;
        contador3=0;
    }
    static int vResultVneg[500];
    static int contadorA = 0, contadorB = 0;
    contadorA++;
    if(contadorA>=1){
        if(contadorB>=500){contadorB=0;}
        vResultVneg[contadorB] = ResultVneg;
        contadorB++;
        contadorA=0;
    }
}
//
// End of file
//
