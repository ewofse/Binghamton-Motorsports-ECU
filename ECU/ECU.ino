FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;

CAN_message_t msgRX;
CAN_message_t msgTX;

bool isAlreadyReady = false;

int total1, total2 = 0;
int average1, average2 = 0;  
int counter1, counter2 = 0;
int array1[ARRAY_SIZE], array2[ARRAY_SIZE];

int acceleratorInitial1, acceleratorFinal1;
int acceleratorInitial2, acceleratorFinal2;
int brakeInitial, brakeFinal;

void setup()
{
    // Set arrays to 0
    InitializeArray(array1, ARRAY_SIZE);
    InitializeArray(array2, ARRAY_SIZE);

    // Begin CAN communication
    Can1.begin();
    Can1.setBaudRate(BAUD_RATE);

    // Begin Serial communication
    Serial.begin(BIT_RATE);
}

void loop()
{
    // Turn brake light on
    BrakeLight(BRAKE, BRAKE_LIGHT);

    // Check if car is ready to drive
    isAlreadyReady = ReadyToDrive(&isAlreadyReady, RTD_BUTTON, BRAKE, RTD_SOUND, RUN, GO);
    
    // Check if car is ready to drive
    if (isAlreadyReady)
    {
        // Obtain accelerator sensor readings
        acceleratorFinal1 = AverageSignal(acceleratorInitial1, acceleratorFinal1, \
            ACCELERATOR_ONE, &total1, &counter1, array1, &average1);
        acceleratorFinal2 = AverageSignal(acceleratorInitial2, acceleratorFinal2, \
            ACCELERATOR_TWO, &total2, &counter2, array2, &average2);

        // Check accelerator signals
        CheckAPPS(&acceleratorInitial1, &acceleratorInitial2);

        // Obtain brake sensor readings and add to both averages
        AverageSignal(brakeInitial, brakeFinal, BRAKE, &total1, &counter1, array1, &average1);
        AverageSignal(brakeInitial, brakeFinal, BRAKE, &total2, &counter2, array2, &average2);

        // Convert signals and send to Bamocar through CAN
        SendPedalMessage(acceleratorFinal1, msgRX, CAN_MESSAGE_ACCELERATOR);
        SendPedalMessage(acceleratorFinal2, msgRX, CAN_MESSAGE_ACCELERATOR);
        SendPedalMessage(brakeFinal, msgRX, CAN_MESSAGE_BRAKE);
    }
}