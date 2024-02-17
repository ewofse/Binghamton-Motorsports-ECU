#include "CANRead.h"

// Send a CAN message
void SendMessage(CAN_message_t msg)
{
    // CAN message parameters
    msg.flags.extended = 0;
    msg.flags.remote = 0;
    msg.flags.overrun = 0;
    msg.flags.reserved = 0;

    // CAN message contents
    msg.id = 0x1;
    msg.len = 1;
    msg.buf[0] = 0xA;

    if (Can1.write(msg))
    {
        Serial.println("Message sent.");
        Serial.println();

        // Output message contents
        // OutputMessageContents(msg);
    }

    else
    {
        Serial.println("Message not sent.");
        Serial.println();
    }
}

// Read a CAN message
void ReadMessage(CAN_message_t msg, const int pin)
{
    // Check if a message was sent
    if (Can1.read(msg))
    {
        // Toggle LED
        digitalWrite(pin, !digitalRead(pin));

        // Output message contents
        OutputMessageContents(msg);
    }

    else
    {
        Serial.println("Message not read.");
        Serial.println();
    }
}

// Output CAN message components to serial monitor
void OutputMessageContents(CAN_message_t msg)
{
    // Output message parameters
    Serial.print("CAN1 "); 
    Serial.print("MB: "); Serial.print(msg.mb);
    Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
    Serial.print("  EXT: "); Serial.print(msg.flags.extended );
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" DATA: ");

    // Output all contents of message
    for (int i = 0; i < msg.len; i++)
    {
        Serial.print(msg.buf[i], HEX);
        Serial.print(" ");
    }

    Serial.print("  TS: "); Serial.println(msg.timestamp);
    Serial.println();
}