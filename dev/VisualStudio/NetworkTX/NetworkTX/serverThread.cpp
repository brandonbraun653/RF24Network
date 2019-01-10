/* C++ Includes */
#include <array>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"

/* Chimera Includes */
#include <Chimera/gpio.hpp>
#include <Chimera/spi.hpp>
#include <Chimera/threading.hpp>

/* Project Includes */
#include "nrf24l01.hpp"
#include "RF24Mesh.hpp"
#include "RF24Network.hpp"
//#include "RF24Ethernet.h"

using namespace NRF24L;
using namespace RF24Network;
using namespace Chimera;
using namespace Chimera::Threading;
using namespace Chimera::GPIO;
using namespace Chimera::SPI;

static uint32_t delayTime = 500;
static const uint64_t address = 0x01;
static const std::array<uint8_t, 5> testArray = { 1, 2, 3, 4, 5 };
static const char * helloWorld = "hello world";

static constexpr std::array<char, sizeof("hello world")> testText = { "hello world" };

//static IPAddress myIP(10, 10, 2, 1);
//static IPAddress serverIP(10, 10, 2, 1);
static uint32_t serverPort = 1000;

static NRF24L01 radio;
static Network network(radio);
//static RF24Mesh mesh(radio, network);
//RF24EthernetClass RF24Ethernet(radio, network, mesh);
//RF24EthernetClass RF24Ethernet(radio, network);

// Set up the server to listen on port 1000
//EthernetServer server = EthernetServer(1000);

void serverThread(void * argument)
{
    Setup spiSetup;
    SPIClass_sPtr spi;
    GPIOClass_sPtr chip_enable;

    spi = std::make_shared<SPIClass>(3);

    spiSetup.clockFrequency = 12000000;
    spiSetup.bitOrder = BitOrder::MSB_FIRST;
    spiSetup.clockMode = ClockMode::MODE0;
    spiSetup.mode = Chimera::SPI::Mode::MASTER;

    spiSetup.CS.pin = 7;
    spiSetup.CS.port = Port::PORTF;
    spiSetup.CS.alternate = Thor::Definitions::GPIO::NOALTERNATE;
    spiSetup.CS.mode = Drive::OUTPUT_PUSH_PULL;
    spiSetup.CS.state = State::HIGH;

    spi->setChipSelectControlMode(ChipSelectMode::MANUAL);

    spi->init(spiSetup);
    spi->setPeripheralMode(SubPeripheral::TXRX, SubPeripheralMode::BLOCKING);

    chip_enable = std::make_shared<GPIOClass>();
    chip_enable->init(Port::PORTF, 6);
    chip_enable->setMode(Drive::OUTPUT_PUSH_PULL, false);
    chip_enable->setState(State::HIGH);

    signalThreadSetupComplete();
    TickType_t lastTimeWoken = xTaskGetTickCount();

    radio = NRF24L01(spi, chip_enable);

//    Ethernet.setMac(00);
//    Ethernet.begin(myIP);
//    Ethernet.listen(1000);
//
//    server.begin(1000);

    for(;;)
    {
//        if (EthernetClient client = server.available())
//        {
//            while (client.waitAvailable() > 0)
//            {
//                printf("yup\r\n");
//            }
//        }

        vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(5));
    }

}


void ledThread(void* argument)
{
    GPIOClass led;
    led.init(Port::PORTB, 7);
    led.setMode(Drive::OUTPUT_PUSH_PULL, false);
    led.setState(State::LOW);

    signalThreadSetupComplete();

    TickType_t lastTimeWoken = xTaskGetTickCount();
    for(;;)
    {
        led.setState(State::LOW);
        vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(delayTime));
        led.setState(State::HIGH);
        vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(delayTime));
    }
}
