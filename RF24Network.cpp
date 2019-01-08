/********************************************************************************
*   RF24Network.cpp
*       Implements the RF24 Network layer. Based on the work originally done by
*       James Coliz on the popular RF24Network library:
*       https://github.com/nRF24/RF24Network.
*
*       This version of the code attempts to make performance improvements, modernize
*       the interface using modern C++, and abstract things further away from specific
*       platforms. The common platform interface is from the Chimera library:
*       https://github.com/brandonbraun653/Chimera
*
*   2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

/* C++ Includes */
#include <cmath>

/* NRF24 Driver Includes */
#include "nrf24l01.hpp"
#include "nrf24l01Definitions.hpp"

/* Network Includes */
#include "RF24Network.hpp"
#include "RF24NetworkDefinitions.hpp"

using namespace NRF24L;
using namespace Chimera;

namespace RF24Network
{
    /**
    *
    */
    static uint16_t levelToAddress(uint8_t level);

    /**
    *
    */
    static uint64_t pipeAddress(uint16_t node, uint8_t pipe);


    static constexpr uint16_t max_frame_payload_size = MAX_FRAME_SIZE - Header::SIZE;


    const char *Header::toString() const
    {
        static char buffer[45];
        sprintf(buffer, "id %u from 0%o to 0%o type %d", id, fromNode, toNode, (int)type);
        return buffer;
    }

    Network::Network(NRF24L01 &radio): radio(radio)
    {
        nextFrame = frameQueue;

        frag_ptr = &fragQueue;
        fragQueue.messageBuffer = &fragQueueMessageBuffer[0];

        txTime = 0;
        networkFlags = 0;
        returnSysMsgs = 0;
        multicastRelay = 0;
    }

    bool Network::begin(const uint8_t channel, const uint16_t nodeAddress, const NRF24L::PowerAmplitude pwr)
    {
        /*------------------------------------------------
        Check error conditions that would prevent a solid startup.
        ------------------------------------------------*/
        if (!isValidNetworkAddress(nodeAddress))
        {
            oopsies = ErrorType::INVALID_ADDRESS;
            IF_SERIAL_DEBUG(printf("ERR: Invalid node address\r\n"););
            return false;
        }

        /*------------------------------------------------
        Turn on the radio. By default, this wipes all pre-existing settings.
        ------------------------------------------------*/
        if (radio.isInitialized())
        {
            /*------------------------------------------------
            The system model is to interact with the network layer, not the
            physical layer. Let this function initialize the radio.
            ------------------------------------------------*/
            oopsies = ErrorType::RADIO_PRE_INITIALIZED;
            IF_SERIAL_DEBUG(printf("ERR: Radio pre-initialized\r\n"););
            return false;
        }
        else if(!radio.begin())
        {
            /*------------------------------------------------
            More than likely a register read/write failed.
            ------------------------------------------------*/
            oopsies = ErrorType::RADIO_FAILED_INIT;
            IF_SERIAL_DEBUG(printf("ERR: Radio HW failed init\r\n"););
            return false;
        }

        /*------------------------------------------------
        Force the setup operations below to negate a failure
        ------------------------------------------------*/
        initialized = true;

        /*------------------------------------------------
        Initialize the radio
        ------------------------------------------------*/
        txTimeout = 25;
        routeTimeout = txTimeout * 3; // Adjust for max delay per node within a single chain
        logicalNodeAddress = nodeAddress;

        initialized &= radio.setChannel(channel);
        initialized &= radio.setPALevel(pwr);
        initialized &= radio.setAutoAck(0, false);

        #if RF24Network_ENABLE_DYNAMIC_PAYLOADS
        radio.enableDynamicPayloads();
        #endif

        /*------------------------------------------------
        Use different retry periods to reduce data collisions
        ------------------------------------------------*/
        auto retryVar = static_cast<AutoRetransmitDelay>((((nodeAddress % 6) + 1) * 2) + 3);
        initialized &= radio.setRetries(retryVar, 5);

        /*------------------------------------------------
        Set up the address helper cache
        ------------------------------------------------*/
        setupAddress();

        /*------------------------------------------------
        Open all the listening pipes
        ------------------------------------------------*/
        for (uint8_t i = 0; i < MAX_NUM_PIPES; i++)
        {
            initialized &= radio.openReadPipe(i, pipeAddress(nodeAddress, i));
        }
        radio.startListening();

        return initialized;
    }

    MessageType Network::update()
    {
        if (!initialized)
        {
            IF_SERIAL_DEBUG(printf("NET: Not initialized\r\n"););
            oopsies = ErrorType::NOT_INITIALIZED;
            return MessageType::NETWORK_ERR;
        }

        uint8_t pipe_num;
        MessageType returnVal = MessageType::TX_NORMAL;

        /*------------------------------------------------
        If BYPASS_HOLDS is enabled, incoming user data may be dropped. This allows for
        system payloads to be read while the user cache is full. HOLD_INCOMING prevents data from
        being read from the radios, thereby preventing incoming payloads from being acked.
        ------------------------------------------------*/
        if (!(networkFlags & static_cast<uint8_t>(FlagType::BYPASS_HOLDS)))
        {
            if (    (networkFlags & static_cast<uint8_t>(FlagType::HOLD_INCOMING))
                 || ((nextFrame - frameQueue + 34) > MAIN_BUFFER_SIZE)
               )
            {
                //TODO: What the heck is this ^^^ 34 doing??

                if (!available())
                {
                    networkFlags &= ~static_cast<uint8_t>(FlagType::HOLD_INCOMING);
                }
                else
                {
                    return returnVal;
                }
            }
        }

        /*------------------------------------------------
        Process the incoming/outgoing data
        ------------------------------------------------*/
        while (radio.available(pipe_num))
        {
            frameSize = radio.getDynamicPayloadSize();
            if (frameSize < Header::SIZE)
            {
                delayMilliseconds(10);
                continue;
            }

            /*------------------------------------------------
            Dump the payloads until we've gotten everything.
            Fetch the payload, and see if this was the last one.
            ------------------------------------------------*/
            radio.read(frameBuffer, frameSize);

            /*------------------------------------------------
            Read the beginning of the frame as the header
            ------------------------------------------------*/
            Header *header = (Header *)(&frameBuffer);

            IF_SERIAL_DEBUG
            (
                const uint16_t *i = reinterpret_cast<const uint16_t *>(frameBuffer + Header::SIZE);
                printf("%lu: MAC Received on %u %s\n\r", millis(), pipe_num, header->toString());
                printf("%lu: NET message %04x\n\r", millis(), *i);
            );

            /*------------------------------------------------
            Throw it away if it's not a valid address
            ------------------------------------------------*/
            if (!isValidNetworkAddress(header->toNode))
            {
                continue;
            }

            returnVal = header->type;

            /*------------------------------------------------
            Is this message for us?
            ------------------------------------------------*/
            if (header->toNode == this->logicalNodeAddress)
            {
                /*------------------------------------------------
                No action required for this one
                ------------------------------------------------*/
                if (header->type == MessageType::NETWORK_PING)
                {
                    continue;
                }

                /*------------------------------------------------

                ------------------------------------------------*/
                if (header->type == MessageType::NETWORK_ADDR_RESPONSE)
                {
                    uint16_t requester = DEFAULT_ADDRESS;
                    if (requester != this->logicalNodeAddress)
                    {
                        header->toNode = requester;
                        writeDirect(header->toNode, MessageType::USER_TX_TO_PHYSICAL_ADDRESS);
                        delayMilliseconds(10);
                        writeDirect(header->toNode, MessageType::USER_TX_TO_PHYSICAL_ADDRESS);
                        continue;
                    }
                }

                /*------------------------------------------------

                ------------------------------------------------*/
                if (header->type == MessageType::NETWORK_REQ_ADDRESS && this->logicalNodeAddress)
                {
                    header->fromNode = this->logicalNodeAddress;
                    header->toNode = 0;
                    writeDirect(header->toNode, MessageType::TX_NORMAL);
                    continue;
                }

                /*------------------------------------------------

                ------------------------------------------------*/
                if (    (returnSysMsgs && (header->type > MessageType::MAX_USER_DEFINED_HEADER_TYPE))
                    ||  header->type == MessageType::NETWORK_ACK
                   )
                {
                    IF_SERIAL_DEBUG_ROUTING(printf("%lu MAC: System payload rcvd %d\n", millis(), returnVal););

                    if (    header->type != MessageType::NETWORK_FIRST_FRAGMENT
                        &&  header->type != MessageType::NETWORK_MORE_FRAGMENTS
                        &&  header->type != MessageType::NETWORK_MORE_FRAGMENTS_NACK
                        &&  header->type != MessageType::EXTERNAL_DATA_TYPE
                        &&  header->type != MessageType::NETWORK_LAST_FRAGMENT
                       )
                    {
                        return returnVal;
                    }
                }

                /*------------------------------------------------
                External data received
                ------------------------------------------------*/
                if (enqueue(header) == 2)
                {
                    return MessageType::EXTERNAL_DATA_TYPE;
                }
            }
            else
            {
                /*------------------------------------------------
                Handle a multicast scenario
                ------------------------------------------------*/
                if (header->toNode == 0100)
                {
                    if (header->type == MessageType::NETWORK_POLL)
                    {
                        if (    !(networkFlags & static_cast<uint8_t>(FlagType::NO_POLL)
                            &&  this->logicalNodeAddress != DEFAULT_ADDRESS)
                           )
                        {
                            header->toNode = header->fromNode;
                            header->fromNode = this->logicalNodeAddress;
                            delayMilliseconds(parentPipe);
                            writeDirect(header->toNode, MessageType::USER_TX_TO_PHYSICAL_ADDRESS);
                        }
                        continue;
                    }
                    uint8_t val = enqueue(header);

                    if (multicastRelay)
                    {
                        IF_SERIAL_DEBUG_ROUTING(printf("%u MAC: FWD multicast frame from 0%o to level %u\n", millis(), header->fromNode, multicast_level + 1););

                        /*------------------------------------------------
                        For all but the first level of nodes (those not directly
                        connected to the master) we add the total delay per level.
                        ------------------------------------------------*/
                        if ((this->logicalNodeAddress >> 3) != 0)
                        {
                            delayMilliseconds(1);
                        }

                        delayMilliseconds(this->logicalNodeAddress % 4);
                        writeDirect(levelToAddress(multicastLevel) << 3, MessageType::USER_TX_MULTICAST);
                    }

                    if (val == 2)
                    {
                        return MessageType::EXTERNAL_DATA_TYPE;
                    }
                }
                else
                {
                    /*------------------------------------------------
                    Send it on, indicate it is a routed payload
                    ------------------------------------------------*/
                    writeDirect(header->toNode, MessageType::TX_ROUTED);
                }
            }
        }
        return returnVal;
    }

    uint8_t Network::enqueue(Header *header)
    {
        bool result = false;
        uint16_t messageSize = frameSize - Header::SIZE;

        IF_SERIAL_DEBUG(printf("%lu: NET Enqueue @%x ", millis(), nextFrame - frameQueue););

        bool isFragment =  (header->type == MessageType::NETWORK_FIRST_FRAGMENT)
                        || (header->type == MessageType::NETWORK_MORE_FRAGMENTS)
                        || (header->type == MessageType::NETWORK_LAST_FRAGMENT)
                        || (header->type == MessageType::NETWORK_MORE_FRAGMENTS_NACK);

        if (isFragment)
        {

            /*------------------------------------------------
            This is the first fragment of many!
            ------------------------------------------------*/
            if (header->type == MessageType::NETWORK_FIRST_FRAGMENT)
            {
                // Drop frames exceeding max size and duplicates (MAX_PAYLOAD_SIZE needs to be divisible by 24)
                if (header->reserved > (uint16_t(MAX_PAYLOAD_SIZE) / max_frame_payload_size))
                {
                    IF_SERIAL_DEBUG_FRAGMENTATION(printf("Frag frame with %d frags exceeds MAX_PAYLOAD_SIZE or out of sequence\n", header->reserved););

                    fragQueue.header.reserved = 0;
                    return false;
                }
                else if (fragQueue.header.id == header->id && fragQueue.header.fromNode == header->fromNode)
                {
                    return true;
                }

                if ((header->reserved * 24) > (MAX_PAYLOAD_SIZE - (nextFrame - frameQueue)))
                {
                    networkFlags |= static_cast<uint8_t>(FlagType::HOLD_INCOMING);
                    radio.stopListening();
                }

                memcpy(&fragQueue, &frameBuffer, 8);
                memcpy(fragQueue.messageBuffer, frameBuffer + Header::SIZE, messageSize);

                IF_SERIAL_DEBUG_FRAGMENTATION(printf("queue first, total frags %d\r\n", header->reserved););

                //Store the total size of the stored frame in messageSize
                fragQueue.messageSize = messageSize;
                --fragQueue.header.reserved;

                IF_SERIAL_DEBUG_FRAGMENTATION_L2
                (
                    for (int i = 0; i < frag_queue.messageSize; i++)
                    {
                        printf("0x%02x ", frag_queue.messageBuffer[i]);
                    }
                    printf("\r\n")
                );

                return true;
            }
            /*------------------------------------------------
            More fragments || last fragment!
            ------------------------------------------------*/
            else if(    (header->type == MessageType::NETWORK_LAST_FRAGMENT)
                    ||  (header->type == MessageType::NETWORK_MORE_FRAGMENTS)
                    ||  (header->type == MessageType::NETWORK_MORE_FRAGMENTS_NACK)
                   )
            {
                /*------------------------------------------------

                ------------------------------------------------*/
                if (fragQueue.messageSize + messageSize > MAX_PAYLOAD_SIZE)
                {
                    IF_SERIAL_DEBUG_FRAGMENTATION(printf("Drop frag 0x%02x Size exceeds max.\r\n", header->reserved););
                    fragQueue.header.reserved = 0;
                    return false;
                }

                /*------------------------------------------------

                ------------------------------------------------*/
                if (    (fragQueue.header.reserved == 0)
                    ||  (header->type != MessageType::NETWORK_LAST_FRAGMENT && header->reserved != fragQueue.header.reserved)
                    ||  (fragQueue.header.id != header->id)
                   )
                {
                    IF_SERIAL_DEBUG_FRAGMENTATION(printf("Drop frag 0x%02x header id %d out of order\r\n", header->reserved, header->id););
                    return false;
                }

                /*------------------------------------------------

                ------------------------------------------------*/
                memcpy(fragQueue.messageBuffer + fragQueue.messageSize, frameBuffer + Header::SIZE, messageSize);
                fragQueue.messageSize += messageSize;

                /*------------------------------------------------

                ------------------------------------------------*/
                if (header->type != MessageType::NETWORK_LAST_FRAGMENT)
                {
                    --fragQueue.header.reserved;
                    return true;
                }


                fragQueue.header.reserved = 0;
                fragQueue.header.type = static_cast<MessageType>(header->reserved);

                IF_SERIAL_DEBUG_FRAGMENTATION(printf("fq 3: %d\n", frag_queue.messageSize););
                IF_SERIAL_DEBUG_FRAGMENTATION_L2
                (
                    for (int i = 0; i < frag_queue.messageSize; i++)
                    {
                        printf("0x%02x", frag_queue.messageBuffer[i]);
                    }
                );

                //Frame assembly complete, copy to main buffer if OK
                if (fragQueue.header.type == MessageType::EXTERNAL_DATA_TYPE)
                {
                    return static_cast<uint8_t>(MessageType::USER_TX_TO_PHYSICAL_ADDRESS);
                }

                if (MAX_PAYLOAD_SIZE - (nextFrame - frameQueue) >= fragQueue.messageSize)
                {
                    memcpy(nextFrame, &fragQueue, Frame::PREAMBLE_SIZE_Byte);
                    memcpy(nextFrame + Frame::PREAMBLE_SIZE_Byte, fragQueue.messageBuffer, fragQueue.messageSize);
                    nextFrame += (Frame::PREAMBLE_SIZE_Byte + fragQueue.messageSize);

                    if (uint8_t padding = (fragQueue.messageSize + Frame::PREAMBLE_SIZE_Byte) % 4)
                    {
                        nextFrame += 4 - padding;
                    }

                    IF_SERIAL_DEBUG_FRAGMENTATION(printf("enq size %d\n", frag_queue.messageSize););
                    return true;
                }
                else
                {
                    radio.stopListening();
                    networkFlags |= static_cast<uint8_t>(FlagType::HOLD_INCOMING);
                }
                IF_SERIAL_DEBUG_FRAGMENTATION(printf("Drop frag payload, queue full\r\n"););
                return false;
            }
        }
        /*------------------------------------------------
        Nope, it's not a fragment
        ------------------------------------------------*/
        else
        {
            if (header->type == MessageType::EXTERNAL_DATA_TYPE)
            {
                memcpy(&fragQueue, &frameBuffer, 8);
                fragQueue.messageBuffer = frameBuffer + Header::SIZE;
                fragQueue.messageSize = messageSize;
                return static_cast<uint8_t>(MessageType::USER_TX_TO_PHYSICAL_ADDRESS);
            }


            if (messageSize + (nextFrame - frameQueue) <= MAIN_BUFFER_SIZE)
            {
                memcpy(nextFrame, &frameBuffer, Frame::PREAMBLE_FIELD_HEADER_SIZE_Byte);
                memcpy(nextFrame + Frame::PREAMBLE_FIELD_HEADER_SIZE_Byte, &messageSize, Frame::PREAMBLE_FIELD_PAYLOAD_SIZE_Byte);
                memcpy(nextFrame + Frame::PREAMBLE_SIZE_Byte, frameBuffer + Frame::PREAMBLE_FIELD_HEADER_SIZE_Byte, messageSize);

                IF_SERIAL_DEBUG_FRAGMENTATION
                (
                    for(int i=0; i<messageSize;i++)
                    {
                        printf("0x%02x : ", nextFrame[i]);
                    }
                    printf("\r\n");
                );

                nextFrame += (messageSize + Frame::PREAMBLE_SIZE_Byte);

                uint8_t padding = (messageSize + Frame::PREAMBLE_SIZE_Byte) % 4;
                if (padding)
                {
                    nextFrame += 4 - padding;
                }

                IF_SERIAL_DEBUG_FRAGMENTATION(printf("Enq %d\r\n", (nextFrame-frameQueue)););

                result = true;
            }
            else
            {
                result = false;
                IF_SERIAL_DEBUG(printf("NET **Drop Payload** Buffer Full\r\n"););
            }
            return result;
        }
    }

    bool Network::available() const
    {
        if (!initialized)
        {
            return false;
        }

        return (nextFrame > frameQueue);
    }

    uint16_t Network::parent() const
    {
        if (logicalNodeAddress == 0)
        {
            return -1;
        }
        else
        {
            return parentNode;
        }
    }

    uint16_t Network::peek(Header &header)
    {
        if (available())
        {
            uint16_t msg_size;
            Frame *frame = (Frame *)(frameQueue);

            memcpy(&header, &frame->header, Header::SIZE);
            memcpy(&msg_size, frameQueue + Frame::PREAMBLE_FIELD_HEADER_SIZE_Byte, Frame::PREAMBLE_FIELD_PAYLOAD_SIZE_Byte);

            return msg_size;
        }

        return 0;
    }

    void Network::peek(Header &header, void *message, uint16_t maxlen)
    {
        if (available())
        {
            /*------------------------------------------------
            Copy the header
            ------------------------------------------------*/
            memcpy(&header, frameQueue, 8);

            /*------------------------------------------------
            Copy the message
            ------------------------------------------------*/
            if (maxlen > 0)
            {
                memcpy(message, frameQueue + Frame::PREAMBLE_SIZE_Byte, maxlen);
            }
        }
    }

    uint16_t Network::read(Header &header, void *message, uint16_t maxlen)
    {
        uint8_t padding = 0;        /** TODO */
        uint16_t readLength = 0;    /** How many bytes will actually be read */
        uint16_t bufferSize = 0;    /** How large the payload is */

        const uint8_t *framePayloadDataOffset = frameQueue + Frame::PREAMBLE_SIZE_Byte;
        const uint8_t *framePayloadSizeOffset = frameQueue + Frame::PREAMBLE_FIELD_HEADER_SIZE_Byte;

        if (available())
        {
            /*------------------------------------------------
            Copy the header over to the user's variable and
            ------------------------------------------------*/
            memcpy(&header, frameQueue, Frame::PREAMBLE_FIELD_HEADER_SIZE_Byte);
            memcpy(&bufferSize, framePayloadSizeOffset, Frame::PREAMBLE_FIELD_PAYLOAD_SIZE_Byte);

            /*------------------------------------------------
            Make sure we are reading more than 0 bytes
            ------------------------------------------------*/
            if (maxlen > 0)
            {
                /*------------------------------------------------
                Copy over the message data
                ------------------------------------------------*/
                readLength = std::min(maxlen, bufferSize);
                memcpy(message, framePayloadDataOffset, readLength);

                /*------------------------------------------------
                If enabled, print out the message data
                ------------------------------------------------*/
                IF_SERIAL_DEBUG
                (
                    uint16_t len = maxlen;
                    printf("%lu: NET message size %d\n", millis(), bufferSize);
                    printf("%lu: NET r message ", millis());
                    const uint8_t *charPtr = reinterpret_cast<const uint8_t *>(message);
                    while (len--)
                    {
                        printf("%02x ", charPtr[len]);
                    }

                    printf("\n\r");
                );
            }

            /*------------------------------------------------
            TODO: Yeah I'm not too sure what this is for
            ------------------------------------------------*/
            nextFrame -= bufferSize + Frame::PREAMBLE_SIZE_Byte;

            if ((padding = (bufferSize + Frame::PREAMBLE_SIZE_Byte) % 4))
            {
                padding = 4 - padding;
                nextFrame -= padding;
            }

            /*------------------------------------------------
            TODO: I think this is shifting the next frame to the top of the queue?
            ------------------------------------------------*/
            memmove(frameQueue, frameQueue + bufferSize + Frame::PREAMBLE_SIZE_Byte + padding, sizeof(frameQueue) - bufferSize);
            IF_SERIAL_DEBUG(printf("%lu: NET Received %s\n\r", millis(), header.toString()););
        }

        return bufferSize;
    }

    bool Network::multicast(Header &header, const void *const message, const uint16_t len, const uint8_t level)
    {
        /*------------------------------------------------
        Fill out the header
        ------------------------------------------------*/
        header.toNode = 0100; /** TODO: What does this number mean?? It's not master...000 */
        header.fromNode = this->logicalNodeAddress;

        return write(header, message, len, levelToAddress(level));
    }

    bool Network::write(Header &header, const void *const message, const uint16_t len)
    {
        return write(header, message, len, 070); /** TODO: the heck is THIS number? */
    }

    bool Network::write(Header &header, const void *const message, const uint16_t len, const uint16_t writeDirect)
    {
        /*------------------------------------------------
        Protect against invalid inputs
        ------------------------------------------------*/
        if (!initialized)
        {
            oopsies = ErrorType::NOT_INITIALIZED;
            return false;
        }
        else if (!message || !len)
        {
            oopsies = ErrorType::INVALID_INPUTS;
            return false;
        }

        /*------------------------------------------------
        Allows time for requests (RF24Mesh) to get through between failed writes on busy nodes
        ------------------------------------------------*/
        while (millis() - txTime < 25)
        {
            if (update() > MessageType::MAX_USER_DEFINED_HEADER_TYPE)
            {
                break;
            }
        }
        delayMicroseconds(200);

        if (len <= max_frame_payload_size)
        {
            //Normal Write (Un-Fragmented)
            frameSize = len + Header::SIZE;
            if (_write(header, message, len, writeDirect))
            {
                return 1;
            }
            txTime = millis();
            return 0;
        }

        //Check payload size
        if (len > MAX_PAYLOAD_SIZE)
        {
            IF_SERIAL_DEBUG(printf("%lu: NET write message failed. Given 'len' %u is bigger than the MAX Payload size %i\n\r", millis(), len, MAX_PAYLOAD_SIZE););
            return false;
        }

        //Divide the message payload into chunks of max_frame_payload_size
        uint8_t fragment_id = (len % max_frame_payload_size != 0) + ((len) / max_frame_payload_size); //the number of fragments to send = ceil(len/max_frame_payload_size)

        uint8_t msgCount = 0;

        IF_SERIAL_DEBUG_FRAGMENTATION(printf("%lu: FRG Total message fragments %d\n\r", millis(), fragment_id););

        if (header.toNode != 0100)
        {
            networkFlags |= static_cast<uint8_t>(FlagType::FAST_FRAG);
            radio.stopListening();
        }

        uint8_t retriesPerFrag = 0;
        MessageType type = header.type;
        bool ok = 0;

        while (fragment_id > 0)
        {
            //Copy and fill out the header
            //Header fragmentHeader = header;
            header.reserved = fragment_id;

            if (fragment_id == 1)
            {
                header.type = MessageType::NETWORK_LAST_FRAGMENT; //Set the last fragment flag to indicate the last fragment
                header.reserved = static_cast<uint8_t>(type);              //The reserved field is used to transmit the header type
            }
            else
            {
                if (msgCount == 0)
                {
                    header.type = MessageType::NETWORK_FIRST_FRAGMENT;
                }
                else
                {
                    header.type = MessageType::NETWORK_MORE_FRAGMENTS; //Set the more fragments flag to indicate a fragmented frame
                }
            }

            uint16_t offset = msgCount * max_frame_payload_size;
            uint16_t fragmentLen = std::min((unsigned int)(len - offset), (unsigned int)max_frame_payload_size);

            //Try to send the payload chunk with the copied header
            frameSize = Header::SIZE + fragmentLen;
            ok = _write(header, ((char *)message) + offset, fragmentLen, writeDirect);

            if (!ok)
            {
                delayMilliseconds(2);
                ++retriesPerFrag;
            }
            else
            {
                retriesPerFrag = 0;
                fragment_id--;
                msgCount++;
            }

            if (!ok && retriesPerFrag >= 3)
            {
                IF_SERIAL_DEBUG_FRAGMENTATION(printf("%lu: FRG TX with fragmentID '%d' failed after %d fragments. Abort.\n\r", millis(), fragment_id, msgCount););
                break;
            }

            //Message was successful sent
            IF_SERIAL_DEBUG_FRAGMENTATION_L2(printf("%lu: FRG message transmission with fragmentID '%d' sucessfull.\n\r", millis(), fragment_id););
        }
        header.type = type;

        if (networkFlags & static_cast<uint8_t>(FlagType::FAST_FRAG))
        {
            ok = radio.txStandBy(txTimeout);
            radio.startListening();
            radio.setAutoAck(0, 0);
        }
        networkFlags &= ~static_cast<uint8_t>(FlagType::FAST_FRAG);

        if (!ok)
        {
            return false;
        }

        //Return true if all the chunks where sent successfully
        IF_SERIAL_DEBUG_FRAGMENTATION(printf("%lu: FRG total message fragments sent %i. \n", millis(), msgCount););
        if (fragment_id > 0)
        {
            txTime = millis();
            return false;
        }

        return true;
    }

    bool Network::_write(Header &header, const void *message, uint16_t len, uint16_t directTo)
    {
        /*------------------------------------------------
        Fill out the header
        ------------------------------------------------*/
        header.fromNode = logicalNodeAddress;

        /*------------------------------------------------
        Build the full frame to send
        ------------------------------------------------*/
        memcpy(frameBuffer, &header, Header::SIZE);
        IF_SERIAL_DEBUG(printf("%lu: NET Sending %s\n\r", millis(), header.toString()););

        if (len)
        {
            memcpy(frameBuffer + Header::SIZE, message, len);
            IF_SERIAL_DEBUG
            (
                uint16_t tmpLen = len;
                const uint8_t *charPtr = reinterpret_cast<const uint8_t *>(message);

                printf("%lu: NET message ", millis());
                while (tmpLen--)
                {
                    printf("%02x ", charPtr[tmpLen]);
                }
                    printf("\n\r");
            );
        }

        if (directTo != 070)
        {
            /*------------------------------------------------
            Payload is multicast to the first node, and routed normally to the next
            ------------------------------------------------*/
            MessageType sendType = MessageType::USER_TX_TO_LOGICAL_ADDRESS;

            if (header.toNode == 0100)
            {
                sendType = MessageType::USER_TX_MULTICAST;
            }

            if (header.toNode == directTo)
            {
                sendType = MessageType::USER_TX_TO_PHYSICAL_ADDRESS; // Payload is multicast to the first node, which is the recipient
            }

            return writeDirect(directTo, sendType);
        }

        return writeDirect(header.toNode, MessageType::TX_NORMAL);
    }

    bool Network::writeDirect(uint16_t toNode, MessageType directTo)
    {
        // Direct To: 0 = First Payload, standard routing, 1=routed payload, 2=directRoute to host, 3=directRoute to Route

        bool ok = false;
        bool isAckType = false;

        if (frameBuffer[6] > 64 && frameBuffer[6] < 192)
        {
            isAckType = true;
        }

        // Throw it away if it's not a valid address
        if (!isValidNetworkAddress(toNode))
            return false;

        //Load info into our conversion structure, and get the converted address info
        logicalToPhysicalStruct conversion = {toNode, static_cast<uint8_t>(directTo), 0};
        logicalToPhysicalAddress(&conversion);

        IF_SERIAL_DEBUG(printf("%lu: MAC Sending to 0%o via 0%o on pipe %x\n\r", millis(), toNode, conversion.send_node, conversion.send_pipe););

        /**Write it*/
        ok = writeToPipe(conversion.send_node, conversion.send_pipe, conversion.multicast);

        if (!ok)
        {
            IF_SERIAL_DEBUG_ROUTING(printf("%lu: MAC Send fail to 0%o via 0%o on pipe %x\n\r", millis(), toNode, conversion.send_node, conversion.send_pipe););
        }

        if (    ok
            &&  isAckType
            &&  directTo == MessageType::TX_ROUTED
            &&  conversion.send_node == toNode
           )
        {

            Header *header = (Header *)&frameBuffer;
            header->type = MessageType::NETWORK_ACK;          // Set the payload type to NETWORK_ACK
            header->toNode = header->fromNode; // Change the 'to' address to the 'from' address

            conversion.send_node = header->fromNode;
            conversion.send_pipe = static_cast<uint8_t>(MessageType::TX_ROUTED);
            conversion.multicast = 0;
            logicalToPhysicalAddress(&conversion);

            //Write the data using the resulting physical address
            frameSize = Header::SIZE;
            writeToPipe(conversion.send_node, conversion.send_pipe, conversion.multicast);

            IF_SERIAL_DEBUG_ROUTING(printf("%lu MAC: Route OK to 0%o ACK sent to 0%o\n", millis(), toNode, header->fromNode););
        }

        if (    ok
            &&  conversion.send_node != toNode
            &&  (directTo == MessageType::TX_NORMAL || directTo == MessageType::USER_TX_TO_LOGICAL_ADDRESS)
            &&  isAckType
           )
        {
            // Now, continue listening
            if (networkFlags & static_cast<uint8_t>(FlagType::FAST_FRAG))
            {
                radio.txStandBy(txTimeout);
                networkFlags &= ~static_cast<uint8_t>(FlagType::FAST_FRAG);
                radio.setAutoAck(0, 0);
            }
            radio.startListening();

            uint32_t reply_time = millis();

            while (update() != MessageType::NETWORK_ACK)
            {
                if (millis() - reply_time > routeTimeout)
                {
                    IF_SERIAL_DEBUG_ROUTING(printf("%lu: MAC Network ACK fail from 0%o via 0%o on pipe %x\n\r", millis(), toNode, conversion.send_node, conversion.send_pipe););
                    ok = false;
                    break;
                }
            }
        }

        if (!(networkFlags & static_cast<uint8_t>(FlagType::FAST_FRAG)))
        {
            // Now, continue listening
            radio.startListening();
        }

        return ok;
    }

    bool Network::logicalToPhysicalAddress(logicalToPhysicalStruct *conversionInfo)
    {
        //Create pointers so this makes sense.. kind of
        //We take in the toNode(logical) now, at the end of the function, output the send_node(physical) address, etc.
        //back to the original memory address that held the logical information.
        uint16_t *toNode = &conversionInfo->send_node;
        uint8_t *directTo = &conversionInfo->send_pipe;
        bool *multicast = &conversionInfo->multicast;

        // Where do we send this?  By default, to our parent
        uint16_t pre_conversion_send_node = parentNode;

        // On which pipe
        uint8_t pre_conversion_send_pipe = parentPipe;

        if (*directTo > static_cast<uint8_t>(MessageType::TX_ROUTED))
        {
            pre_conversion_send_node = *toNode;
            *multicast = 1;
            pre_conversion_send_pipe = 0;
        }
        else if (isDirectChild(*toNode))
        {
            // Send directly
            pre_conversion_send_node = *toNode;
            // To its listening pipe
            pre_conversion_send_pipe = 5;
        }
        // If the node is a child of a child
        // talk on our child's listening pipe,
        // and let the direct child relay it.
        else if (isDescendant(*toNode))
        {
            pre_conversion_send_node = directChildRouteTo(*toNode);
            pre_conversion_send_pipe = 5;
        }

        *toNode = pre_conversion_send_node;
        *directTo = pre_conversion_send_pipe;

        return 1;
    }

    bool Network::writeToPipe(uint16_t node, uint8_t pipe, bool multicast)
    {
        bool ok = false;
        uint64_t writePipeAddress = pipeAddress(node, pipe);

        // Open the correct pipe for writing.
        // First, stop listening so we can talk

        if (!(networkFlags & static_cast<uint8_t>(FlagType::FAST_FRAG)))
        {
            radio.stopListening();
        }

        /*------------------------------------------------
        If we are multicasting, turn off auto ack. We don't
        care how the message gets out as long as it does.
        ------------------------------------------------*/
        if (multicast)
        {
            radio.setAutoAck(0, false);
        }
        else
        {
            radio.setAutoAck(0, true);
        }

        radio.openWritePipe(writePipeAddress);

        ok = radio.writeFast(frameBuffer, frameSize, false);

        if (!(networkFlags & static_cast<uint8_t>(FlagType::FAST_FRAG)))
        {
            ok = radio.txStandBy(txTimeout);
            radio.setAutoAck(0, 0);
        }

        return ok;
    }

    bool Network::isDirectChild(uint16_t node)
    {
        bool result = false;

        // A direct child of ours has the same low numbers as us, and only
        // one higher number.
        //
        // e.g. node 0234 is a direct child of 034, and node 01234 is a
        // descendant but not a direct child

        // First, is it even a descendant?
        if (isDescendant(node))
        {
            // Does it only have ONE more level than us?
            uint16_t child_node_mask = (~nodeMask) << 3;
            result = (node & child_node_mask) == 0;
        }
        return result;
    }

    bool Network::isDescendant(uint16_t node)
    {
        return (node & nodeMask) == logicalNodeAddress;
    }

    void Network::setupAddress()
    {
        /*------------------------------------------------
        First, establish the node mask
        ------------------------------------------------*/
        uint16_t node_mask_check = 0xFFFF;
        uint8_t count = 0;

        while (this->logicalNodeAddress & node_mask_check)
        {
            node_mask_check <<= 3;
            count++;
        }
        multicastLevel = count;

        nodeMask = ~node_mask_check;

        /*------------------------------------------------
        Parent mask is the next level down
        ------------------------------------------------*/
        uint16_t parent_mask = nodeMask >> 3;

        /*------------------------------------------------
        Parent node is the part IN the mask
        ------------------------------------------------*/
        parentNode = this->logicalNodeAddress & parent_mask;

        /*------------------------------------------------
        Parent pipe is the part OUT of the mask
        ------------------------------------------------*/
        uint16_t i = this->logicalNodeAddress;
        uint16_t m = parent_mask;

        while (m)
        {
            i >>= 3;
            m >>= 3;
        }
        parentPipe = i;

        IF_SERIAL_DEBUG_MINIMAL(printf("setup_address node=0%o mask=0%o parent=0%o pipe=0%o\n\r", this->logicalNodeAddress, node_mask, parent_node, parent_pipe););
    }

    uint16_t Network::addressOfPipe(uint16_t node, uint8_t pipeNo)
    {
        //Say this node is 013 (1011), mask is 077 or (00111111)
        //Say we want to use pipe 3 (11)
        //6 bits in node mask, so shift pipeNo 6 times left and | into address
        uint16_t m = nodeMask >> 3;
        uint8_t i = 0;

        while (m)
        {            //While there are bits left in the node mask
            m >>= 1; //Shift to the right
            i++;     //Count the # of increments
        }
        return node | (pipeNo << i);
    }

    uint16_t Network::directChildRouteTo(uint16_t node)
    {
        // Presumes that this is in fact a child!!
        uint16_t child_mask = (nodeMask << 3) | 0x07;
        return node & child_mask;
    }

    bool Network::isValidNetworkAddress(const uint16_t node)
    {
        bool result = true;
        uint16_t n = node;

        while (n)
        {
            uint8_t digit = n & 0x07;
            if (digit < 0 || digit > 5) //Allow our out of range multicast address
            {
                result = false;
                IF_SERIAL_DEBUG_MINIMAL(printf("*** WARNING *** Invalid address 0%o\n\r", n););
                break;
            }

            n >>= 3;
        }

        return result;
    }

    void Network::setMulticastLevel(uint8_t level)
    {
        multicastLevel = level;
        radio.openReadPipe(0, pipeAddress(levelToAddress(level), 0));
    }

    static uint16_t levelToAddress(uint8_t level)
    {
        uint16_t levelAddr = 1;
        if (level)
        {
            levelAddr = levelAddr << ((level - 1) * 3);
        }
        else
        {
            return 0;
        }
        return levelAddr;
    }

    static uint64_t pipeAddress(uint16_t node, uint8_t pipe)
    {
        static uint8_t address_translation[] = {0xc3, 0x3c, 0x33, 0xce, 0x3e, 0xe3, 0xec};
        uint64_t result = 0xCCCCCCCCCCLL;
        uint8_t *out = reinterpret_cast<uint8_t *>(&result);

        // Translate the address to use our optimally chosen radio address bytes
        uint8_t count = 1;
        uint16_t dec = node;

        /*------------------------------------------------
        Convert our decimal values to octal, translate them to address bytes, and set our address
        ------------------------------------------------*/
        while (dec)
        {
            if (pipe != 0 || !node)
            {
                out[count] = address_translation[(dec % 8)];
            }

            dec /= 8;
            count++;
        }

        if (pipe != 0 || !node)
        {
            out[0] = address_translation[pipe];
        }
        else
        {
            out[1] = address_translation[count - 1];
        }

        IF_SERIAL_DEBUG
        (
            uint32_t *top = reinterpret_cast<uint32_t *>(out + 1);
            printf("%lu: NET Pipe %i on node 0%o has address %lx%x\n\r", millis(), pipe, node, *top, *out);
        );

        return result;
    }

} /* !NRF24Network */
