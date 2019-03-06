/********************************************************************************
*   RF24Network.cpp
*       Implementation of the NRF24 network layer.
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

namespace RF24Network
{

    static constexpr uint16_t max_frame_payload_size = FRAME_TOTAL_SIZE - sizeof(Header_t);

    /*------------------------------------------------
    Initialize private static variables
    ------------------------------------------------*/
    uint16_t Header::universalHeaderID = 0u;

    const char *Header::toString() const
    {
        static char buffer[45];
        sprintf(buffer, "Id: %u, Src: 0%o, Dst: 0%o, Type: %d, Reserved: %d",
            data.id, data.srcNode, data.dstNode, (int)data.msgType, (int)data.reserved);
        return buffer;
    }

    Network::Network(NRF24L01 &radio): radio(radio)
    {
        txTime = 0;
        networkFlags = 0;
        returnSysMsgs = 0;
        multicastRelay = 0;

        children.fill(EMPTY_LOGICAL_ADDRESS);
        childAttached.fill(false);
    }

    bool Network::begin(const uint8_t channel, const uint16_t nodeAddress, const NRF24L::DataRate dataRate, const NRF24L::PowerAmplitude pwr)
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
        initialized &= radio.setDataRate(dataRate);
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
            initialized &= radio.openReadPipe(i, pipeAddress(nodeAddress, i), true);
        }
        radio.startListening();

        return initialized;
    }

    MessageType Network::update()
    {
        if (!initialized)
        {
            IF_SERIAL_DEBUG(printf("%lu: NET Not initialized\r\n", radio.millis()););
            oopsies = ErrorType::NOT_INITIALIZED;
            return MessageType::NETWORK_ERR;
        }

        uint8_t pipeNum = 0u;
        MessageType returnVal = MessageType::TX_NORMAL;

        /*------------------------------------------------
        If BYPASS_HOLDS is enabled, incoming user data may be dropped. This allows for
        system payloads to be read while the user cache is full. HOLD_INCOMING prevents data from
        being read from the radios, thereby preventing incoming payloads from being acked.
        ------------------------------------------------*/
        if (!(networkFlags & static_cast<uint8_t>(FlagType::BYPASS_HOLDS)))
        {
            if (    (networkFlags & static_cast<uint8_t>(FlagType::HOLD_INCOMING))
                 || frameQueue.full()
               )
            {
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
        while (radio.available(pipeNum))
        {
            radioPayloadSize = radio.getDynamicPayloadSize();
            if (radioPayloadSize < FRAME_PREAMBLE_SIZE)
            {
                radio.delayMilliseconds(10);
                continue;
            }

            /*------------------------------------------------
            Dump the payloads until we've gotten everything.
            Fetch the payload, and see if this was the last one.
            ------------------------------------------------*/
            radio.read(frameBuffer.begin(), radioPayloadSize);

            /*------------------------------------------------
            Read the beginning of the frame as the header
            ------------------------------------------------*/
            Frame frame(frameBuffer);

            Header_t *header = &frame.data.header;

            IF_SERIAL_DEBUG
            (
                Header headerClass;
                headerClass = frame.data.header;
                printf("%lu: MAC Received on pipe %u, Header: [%s]\n\r", radio.millis(), pipeNum, headerClass.toString());
            );

            /*------------------------------------------------
            Throw it away if it's not a valid address
            ------------------------------------------------*/
            if (!isValidNetworkAddress(header->dstNode))
            {
                continue;
            }

            returnVal = static_cast<MessageType>(header->msgType);

            /*------------------------------------------------
            Is this message for us?
            ------------------------------------------------*/
            if (header->dstNode == this->logicalNodeAddress)
            {
                /*------------------------------------------------
                No action required for this one
                ------------------------------------------------*/
                if (header->msgType == static_cast<uint8_t>(MessageType::NETWORK_PING))
                {
                    continue;
                }

                /*------------------------------------------------
                Allow the Mesh layer to process the address response
                ------------------------------------------------*/
                if (header->msgType == static_cast<uint8_t>(MessageType::MESH_ADDR_RESPONSE))
                {
                    return MessageType::MESH_ADDR_RESPONSE;
                }

                /*------------------------------------------------
                Allow the Mesh layer to process the address requesting
                ------------------------------------------------*/
                if (header->msgType == static_cast<uint8_t>(MessageType::MESH_REQ_ADDRESS))
                {
                    return MessageType::MESH_REQ_ADDRESS;
                }

                /*------------------------------------------------

                ------------------------------------------------*/
                if (    (returnSysMsgs && (header->msgType > static_cast<uint8_t>(MessageType::MAX_USER_DEFINED_HEADER_TYPE)))
                    ||  header->msgType == static_cast<uint8_t>(MessageType::NETWORK_ACK)
                   )
                {
                    IF_SERIAL_DEBUG_ROUTING(printf("%lu MAC: System payload rcvd %d\n", radio.millis(), returnVal););

                    if (    header->msgType != static_cast<uint8_t>(MessageType::NETWORK_FIRST_FRAGMENT)
                        &&  header->msgType != static_cast<uint8_t>(MessageType::NETWORK_MORE_FRAGMENTS)
                        &&  header->msgType != static_cast<uint8_t>(MessageType::NETWORK_MORE_FRAGMENTS_NACK)
                        &&  header->msgType != static_cast<uint8_t>(MessageType::EXTERNAL_DATA_TYPE)
                        &&  header->msgType != static_cast<uint8_t>(MessageType::NETWORK_LAST_FRAGMENT)
                       )
                    {
                        return returnVal;
                    }
                }

                enqueue(frame);
            }
            else
            {
                /*------------------------------------------------
                Handle a multicast scenario
                ------------------------------------------------*/
                if (header->dstNode == MULTICAST_ADDRESS)
                {
                    if (header->msgType == static_cast<uint8_t>(MessageType::NETWORK_POLL))
                    {
                        /*------------------------------------------------
                        Assuming we:
                            1. Have a slot available for another child to attach
                            2. Are initialized and connected to the network
                            3. Are not blocked by a flag

                        Send an empty message directly back to the polling node to indicate they can attach
                        ------------------------------------------------*/
                        if (    childrenAvailable()
                            && (this->logicalNodeAddress != DEFAULT_LOGICAL_ADDRESS)
                            &&  !(networkFlags & static_cast<uint8_t>(FlagType::NO_POLL))
                           )
                        {
                            header->dstNode = header->srcNode;
                            header->srcNode = this->logicalNodeAddress;

                            radio.delayMilliseconds(parentPipe);

                            memcpy(frameBuffer.begin(), header, sizeof(Header_t));
                            writeDirect(header->dstNode, MessageType::USER_TX_TO_PHYSICAL_ADDRESS);
                        }

                        continue;
                    }

                    if (multicastRelay)
                    {
                        IF_SERIAL_DEBUG_ROUTING(printf("%u: MAC FWD multicast frame from 0%o to level %u\n", radio.millis(), header.fromNode, multicast_level + 1););

                        /*------------------------------------------------
                        For all but the first level of nodes (those not directly
                        connected to the master) we add the total delay per level.
                        ------------------------------------------------*/
                        if ((this->logicalNodeAddress >> 3) != 0)
                        {
                            radio.delayMilliseconds(1);
                        }

                        radio.delayMilliseconds(this->logicalNodeAddress % 4);
                        writeDirect(levelToAddress(multicastLevel) << 3, MessageType::USER_TX_MULTICAST);
                    }
                }
                else
                {
                    /*------------------------------------------------
                    Send it on, indicate it is a routed payload
                    ------------------------------------------------*/
                    writeDirect(header->dstNode, MessageType::TX_ROUTED);
                }
            }
        }
        return returnVal;
    }

    bool Network::available() const
    {
        if (!initialized)
        {
            return false;
        }

        return !frameQueue.empty();
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
        uint16_t msgSize = 0u;

        if (available())
        {
            auto frame = frameQueue.front();

            header = frame.header;
            msgSize = frame.messageLength;
        }

        return msgSize;
    }

    void Network::peek(Header &header, void *message, uint16_t maxlen)
    {
        if (available())
        {
            auto frame = frameQueue.front();
            /*------------------------------------------------
            Copy the header and message
            ------------------------------------------------*/
            header = frame.header;

            if (maxlen <= frame.message.max_size())
            {
                memcpy(message, frame.message.begin(), maxlen);
            }
        }
    }

    uint16_t Network::read(Header &header, void *message, uint16_t maxlen)
    {
        uint16_t msgLen = 0;    /** How large the payload is */

        if (available())
        {
            uint16_t readLength = 0;
            auto frame = frameQueue.front();

            /*------------------------------------------------
            Copy the header over to the user's variable
            ------------------------------------------------*/
            header = frame.header;
            msgLen = frame.messageLength;

            /*------------------------------------------------
            Make sure we are reading more than 0 bytes
            ------------------------------------------------*/
            if (maxlen <= frame.message.max_size())
            {
                /*------------------------------------------------
                Copy over the message data
                ------------------------------------------------*/
                readLength = std::min(maxlen, msgLen);
                memcpy(message, frame.message.cbegin(), readLength);

                /*------------------------------------------------
                If enabled, print out the message data
                ------------------------------------------------*/
                IF_SERIAL_DEBUG
                (
                    uint16_t len = maxlen;
                    printf("%lu: NET message size %d\n", radio.millis(), msgLen);
                    printf("%lu: NET r message ", radio.millis());
                    const uint8_t *charPtr = reinterpret_cast<const uint8_t *>(message);
                    while (len--)
                    {
                        printf("%02x ", charPtr[len]);
                    }

                    printf("\n\r");
                );
            }

            /*------------------------------------------------
            Remove the packet that was just read out
            ------------------------------------------------*/
            frameQueue.pop_front();
            IF_SERIAL_DEBUG(printf("%lu: NET Received %s\n\r", radio.millis(), header.toString()););
        }

        return msgLen;
    }

    bool Network::multicast(Header &header, const void *const message, const uint16_t len, const uint8_t level)
    {
        header.data.dstNode = MULTICAST_ADDRESS;
        header.data.srcNode = this->logicalNodeAddress;

        return write(header, message, len, levelToAddress(level));
    }

    bool Network::write(Header &header, const void *const message, const uint16_t len)
    {
        return write(header, message, len, ROUTED_ADDRESS);
    }

    bool Network::write(Header &header, const void *const message, const uint16_t len, const uint16_t writeDirect)
    {
        /*------------------------------------------------
        Protect against invalid inputs
        ------------------------------------------------*/
        if (!initialized)
        {
            oopsies = ErrorType::NOT_INITIALIZED;
            IF_SERIAL_DEBUG(printf("%lu: ERR Not initialized\r\n", radio.millis()););
            return false;
        }

        /*------------------------------------------------
        Allows time for requests (RF24Mesh) to get through between failed writes on busy nodes
        ------------------------------------------------*/
        while (radio.millis() - txTime < 25)
        {
            if (update() > MessageType::MAX_USER_DEFINED_HEADER_TYPE)
            {
                break;
            }
        }

        /*------------------------------------------------
        Do a normal, unfragmented write
        ------------------------------------------------*/
        if (len <= max_frame_payload_size)
        {
            radioPayloadSize = len + FRAME_PREAMBLE_SIZE;
            if (_write(header, message, len, writeDirect))
            {
                return true;
            }
            txTime = radio.millis();
            return false;
        }

        return true;
    }

    bool Network::_write(Header &header, const void *const message, const uint16_t len, const uint16_t directTo)
    {
        /*------------------------------------------------
        Fill out the header
        ------------------------------------------------*/
        header.data.srcNode = logicalNodeAddress;

        /*------------------------------------------------
        Build the full frame to send
        ------------------------------------------------*/
        Frame frame(header.data, len, message);
        memcpy(frameBuffer.begin(), &frame.data, sizeof(Frame_t));
        IF_SERIAL_DEBUG
        (
            printf("%lu: NET Sending Header [%s]\n\r", radio.millis(), header.toString());

            if (len)
            {
                uint16_t tmpLen = len;
                const uint8_t *charPtr = reinterpret_cast<const uint8_t *>(message);

                printf("%lu: NET message ", radio.millis());
                while (tmpLen--)
                {
                    printf("%02x ", charPtr[tmpLen]);
                }
                printf("\n\r");
            }
        );

        if (directTo != ROUTED_ADDRESS)
        {
            /*------------------------------------------------
            Payload is multicast to the first node, and routed normally to the next
            ------------------------------------------------*/
            MessageType sendType = MessageType::USER_TX_TO_LOGICAL_ADDRESS;

            if (header.data.dstNode == MULTICAST_ADDRESS)
            {
                sendType = MessageType::USER_TX_MULTICAST;
            }

            if (header.data.dstNode == directTo)
            {
                sendType = MessageType::USER_TX_TO_PHYSICAL_ADDRESS; // Payload is multicast to the first node, which is the recipient
            }

            return writeDirect(directTo, sendType);
        }

        return writeDirect(header.data.dstNode, MessageType::TX_NORMAL);
    }

    bool Network::writeDirect(uint16_t toNode, MessageType directTo)
    {
        // Direct To: 0 = First Payload, standard routing, 1=routed payload, 2=directRoute to host, 3=directRoute to Route

        bool ok = false;
        bool isAckType = false;

        /*------------------------------------------------
        Check if the payload message type requires an ACK
        ------------------------------------------------*/
        Frame frame(frameBuffer);

        //TODO: Convert these magic numbers into their enum equivalent
        if (frame.data.header.msgType > 64 && frame.data.header.msgType < 192)
        {
            isAckType = true;
        }

        /*------------------------------------------------
        Throw it away if it's not a valid address
        ------------------------------------------------*/
        if (!isValidNetworkAddress(toNode))
        {
            oopsies = ErrorType::INVALID_ADDRESS;
            return false;
        }

        /*------------------------------------------------
        Load info into our conversion structure, and get the converted address info
        ------------------------------------------------*/
        logicalToPhysicalStruct conversion = {toNode, static_cast<uint8_t>(directTo), 0};
        logicalToPhysicalAddress(&conversion);

        /*------------------------------------------------
        Write it
        ------------------------------------------------*/
        IF_SERIAL_DEBUG(printf("%lu: MAC Sending to node [0%o] via node [0%o] on pipe %x\n\r",
            radio.millis(), toNode, conversion.send_node, conversion.send_pipe););

        ok = writeToPipe(conversion.send_node, conversion.send_pipe, conversion.multicast);

        if (!ok)
        {
            IF_SERIAL_DEBUG_ROUTING(printf("%lu: MAC Send fail to 0%o via 0%o on pipe %x\n\r",
                radio.millis(), toNode, conversion.send_node, conversion.send_pipe););
        }

        /*------------------------------------------------

        ------------------------------------------------*/
        if (    ok
            &&  isAckType
            &&  directTo == MessageType::TX_ROUTED
            &&  conversion.send_node == toNode
           )
        {
            auto frame = reinterpret_cast<Frame_t*>(frameBuffer.begin());

            frame->header.msgType = static_cast<uint8_t>(MessageType::NETWORK_ACK);          // Set the payload type to NETWORK_ACK
            frame->header.dstNode = frame->header.srcNode; // Change the 'to' address to the 'from' address

            conversion.send_node = frame->header.srcNode;
            conversion.send_pipe = static_cast<uint8_t>(MessageType::TX_ROUTED);
            conversion.multicast = 0;
            logicalToPhysicalAddress(&conversion);

            // TODO: When you figure out what this section does, re-evaluate radioPayloadSize...likely to be wrong
            // TODO: BUG!
            radioPayloadSize = sizeof(Header_t);
            writeToPipe(conversion.send_node, conversion.send_pipe, conversion.multicast);

            IF_SERIAL_DEBUG_ROUTING(printf("%lu MAC: Route OK to 0%o ACK sent to 0%o\n", radio.millis(), toNode, header->fromNode););
        }

        /*------------------------------------------------

        ------------------------------------------------*/
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

            uint32_t reply_time = radio.millis();

            while (update() != MessageType::NETWORK_ACK)
            {
                if (radio.millis() - reply_time > routeTimeout)
                {
                    IF_SERIAL_DEBUG_ROUTING(printf("%lu: MAC Network ACK fail from 0%o via 0%o on pipe %x\n\r", radio.millis(), toNode, conversion.send_node, conversion.send_pipe););
                    ok = false;
                    break;
                }
            }
        }

        /*------------------------------------------------

        ------------------------------------------------*/
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

        ok = radio.writeFast(frameBuffer.begin(), radioPayloadSize, false);

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
            node_mask_check <<= OCTAL_TO_BIN_BITSHIFT;
            count++;
        }
        multicastLevel = count;

        nodeMask = ~node_mask_check;

        /*------------------------------------------------
        Parent mask is the next level down
        ------------------------------------------------*/
        uint16_t parent_mask = nodeMask >> OCTAL_TO_BIN_BITSHIFT;

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
            i >>= OCTAL_TO_BIN_BITSHIFT;
            m >>= OCTAL_TO_BIN_BITSHIFT;
        }
        parentPipe = i;

        IF_SERIAL_DEBUG_MINIMAL(printf("setup_address node=0%o mask=0%o parent=0%o pipe=0%o\n\r", this->logicalNodeAddress, node_mask, parent_node, parent_pipe););
    }

    void Network::enqueue(Frame &frame)
    {
        if (!frameQueue.full())
        {
            frameQueue.push_back(frame.data);
        }
        else
        {
            IF_SERIAL_DEBUG(printf("%lu: NET **Drop Payload** Buffer Full\r\n", radio.millis()););
        }
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
        uint8_t nodeID = 0;
        uint16_t n = node;

        /*------------------------------------------------
        If this isn't one of our special (invalid) node destinations, manually
        check each level of the octal address.
        ------------------------------------------------*/
        if (!(node == MULTICAST_ADDRESS) && !(node == ROUTED_ADDRESS))
        {
            while (n)
            {
                /*------------------------------------------------
                Grab the node id of the LSB octal number
                ------------------------------------------------*/
                nodeID = n & OCTAL_MASK;

                /*------------------------------------------------
                Test for the proper node id range
                ------------------------------------------------*/
                if ((nodeID < MIN_NODE_ID) || (nodeID > MAX_NODE_ID))
                {
                    result = false;
                    IF_SERIAL_DEBUG(printf("*** WARNING *** Invalid address 0%o\n\r", n););
                    break;
                }

                /*------------------------------------------------
                Push the next octal number to the LSB
                ------------------------------------------------*/
                n >>= OCTAL_TO_BIN_BITSHIFT;
            }
        }

        return result;
    }

    bool Network::setAddress(const uint16_t address)
    {
        if(isValidNetworkAddress(address))
        {
            bool initialized = true;
            this->logicalNodeAddress = address;

            /*------------------------------------------------
            Make sure we can't receive any data
            ------------------------------------------------*/
            radio.stopListening();

            /*------------------------------------------------
            Close all the previously opened pipes
            ------------------------------------------------*/
            for (uint8_t i = 0; i < MAX_NUM_PIPES; i++)
            {
                radio.closeReadPipe(i);
            }

            /*------------------------------------------------
            Re-setup the address helper cache
            ------------------------------------------------*/
            setupAddress();

            /*------------------------------------------------
            Open all the listening pipes
            ------------------------------------------------*/
            for (uint8_t i = 0; i < MAX_NUM_PIPES; i++)
            {
                initialized &= radio.openReadPipe(i, pipeAddress(address, i), true);
            }
            radio.startListening();

            return initialized;
        }
        else
        {
            oopsies = ErrorType::INVALID_ADDRESS;
            return false;
        }
    }

    uint16_t Network::getLogicalAddress()
    {
        return this->logicalNodeAddress;
    }

    void Network::setMulticastLevel(uint8_t level)
    {
        multicastLevel = level;
        radio.openReadPipe(0, pipeAddress(levelToAddress(level), 0), true);
    }

    uint16_t Network::levelToAddress(uint8_t level)
    {
        uint16_t levelAddr = 1;
        if (level)
        {
            levelAddr = levelAddr << ((level - 1) * OCTAL_TO_BIN_BITSHIFT);
        }
        else
        {
            return 0;
        }
        return levelAddr;
    }

    uint64_t Network::pipeAddress(const uint16_t nodeID, const uint8_t pipeNum)
    {
        /*------------------------------------------------
        These bytes take on many uses. They can represent pipe numbers, device numbers,
        parent/child numbers, etc. They are really just a way to identify where a device
        is inside the network. As needed, a byte can be used to compose an address.
        ------------------------------------------------*/
        static uint8_t addressBytePool[] = { 0xec, 0xc3, 0x3c, 0x33, 0xce, 0x3e };

        /*------------------------------------------------
        The base, 5 byte address we start off with. 0xCC is reserved to indicate
        the master node.
        ------------------------------------------------*/
        uint64_t result = 0xCCCCCCCCCC;
        uint8_t *address = reinterpret_cast<uint8_t *>(&result);

        /*------------------------------------------------
        If we want the master node (00), calculating the pipe address is easy. Simply
        replace the first bytes of the base address with the appropriate addressByte[].
        ------------------------------------------------*/
        if (!nodeID)
        {
            address[0] = addressBytePool[pipeNum];
        }
        else
        {
            Node node;
            node.fromOctal(nodeID);

            /*------------------------------------------------
            Calculate what byte goes at each level of the address scheme. The first byte
            will always be associated with the pipe number. The remaining bytes are determined
            like so (MSB -> LSB):

            Level 0: (Direct child of master)
                [ 0xCC, 0xCC, 0xCC, childID, pipeNum ]

            Level 1: (Has a parent, ie 011: 1st child of the 1st level 0 node)
                [ 0xCC, 0xCC, childID, parentID, pipeNum ]

            Level 2: (Has a parent and grandparent)
                [ 0xCC, childID, parentID, grandParentID, pipeNum ]

            Level 3: (You get the idea)
                [ childID, parentID, grandParentID, greatGrandParentID, pipeNum ]

            Each level is limited to the range indicated by MIN_NODE_ID and MAX_NODE_ID.
            ------------------------------------------------*/
            address[0] = addressBytePool[pipeNum];

            for (uint8_t i = 1; i < NRF24L::MAX_ADDRESS_WIDTH; i++)
            {
                if (node.greatGrandParentID_isValid())
                {
                    address[i] = addressBytePool[node.greatGrandParentID];
                    node.greatGrandParentID = INVALID_NODE_ID;
                    continue;
                }
                else if (node.grandParentID_isValid())
                {
                    address[i] = addressBytePool[node.grandParentID];
                    node.grandParentID = INVALID_NODE_ID;
                    continue;
                }
                else if (node.parentID_isValid())
                {
                    address[i] = addressBytePool[node.parentID];
                    node.parentID = INVALID_NODE_ID;
                    continue;
                }
                else if (node.childID_isValid())
                {
                    address[i] = addressBytePool[node.childID];
                    node.childID = INVALID_NODE_ID;
                    continue;
                }
                else
                {
                    break;
                }
            }
        }

        IF_SERIAL_DEBUG(
            uint32_t *top = reinterpret_cast<uint32_t *>(address + 1);
            printf("%lu: NET Pipe %i on node 0%o has address 0x%05x\n\r", radio.millis(), pipeNum, nodeID, result););

        return result;
    }

} /* !NRF24Network */
