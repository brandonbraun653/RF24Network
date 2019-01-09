/********************************************************************************
*   RF24Network.hpp
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

#pragma once
#ifndef RF24NETWORK_HPP
#define RF24NETWORK_HPP

/* C++ Includes */
#include <cstddef>
#include <cstdint>

/* NRF24L01 Includes */
#include "nrf24l01.hpp"

/* Network Includes */
#include "RF24NetworkDefinitions.hpp"

namespace RF24Network
{
    /**
    *   Header which is sent with each message. The frame put over the air consists of this header
    *   + message. Each are addressed to the appropriate node, and the network forwards them on to
    *   their final destination.
    */
    class Header
    {
    public:
        /**
        *   Send constructor
        *
        *   @param[in]  to      The Octal format, logical node address where the message is going
        *   @param[in]  type    The type of message
        */
        Header(uint16_t to, MessageType type = MessageType::TX_NORMAL)
        {
            this->toNode = to;
            this->type = type;
        }

        Header() = default;
        ~Header() = default;

        /**
        *   Convert the header data into a string. Uses internal memory to create the string, which will
        *   be overridden on the next call.
        *
        *   @return String representation of this object
        */
        const char *toString() const;

        uint16_t id;       /**< Sequential message ID, incremented every time a new frame is constructed */
        uint16_t toNode;   /**< Logical address where the message is going */
        uint16_t fromNode; /**< Logical address where the message was generated */
        MessageType type;  /**< Message type for the header */

        /**
        *   During fragmentation, it carries the fragment_id, and on the last fragment
        *   it carries the header_type.
        */
        uint8_t reserved; /**< Reserved for system use */

        /*------------------------------------------------
        Number of bytes in the header. Currently should be 8.
        ------------------------------------------------*/
        static constexpr uint8_t SIZE = sizeof(id) + sizeof(toNode) + sizeof(fromNode) + sizeof(type) + sizeof(reserved);
    };

    /**
    *   Frame structure for internal message handling, and for use by external applications
    *
    *   The actual frame put over the air consists of a header (8-bytes) and a message payload
    *   (Up to 24-bytes). When data is received, it is stored using the Frame structure,
    *   which includes:
    *       1. The header
    *       2. The size of the included message
    *       3. The data being received
    */
    class Frame
    {
    public:
        Frame() = default;
        ~Frame() = default;

        /**
        *   Constructor - create a network frame with data
        *   Frames are constructed and handled differently on Arduino/AVR and Linux devices (defined RF24_LINUX)
        *
        *   @param header The RF24Network header to be stored in the frame
        *   @param message_size The size of the 'message' or data
        */
        Frame(Header &header, uint16_t messageSize)
        {
            this->header = header;
            this->messageSize = messageSize;
        }

        Header header;          /**< Header which is sent with each message */
        uint16_t messageSize;   /**< The size in bytes of the payload length */
        uint8_t *messageBuffer; /**< Pointer to the buffer storing the actual message */

        /**
        *   Number of bytes contained in a frame of data
        *   Preamble: Header (8-bytes) + Frame_Size (2-bytes)
        */
        static constexpr uint8_t PREAMBLE_SIZE_Byte = 10;
        static constexpr uint8_t PREAMBLE_FIELD_HEADER_SIZE_Byte = Header::SIZE;
        static constexpr uint8_t PREAMBLE_FIELD_PAYLOAD_SIZE_Byte = 2;
    };

    /**
    *   The size of the main buffer. This is the user-cache, where incoming data is stored.
    *   Data is stored using Frames: Preamble + Data (?-bytes)
    *
    *   @note The MAX_PAYLOAD_SIZE is (MAIN_BUFFER_SIZE - 10), and the result must be divisible by 24.
    */
    constexpr uint16_t MAIN_BUFFER_SIZE = 144 + Frame::PREAMBLE_SIZE_Byte;

    /**
    *   Maximum size of fragmented network frames and fragmentation cache. This MUST BE divisible by 24.
    *   @note: Must be a multiple of 24.
    *   @note: If used with RF24Ethernet, this value is used to set the buffer sizes.
    */
    constexpr uint16_t MAX_PAYLOAD_SIZE = MAIN_BUFFER_SIZE - Frame::PREAMBLE_SIZE_Byte;
    static_assert((MAX_PAYLOAD_SIZE % 24u) == 0, "The max payload size must be divisible by 24!");

    /**
    *   Business logic for handling network communications
    */
    class Network
    {
    public:
        /**
        *   Construct the network
        *
        *   @param[in]  radio   The underlying radio driver instance
        */
        Network(NRF24L::NRF24L01 &radio);

        /**
        *   Initializes the network and configures the address, which designates the location
        *   of the node within RF24Network topology.
        *
        *   @note Node addresses are specified in Octal format
        *
        *   @warning Be sure to 'begin' the radio first.
        *
        *   **Example 1:** Begin on channel 90 with address 0 (master node)
        *       network.begin(90,0);
        *
        *   **Example 2:** Begin on channel 90 with address 01 (child of master)
        *       network.begin(90,01);
        *
        *   **Example 3:** Begin on channel 90 with address 011 (child of 01, grandchild of master)
        *       network.begin(90,011);
        *
        *   @param[in]  channel         The radio channel to operate on
        *   @param[in]  node_address    The logical address of this node
        *   @return True if the setup was successful, false if not
        */
        bool begin(const uint8_t channel, const uint16_t nodeAddress,
                   const NRF24L::PowerAmplitude pwr = NRF24L::PowerAmplitude::MAX);

        /**
        *   Updates the internal network processing stack. This function must be called regularly to
        *   keep the network layer going.  This is where payloads are re-routed, received, and all the
        *   action happens.
        *
        *   @return Returns the type of the last received payload.
        */
        MessageType update();

        /**
        *   Check whether a message available for this node.
        *
        *   @return True if a message is available, False if not
        */
        bool available() const;

        /**
        *   Reads the next available header without advancing to the next incoming message.  Useful
        *   for doing a switch on the message type. If there is no message available, the header is
        *   not touched.
        *
        *   @param[out] header      The header of the next message
        *   @return TODO?
        */
        uint16_t peek(Header &header);

        /**
        *   Read the next available payload
        *
        *   Reads the next available payload without advancing to the next
        *   incoming message.  Useful for doing a transparent packet
        *   manipulation layer on top of RF24Network.
        *
        *   @param[out] header      The header (envelope) of this message
        *   @param[out] message     Pointer to memory where the message should be placed
        *   @param maxlen Amount of bytes to copy to message.
        *   @return void
        */
        void peek(Header &header, void *const message, const uint16_t maxlen);

        /**
        *   Read a message
        *
        *   @param[out] header      The header (envelope) of this message
        *   @param[out] message     Pointer to memory where the message should be placed
        *   @param[in]  maxlen      The largest message size which can be held in message
        *   @return The total number of bytes copied into message
        */
        uint16_t read(Header &header, void *const message, const uint16_t maxlen);

        /**
        *   Send a message
        *
        *   @note RF24Network now supports fragmentation for very long messages, send as normal.
        *   Default max payload size is 120 bytes.
        *
        *   @param[in,out] header   The header (envelope) of this message.  The critical
        *                           thing to fill in is the @p to_node field so we know where to send the
        *                           message.  It is then updated with the details of the actual header sent.
        *
        *   @param[in]  message     Pointer to memory where the message is located
        *   @param[in]  len         The size of the message
        *   @return True if the message sent successfully, false if not
        */
        bool write(Header &header, const void *message, const uint16_t len);

        /**
        *   Writes a direct (unicast) payload. This allows routing or sending messages outside of the
        *   usual routing paths. The same as write, but a physical address is specified as the last option.
        *   The payload will be written to the physical address, and routed as necessary by the recipient
        *
        *   @param[in]  header      TODO
        *   @param[in]  message     TODO
        *   @param[in]  length      TODO
        *   @param[in]  writeDirect TODO
        *   @return True if the message sent successfully, false if not
        */
        bool write(Header &header, const void *message, uint16_t length, uint16_t writeDirect);

        /**
        *   Allows messages to be rapidly broadcast through the network by seding to multiple nodes at once
        *
        *   Multicasting is arranged in levels, with all nodes on the same level listening to the same address
        *   Levels are assigned by network level ie: nodes 01-05: Level 1, nodes 011-055: Level 2
        *   @see multicastLevel
        *   @see multicastRelay
        *
        *   @param[in] header       TODO
        *   @param[in] message      Pointer to memory where the message is located
        *   @param[in] len          The size of the message
        *   @param[in] level        Multicast level to broadcast to
        *   @return True if the message sent successfully, false if not
        */
        bool multicast(Header &header, const void *message, const uint16_t length, const uint8_t level);

        /**
        *   By default, multicast addresses are divided into levels.
        *
        *   Nodes 1-5 share a multicast address, nodes n1-n5 share a multicast address, and nodes n11-n55 share a multicast address.<br>
        *
        *   This option is used to override the defaults, and create custom multicast groups that all share a single
        *   address. The level should be specified in decimal format 1-6 <br>
        *   @see multicastRelay

        *   @param[in]  level       Levels 1 to 6 are available. All nodes at the same level will receive the same
        *                           messages if in range. Messages will be routed in order of level, low to high by default, with the
        *                           master node (00) at multicast Level 0
        *   @return void
        */
        void setMulticastLevel(const uint8_t level);

        /**
        *   Return the number of failures and successes for all transmitted payloads, routed or sent directly
        *   @note This needs to be enabled via #define ENABLE_NETWORK_STATS in RF24Network_config.h
        *
        *   @param[out] fails       Number of failed transmissions
        *   @param[out] ok          Number of successful transmissions???? TODO
        *   @return void
        */
        void failures(uint32_t &fails, uint32_t &ok);

        /**
        *   Check if a network address is valid or not
        *   @note Addresses are specified in octal: 011, 034
        *
        *   @return True if a supplied address is valid
        */
        bool isValidNetworkAddress(const uint16_t node);

        /**
        *   This node's parent address
        *
        *   @return This node's parent address, or -1 if this is the base
        */
        uint16_t parent() const;

        /**
        *   Enabling this will allow this node to automatically forward received multicast frames to the next highest
        *   multicast level. Duplicate frames are filtered out, so multiple forwarding nodes at the same level should
        *   not interfere. Forwarded payloads will also be received.
        *   @see multicastLevel
        */
        bool multicastRelay;

        /**
        *   @note: This value is automatically assigned based on the node address
        *   to reduce errors and increase throughput of the network.
        *
        *   Sets the timeout period for individual payloads in milliseconds at staggered intervals.
        *   Payloads will be retried automatically until success or timeout
        *   Set to 0 to use the normal auto retry period defined by radio.setRetries()
        */
        uint32_t txTimeout; /**< Network timeout value */

        /**
        *   This only affects payloads that are routed by one or more nodes.
        *   This specifies how long to wait for an ack from across the network.
        *   Radios sending directly to their parent or children nodes do not
        *   utilize this value.
        */
        uint16_t routeTimeout; /**< Timeout for routed payloads */

        /**
        *   Provided a node address and a pipe number, will return the RF24Network address of that child pipe for that node
        */
        uint16_t addressOfPipe(uint16_t node, uint8_t pipeNo);

        /**
        * The raw system frame buffer of received data.
        */
        uint8_t frameBuffer[MAX_FRAME_SIZE];

        /**
        *   The frag_ptr is only used with Arduino (not RPi/Linux) and is mainly used for external data systems like RF24Ethernet. When
        *   an EXTERNAL_DATA payload type is received, and returned from network.update(), the frag_ptr will always point to the starting
        *   memory location of the received frame. <br>This is used by external data systems (RF24Ethernet) to immediately copy the received
        *   data to a buffer, without using the user-cache.
        */
        Frame *frag_ptr;

        /**
        * Variable to determine whether update() will return after the radio buffers have been emptied (DEFAULT), or
        * whether to return immediately when (most) system types are received.
        *
        * As an example, this is used with RF24Mesh to catch and handle system messages without loading them into the user cache.
        *
        * The following reserved/system message types are handled automatically, and not returned.
        *
        * | System Message Types (Not Returned) |
        * |-----------------------|
        * | NETWORK_ADDR_RESPONSE |
        * | NETWORK_ACK           |
        * | NETWORK_PING          |
        * | NETWORK_POLL <br>(With multicast enabled) |
        * | NETWORK_REQ_ADDRESS   |
        *
        */
        bool returnSysMsgs;

        /**
        *   Network Flags allow control of data flow
        *
        *   Incoming Blocking: If the network user-cache is full, lets radio cache fill up. Radio ACKs are not sent when radio internal cache is full.<br>
        *   This behaviour may seem to result in more failed sends, but the payloads would have otherwise been dropped due to the cache being full.<br>
        *
        *   |       FLAGS       |   Value  | Description                                                                                                |
        *   |-------------------|----------|------------------------------------------------------------------------------------------------------------|
        *   |FLAG_HOLD_INCOMING | 1(bit_1) | INTERNAL: Set automatically when a fragmented payload will exceed the available cache
        *   |FLAG_BYPASS_HOLDS  | 2(bit_2) | EXTERNAL: Can be used to prevent holds from blocking. Note: Holds are disabled & re-enabled by RF24Mesh
        *   |                   |          |           when renewing addresses. This will cause data loss if incoming data exceeds the available cache space.
        *   |FLAG_FAST_FRAG     | 4(bit_3) | INTERNAL: Replaces the fastFragTransfer variable, and allows for faster transfers between directly connected nodes.
        *   |FLAG_NO_POLL       | 8(bit_4) | EXTERNAL/USER: Disables NETWORK_POLL responses on a node-by-node basis.
        *
        */
        uint8_t networkFlags;

    private:
        ErrorType oopsies = ErrorType::NO_ERROR;

        bool initialized = false;

        uint32_t txTime;
        uint8_t frameSize;
        uint16_t logicalNodeAddress; /**< Logical node address of this unit, 1 .. UINT_MAX */

        bool writeDirect(uint16_t toNode, MessageType directTo);

        bool writeToPipe(uint16_t node, uint8_t pipe, bool multicast);


        uint8_t enqueue(Header *header);

        bool isDirectChild(uint16_t node);
        bool isDescendant(uint16_t node);

        uint16_t directChildRouteTo(uint16_t node);
        void setupAddress(void);
        bool _write(Header &header, const void *message, uint16_t len, uint16_t directTo);

        struct logicalToPhysicalStruct
        {
            uint16_t send_node;
            uint8_t send_pipe;
            bool multicast;
        };

        bool logicalToPhysicalAddress(logicalToPhysicalStruct *conversionInfo);

        uint16_t levelToAddress(uint8_t level);

        uint64_t pipeAddress(uint16_t node, uint8_t pipe);

        NRF24L::NRF24L01 &radio; /**< Underlying radio driver, provides link/physical layers */

        uint8_t multicastLevel;

        uint8_t frameQueue[MAIN_BUFFER_SIZE]; /**< Space for a small set of frames that need to be delivered to the app layer */

        uint8_t *nextFrame;    /**< Pointer into the frame_queue where we should place the next received frame */

        Frame fragQueue;
        uint8_t fragQueueMessageBuffer[MAX_PAYLOAD_SIZE]; //frame size + 1

        uint16_t parentNode; /**< Our parent's node address */
        uint8_t parentPipe;  /**< The pipe our parent uses to listen to us */
        uint16_t nodeMask;   /**< The bits which contain signfificant node address information */
    };
}

#endif /* ! RF24_NETWORK_HPP */
