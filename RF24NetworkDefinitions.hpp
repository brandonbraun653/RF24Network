/********************************************************************************
*   File Name:
*       RF24NetworkDefinitions.hpp
*
*   Description:
*       Provides definitions for the network data types and allows the user to
*       configure the runtime behavior and properties of the system as a whole.
*
*   2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

#pragma once
#ifndef RF24NETWORKDEFINITIONS_HPP
#define RF24NETWORKDEFINITIONS_HPP

/* C++ Includes */
#include <cstdint>

/* NRF24L01 Driver Includes */
#include "nrf24l01Definitions.hpp"

namespace RF24Network
{
    constexpr uint8_t MAX_FRAME_SIZE = NRF24L::MAX_PAYLOAD_WIDTH;
    constexpr uint8_t MAX_HEADER_SIZE = 8;

    /**
    *   The default network address
    */
    constexpr uint16_t DEFAULT_LOGICAL_ADDRESS = 04444;   /**< (OCTAL) Default value for new nodes */
    constexpr uint16_t EMPTY_LOGICAL_ADDRESS = 07777;     /**< (OCTAL) Value physically impossible for node to own due to child limits */
    constexpr uint8_t OCTAL_TO_BIN_BITSHIFT = 3u;
    constexpr uint8_t MIN_NODE_ID = 0;
    constexpr uint8_t MAX_NODE_ID = 5;

    /*------------------------------------------------
    Config Options
    ------------------------------------------------*/
    /**
    *   Disable user payloads. Saves memory when used with RF24Ethernet or software that uses external data.
    */
    #define RF24Network_DISABLE_USER_PAYLOADS (true)

    /**
    *   Enable tracking of success and failures for all transmissions, routed and user initiated
    */
    #define RF24Network_ENABLE_NETWORK_STATS (false)

    /**
    *   Enable dynamic payloads - If using different types of NRF24L01 modules, some may be incompatible when using this feature
    */
    #define RF24Network_ENABLE_DYNAMIC_PAYLOADS (false)

    /**
    *   TODO
    */
    #define RF24Network_MULTICAST (true)

    /**
    *   TODO
    */
    #define RF24Network_DISABLE_FRAGMENTATION (true)

    /*------------------------------------------------
    Debug Options
    ------------------------------------------------*/
    #define SERIAL_DEBUG
    //#define SERIAL_DEBUG_MINIMAL
    //#define SERIAL_DEBUG_ROUTING
    //#define SERIAL_DEBUG_FRAGMENTATION
    //#define SERIAL_DEBUG_FRAGMENTATION_L2

#if defined(SERIAL_DEBUG)
#define IF_SERIAL_DEBUG(x) ({ x; })
#else
#define IF_SERIAL_DEBUG(x)
#endif

#if defined(SERIAL_DEBUG_MINIMAL)
#define IF_SERIAL_DEBUG_MINIMAL(x) ({ x; })
#else
#define IF_SERIAL_DEBUG_MINIMAL(x)
#endif

#if defined(SERIAL_DEBUG_FRAGMENTATION)
#define IF_SERIAL_DEBUG_FRAGMENTATION(x) ({ x; })
#else
#define IF_SERIAL_DEBUG_FRAGMENTATION(x)
#endif

#if defined(SERIAL_DEBUG_FRAGMENTATION_L2)
#define IF_SERIAL_DEBUG_FRAGMENTATION_L2(x) ({ x; })
#else
#define IF_SERIAL_DEBUG_FRAGMENTATION_L2(x)
#endif

#if defined(SERIAL_DEBUG_ROUTING)
#define IF_SERIAL_DEBUG_ROUTING(x) ({ x; })
#else
#define IF_SERIAL_DEBUG_ROUTING(x)
#endif

    constexpr uint16_t MULTICAST_NODE = 0100;

    /** System Network Message Types
    *
    *   The network will determine whether to automatically acknowledge payloads based on their general type:
    *   USER TYPES [1-127]:     Numbers [1-64] will have NOACK, [65-127] will have ACK
    *   SYSTEM TYPES [128-255]: Numbers [192-255] will have NOACK, [128-191] will have ACK
    */
    enum class MessageType : uint8_t
    {
        MIN_USER_DEFINED_HEADER_TYPE = 0,

        TX_NORMAL = 0,
        TX_ROUTED = 1,
        USER_TX_TO_PHYSICAL_ADDRESS = 2,
        USER_TX_TO_LOGICAL_ADDRESS = 3,
        USER_TX_MULTICAST = 4,

        M = 7,

        MAX_USER_DEFINED_HEADER_TYPE = 127,

        /**
        *   A NETWORK_ADDR_RESPONSE type is utilized to manually route custom messages containing
        *   a single RF24Network address.
        *
        *   Used by RF24Mesh
        *
        *   If a node receives a message of this type that is directly addressed to it, it will
        *   read the included message, and forward the payload on to the proper recipient. This
        *   allows nodes to forward multicast messages to the master node, receive a response,
        *   and forward it back to the requester.
        */
        NETWORK_ADDR_RESPONSE = 128,

        MESH_ADDR_CONFIRM = 129,

        /**
        *   Messages of type NETWORK_PING will be dropped automatically by the recipient. A NETWORK_ACK
        *   or automatic radio-ack will indicate to the sender whether the payload was successful. The
        *   time it takes to successfully send a NETWORK_PING is the round-trip-time.
        */
        NETWORK_PING = 130,

        /**
        *   External data types are used to define messages that will be passed to an external
        *   data system. This allows RF24Network to route and pass any type of data, such
        *   as TCP/IP frames, while still being able to utilize standard RF24Network messages etc.
        */
        EXTERNAL_DATA_TYPE = 131,

        /**
        *   Messages of this type designate the first of two or more message fragments, and will be
        *   re-assembled automatically.
        */
        NETWORK_FIRST_FRAGMENT = 148,

        /**
        *   Messages of this type indicate a fragmented payload with two or more message fragments.
        */
        NETWORK_MORE_FRAGMENTS = 149,

        /**
        *   Messages of this type indicate the last fragment in a sequence of message fragments.
        */
        NETWORK_LAST_FRAGMENT = 150,

        /**
        *   Signal that an error of some kind occurred.
        */
        NETWORK_ERR = 192,

        /**
        *   Messages of this type are used internally, to signal the sender that a transmission has been completed.
        *   RF24Network does not directly have a built-in transport layer protocol, so message delivery is not 100%
        *   guaranteed. Messages can be lost via corrupted dynamic payloads, or a NETWORK_ACK can fail, while the
        *   message was actually successful.
        *
        *   NETWORK_ACK messages can be utilized as a traffic/flow control mechanism, since transmitting nodes
        *   will be forced to wait until the payload is transmitted across the network and acknowledged, before
        *   sending additional data.
        *
        *   In the event that the transmitting device will be waiting for a direct response, manually sent by the
        *   recipient, a NETWORK_ACK is not required. User messages utilizing a 'type' with a decimal value of 64
        *   or less will not be acknowledged across the network via NETWORK_ACK messages.
        */
        NETWORK_ACK = 193,

        /**
        *   Used by RF24Mesh
        *
        *   Messages of this type are used with multi-casting , to find active/available nodes.
        *   Any node receiving a NETWORK_POLL sent to a multicast address will respond directly to the sender with
        *   a blank message, indicating the address of the available node via the header.
        */
        NETWORK_POLL = 194,

        /**
        *   Used by RF24Mesh
        *
        *   Messages of this type are used to request information from the master node, generally via a unicast (direct)
        *   write. Any (non-master) node receiving a message of this type will manually forward it to the master node
        *   using an normal network write.
        */
        NETWORK_REQ_ADDRESS = 195,

        MESH_ADDR_LOOKUP = 196,
        MESH_ADDR_RELEASE = 197,
        MESH_ID_LOOKUP = 198,

        /**
        *   Not sure what this one does yet.
        */
        NETWORK_MORE_FRAGMENTS_NACK = 200,

        /**
        *   Use the current channel when setting up the network
        */
        USE_CURRENT_CHANNEL = 255
    };

    /**
    *   Prevents reading additional data from the radio when buffers are full
    */
    enum class FlagType : uint8_t
    {
        HOLD_INCOMING = (1u << 0),

        /**
        *   FLAG_BYPASS_HOLDS is mainly for use with RF24Mesh as follows:
        *       a: Ensure no data in radio buffers, else exit
        *       b: Address is changed to multicast address for renewal
        *       c: Holds Cleared (bypass flag is set)
        *       d: Address renewal takes place and is set
        *       e: Holds Enabled (bypass flag off)
        */
        BYPASS_HOLDS = (1u << 1),
        FAST_FRAG = (1u << 2),
        NO_POLL = (1u << 3),
    };

    /**
    *   Various ways we can fail
    */
    enum class ErrorType : uint8_t
    {
        NO_ERROR = 0,
        OK = NO_ERROR,
        NOT_INITIALIZED,
        INVALID_ADDRESS,
        INVALID_INPUTS,
        RADIO_NOT_CONNECTED,
        RADIO_PRE_INITIALIZED,
        RADIO_FAILED_INIT,

    };
}

#endif /* !RF24NETWORKDEFINITIONS_HPP */
