/********************************************************************************
*   RF24NetworkDefinitions.hpp
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

namespace RF24Network
{

    // ACK Response Types
    /**
     * **Reserved network message types**
     *
     * The network will determine whether to automatically acknowledge payloads based on their general type <br>
     *
     * **User types** (1-127) 1-64 will NOT be acknowledged <br>
     * **System types** (128-255) 192 through 255 will NOT be acknowledged<br>
     *
     * @defgroup DEFINED_TYPES Reserved System Message Types
     *
     * System types can also contain message data.
     *
     * @{
     */


     //#define NETWORK_ADDR_CONFIRM 129


     /** @} */

     #define NETWORK_MORE_FRAGMENTS_NACK 200

    enum class MessageTypes : uint8_t
    {
        MIN_USER_DEFINED_HEADER_TYPE = 0,

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

        /**
* Messages of type NETWORK_PING will be dropped automatically by the recipient. A NETWORK_ACK or automatic radio-ack will indicate to the sender whether the
* payload was successful. The time it takes to successfully send a NETWORK_PING is the round-trip-time.
*/
#define NETWORK_PING 130

/**
 * External data types are used to define messages that will be passed to an external data system. This allows RF24Network to route and pass any type of data, such
 * as TCP/IP frames, while still being able to utilize standard RF24Network messages etc.
 *
 * **Linux**
 * Linux devices (defined RF24_LINUX) will buffer all data types in the user cache.
 *
 * **Arduino/AVR/Etc:** Data transmitted with the type set to EXTERNAL_DATA_TYPE will not be loaded into the user cache. <br>
 * External systems can extract external data using the following process, while internal data types are cached in the user buffer, and accessed using network.read() :
 * @code
 * uint8_t return_type = network.update();
 * if(return_type == EXTERNAL_DATA_TYPE){
 *     uint16_t size = network.frag_ptr->message_size;
 *     memcpy(&myDataBuffer,network.frag_ptr->message_buffer,network.frag_ptr->message_size);
 * }
 * @endcode
 */
#define EXTERNAL_DATA_TYPE 131

/**
 * Messages of this type designate the first of two or more message fragments, and will be re-assembled automatically.
 */
#define NETWORK_FIRST_FRAGMENT 148

/**
 * Messages of this type indicate a fragmented payload with two or more message fragments.
 */
#define NETWORK_MORE_FRAGMENTS 149

/**
 * Messages of this type indicate the last fragment in a sequence of message fragments.
 * Messages of this type do not receive a NETWORK_ACK
 */
#define NETWORK_LAST_FRAGMENT 150
//#define NETWORK_LAST_FRAGMENT 201

// NO ACK Response Types
//#define NETWORK_ACK_REQUEST 192

/**
 * Messages of this type are used internally, to signal the sender that a transmission has been completed.
 * RF24Network does not directly have a built-in transport layer protocol, so message delivery is not 100% guaranteed.<br>
 * Messages can be lost via corrupted dynamic payloads, or a NETWORK_ACK can fail, while the message was actually successful.
 *
 * NETWORK_ACK messages can be utilized as a traffic/flow control mechanism, since transmitting nodes will be forced to wait until
 * the payload is transmitted across the network and acknowledged, before sending additional data.
 *
 * In the event that the transmitting device will be waiting for a direct response, manually sent by the recipient, a NETWORK_ACK is not required. <br>
 * User messages utilizing a 'type' with a decimal value of 64 or less will not be acknowledged across the network via NETWORK_ACK messages.
 */
#define NETWORK_ACK 193

/**
 * Used by RF24Mesh
 *
 * Messages of this type are used with multi-casting , to find active/available nodes.
 * Any node receiving a NETWORK_POLL sent to a multicast address will respond directly to the sender with a blank message, indicating the
 * address of the available node via the header.
 */
#define NETWORK_POLL 194

/**
 * Used by RF24Mesh
 *
 * Messages of this type are used to request information from the master node, generally via a unicast (direct) write.
 * Any (non-master) node receiving a message of this type will manually forward it to the master node using an normal network write.
 */
#define NETWORK_REQ_ADDRESS 195
//#define NETWORK_ADDR_LOOKUP 196
//#define NETWORK_ADDR_RELEASE 197
    };
}

#define NETWORK_DEFAULT_ADDRESS 04444

#define RF24NetworkMulticast

#define DISABLE_FRAGMENTATION

/** System defines */

/** The size of the main buffer. This is the user-cache, where incoming data is stored.
* Data is stored using Frames: Header (8-bytes) + Frame_Size (2-bytes) + Data (?-bytes)
*
* @note The MAX_PAYLOAD_SIZE is (MAIN_BUFFER_SIZE - 10), and the result must be divisible by 24.
*/
#define MAIN_BUFFER_SIZE 144 + 10

/** Maximum size of fragmented network frames and fragmentation cache. This MUST BE divisible by 24.
* @note: Must be a multiple of 24.
* @note: If used with RF24Ethernet, this value is used to set the buffer sizes.
*/
#define MAX_PAYLOAD_SIZE  MAIN_BUFFER_SIZE-10

/** Disable user payloads. Saves memory when used with RF24Ethernet or software that uses external data.*/
//#define DISABLE_USER_PAYLOADS

/** Enable tracking of success and failures for all transmissions, routed and user initiated */
//#define ENABLE_NETWORK_STATS

/** Enable dynamic payloads - If using different types of NRF24L01 modules, some may be incompatible when using this feature **/
//#define ENABLE_DYNAMIC_PAYLOADS

/** Debug Options */
//#define SERIAL_DEBUG
//#define SERIAL_DEBUG_MINIMAL
//#define SERIAL_DEBUG_ROUTING
//#define SERIAL_DEBUG_FRAGMENTATION
//#define SERIAL_DEBUG_FRAGMENTATION_L2

    #if !defined (ARDUINO_ARCH_AVR)
    #define sprintf_P sprintf
    #define printf_P printf
    #endif

    #if defined (SERIAL_DEBUG_MINIMAL)
      #define IF_SERIAL_DEBUG_MINIMAL(x) ({x;})
    #else
      #define IF_SERIAL_DEBUG_MINIMAL(x)
    #endif

    #if defined (SERIAL_DEBUG_FRAGMENTATION)
      #define IF_SERIAL_DEBUG_FRAGMENTATION(x) ({x;})
    #else
      #define IF_SERIAL_DEBUG_FRAGMENTATION(x)
    #endif

    #if defined (SERIAL_DEBUG_FRAGMENTATION_L2)
      #define IF_SERIAL_DEBUG_FRAGMENTATION_L2(x) ({x;})
    #else
      #define IF_SERIAL_DEBUG_FRAGMENTATION_L2(x)
    #endif

    #if defined (SERIAL_DEBUG_ROUTING)
      #define IF_SERIAL_DEBUG_ROUTING(x) ({x;})
    #else
      #define IF_SERIAL_DEBUG_ROUTING(x)
    #endif


#endif /* !RF24NETWORKDEFINITIONS_HPP */
