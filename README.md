# gnFellerSwitch
Arduino library for generic master/slave communications.
Currently only RS485 as a hardare-layer is implemented. But its easy to implement additional layers like RS232 or others. You can use a shared medium or point2point communication.
Its a message-based protocol, sending frames between master and slave.

Please consult the [examples](./examples), [config.h](./src/config.h) and the [source-code](./src) for additional informations.

You are welcome to contribute. Add some Documentation, post Issues or (preffered) Pull requests. Contact me if you plan a major-change.

# Roles
## Master
Each environment requires one master. It controls the bus and request its slaves to send data. A master can talk to everyone in the environment.

## Slave
A environment can have one or many slaves. Each slave is independent, but requires a master. A slave is just able to talk with the master.

# Protocol
## Frame
| Byte | Type/Value | Description |
| --- | --- | --- |
| 00<br/>01 | `0xAA`<br/>`0x55` | __StartBytes__<br/>`1010 1010` // `0101 0101` |
| 02 | Flagbyte | See below |
| 03 | Addressbyte | See below |
| 04 | Byte | **PayloadSize**<br/>Payload-Byte count (Byte 07 till ..., without CRC16 |
| 05 | Byte | Service |
| 06 | Byte | Subservice |
| 07... | Bytes | Payload |
| 08 + ...<br/>09 + ... | UInt16 | **CRC16**<br/>Dhecksum over every Byte excluding Startbytes, excluding CRC16, excluding StopBytes. |
| 10 + ...<br/>11 + ... | `0x55`<br/>`0xAA` | **StopBytes**<br/>`0101 0101` // `1010 1010` |

### Flagbyte
| Bitmask | Type/Value | Description |
| --- | --- | --- |
| `B1000 0000` | Flag |	**Direction**<br/>0 = Master to Slave<br/>1 = Slave tp Master<br/>This flag is tecnically not neccesarry. But it allows easy debugging on the (shared) bus. |
| `B0100 0000` | Flag |	**Service**<br/>0 = The frame doesnt contain a _Service_. No PayloadSize-, Service-, Subservice-Byte and no Payload. The CRC16 will still sendt!<br/>1 = The frame contains a _Service_. Payload may still be empty. |
| `B0010 0000` | Flag |	**Push**<br/>Temporary delegates the permission to send to a slave.<br/><br/>_From master to slave:_<br/>0 = No push-clearance for the slave. So saster still owns the permission.<br/>1 = Push-clearance for the slave. Slave is now allowed (and requested) to answer with a push-message.<br/>The Push-Flag can be set on a regulare frame with payload, or in a short push-request (Service-Flag set to 0).<br/>The slave needs to respect the Push-Timeout.<br/><br/>_From slave to master:_<br/>0 = No more push-messages in qeue.<br/>1 = More push-messages in qeue (request more push-clearance).<br/>The slave answers with _one_ regular frame, including Payload. It is not allowed to send more than one frame. If the slave has more push-messages, it will signal this to the master by setting the Push-Flag.<br/>If a slave doesnt have any push-messages in its qeue, he sends a frame with Service-Flag: 0 & Push-Flag: 0. |
| `B0001 0000` | Flag |	**CommitReceive (CR)**<br/>If a node (master or slave) gets a frame with CR-Flag, it is requestet to immediatly sendback the CRC16 (just these two bytes, without any Start-/Stopbytes). |
| `B0000 1111` | - |	**Reserved** |

## Addresses (Address-Byte)
Address of the slave (the master has no address - always send with the slave's own address).
There are up to 240 slaves on a bus allowed.

You can limit the allowed addresses if you like.

| Bitmask | Type/Value | Description |
| --- | --- | --- |
| `B1111 1111` | Binary-Number | Address |
| `B1111 xxxx` | Range | Addresses with `0xF?`are reserved for internal propose and featrue implementations. |

## Services (Service-Byte)
You can define your own _Services_. Each Service gets a unique number (the Service-Byte). Each Service can definie its own Sub-Services.

| Service-Nr | Description |
| --- | --- |
| `0x10` | [gnFellerSwitch](https://github.com/AndiGloor/gnFellerSwitch "GitHub Project") |
| `0xFF` | System-Service (every node is requested to implement this Services). |

### System-Services
| SubService-Nr | Description |
| --- | --- |
| `0x00` | **QueryAlive**<br/>Will be sendt together with the Push-Flag.<br/>The slave answers with the same Service/Subservice, without any Palyoad, to signal _i'm alive and responding_. |
| `0x01` | **Ignore**<br/>The node ignore this frame and doesn't answer (except CommitReceive if CR-Flag is set, or regular pull-message if Pull-Flag is set).<br/>Use this service in combination with _CommitReceive-Flag_ to get a faster alive from a slave than with _QueryAlive-Service_ (minimal timeout, minimal traffic). |

## Timeouts
All Timeouts depending on the baudrate.
You can manipulate them (see [config.h](./src/config.h)).

* **Frame-Timeout:**
  * Maximum duration to send a whole frame. Measured from StartBytes (end) to StopBytes (end).
  * FrameTimeout = Maximum Frame Size * Factor * Time/Byte = (10 + GNMSUP1_MAXPAYLOADBUFFER) * GNMSUP1_FRAMELENGHTTIMEOUT * 10000 / Baudrate _ms, rounded up_
* **Push Timeout:**
  * Maximum duration for a slave to response with a _Push-Message_. Measured from push-clearance StopBytes (end) to the StartBytes sendt by the slave.
  * PushTimeout = 50 _ms_
* **CommitReceive Timeout:**
  * Timeout to send back CRC16. Measured from StopBytes to the last CR-Byte sendt by the node.
  * CommitReceiveTimeout = 0.4 * FrameTimeout

# The implementation
You can configure many parameters (see [config.h](./src/config.h)).
Please dont change parameters in the source.
You can also affect the behavior of a node by using some code-commands (see[examples](./examples)).

## Synchronous Modes
* **SYNCHRONOUS**
  * Send waits till the Push-Answer arrives.
  * Poll waits till all Push-Requests are answered or timeouted.
  * Push waits till the Push-Message is sendt or timeouted.
  * Assure reactive code on every node when you go to synchronous. It gives you full control over the flow of frames. But it also blocks you code with waiting for packets or timeouts.
* **NEARLYASYNCHRONOUS (DEFAULT)**
  * Send donesn't wait for a Push-Answer.
    * A seconds Send or Poll during the waiting period for the Push-Answer will wait blocking until the Push-Message arrived or timeouted.
  * Poll donesn't wait for a Push-Answer.
    * A seconds Send or Poll during the waiting period for the Push-Answer will wait blocking until the Push-Message arrived or timeouted.
    * Polling a whole Address-Range is supported, but act vor any address like _SYNCHRONOUS_, except the last address.
    * To avoid this, don't use Range-Polling. Instead implement your own "Range"-Function and use timers and statemachine.
  * Push  donesn't wait for Push-clearance, instead it works with a qeue.
    * A second Push during waiting period will also be qeued.
    * If the qeue is full, Push waits blocking until the qeue gets a free space. You shoud always avoid this condition.
* **FULLASYNCHRONOUS**
  * Send donesn't wait for a Push-Answer.
    * A seconds Send or Poll during the waiting period for the Push-Answer will fail.
  * Poll donesn't wait for a Push-Answer.
    * A seconds Send or Poll during the waiting period for the Push-Answer will fail.
    * Polling a whole Address-Rage is NOT supported.
  * Push  donesn't wait for Push-clearance, instead it works with a qeue.
    * A second Push during waiting period will also be qeued.
    * If the qeue is full, Push fails.

# Additional Notes
The order of frames is NOT guaranteed at all. If you need to assure a order, you have to implement it with your own _Service_/Payload.

# License
GnMsup1 stands under the MIT License.

Copyright (c) 2018 Andreas Gloor

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
