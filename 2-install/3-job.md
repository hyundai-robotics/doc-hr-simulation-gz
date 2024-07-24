# Hi6 Controller Job Code

This job code is designed for UDP communication to control the robot and receive current axis position values.

## Overview

The code sets up a UDP connection and uses an online trajectory buffer to control the robot's movements. It continuously receives messages, sends them back, and adds the received poses to the trajectory buffer.

## Code Breakdown

```text
Hyundai Robot Job File; { version: 1.6, mech_type: "", total_axis: -1, aux_axis: 0 }
```

This line specifies the job file format and version.

### Network Setup

```text
import enet
global enet0
enet0=enet.ENet()
enet0.ip_addr="192.168.1.2" #PC IP
enet0.lport=7000
enet0.rport=7000
enet0.open
```

- Imports the `enet` module for network communication
- Creates a global `ENet` object named `enet0`
- Sets the IP address to communicate with (presumably the PC's IP)
- Sets local and remote ports to 7000
- Opens the network connection

### Online Trajectory Setup

```text
global onl
onl=online.Traject()
onl.time_from_start=-1.0 # disable
onl.look_ahead_time=0.1 #The time it takes to execute after a value is entered into the online.Traject buffer.
onl.interval=0.1 #Online.Traject Movement time from pose accumulated in buffer to next pose
onl.init # online trajectory operation init (buffer init)
```

- Creates a global `online.Traject` object named `onl`
- Configures the online trajectory parameters:
  - Disables `time_from_start`
  - Sets `look_ahead_time` to 0.1 seconds
  - Sets `interval` to 0.1 seconds
- Initializes the online trajectory buffer

### Main Loop

```text
var msg
#var po
10 enet0.recv
msg=result()
enet0.send msg
print msg
#po=Pose(msg)
onl.buf_in msg #Put the pose in the 'po' variable into the online.Traject buffer.
goto 10
```

- Declares variables `msg` (and `po`, which is commented out)
- Enters a loop (label 10):
  1. Receives a message via UDP
  2. Stores the received message in `msg`
  3. Sends the message back (echo)
  4. Prints the message
  5. Adds the received message to the online trajectory buffer
  6. Loops back to the beginning (goto 10)

## Notes

- The code uses UDP for real-time communication, which is suitable for fast, lightweight data transfer.
- The online trajectory buffer allows for smooth, continuous motion by interpolating between received poses.
- The commented-out lines suggest that the received message might be convertible to a `Pose` object, but this functionality is not currently in use.
- The continuous loop allows for real-time control and feedback of the robot's position.

## code

```text
Hyundai Robot Job File; { version: 1.6, mech_type: "", total_axis: -1, aux_axis: 0 }
     import enet
     global enet0
     enet0=enet.ENet()
     enet0.ip_addr="192.168.1.2" #PC IP
     enet0.lport=7000
     enet0.rport=7000
     enet0.open
     global onl
     onl=online.Traject()
     onl.time_from_start=-1.0 # disable
     onl.look_ahead_time=0.1 #The time it takes to execute after a value is entered into the online.Traject buffer.
     onl.interval=0.1 #Online.Traject Movement time from pose accumulated in buffer to next pose
     onl.init # online trajectory operation init (buffer init)
     var msg
     #var po
  10 enet0.recv
     msg=result()
     enet0.send msg
     print msg
     #po=Pose(msg)
     onl.buf_in msg #Put the pose in the ‘po’ variable into the online.Traject buffer.
     goto 10
     end 
```
