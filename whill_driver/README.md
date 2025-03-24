# whill_driver

## About
The "whill_driver" is a ROS2 package for WHILL Model CR2 controller.

<img width=22% title="WHILL Model CR2" src="https://github.com/WHILL/Model_CR_Technical_Support/assets/129816934/387a8aac-3808-4727-895d-9857059ee342">
<img width=24% title="Wheeled Robot Base" src="https://github.com/WHILL/Model_CR_Technical_Support/assets/129816934/6a63ed45-9e0c-40ca-b71c-8dff614af141">


## Package Structure

```mermaid
classDiagram

namespace whill_driver {
    class WhillNode{
        -publiser
        -subscriber
        -server
    }
    class Whill{
        +SendCommand()
        +ReceiveDataset()
    }
    class Parser{
        +Parse()
        +Checksum()
    }
    class SerialPort{
        +Open()
        +Close()
        +Send()
        +Receive()
    }
}

namespace rclcpp {
    class Node{
    }
    class Publisher{
    }
    class Subscription{
    }
    class Service{
    }
}

namespace whill_msgs {
    class WhillModelCMsg{        
    }
    class SpeedProfileMsg{        
    }
    class SetPowerSrv{
    }
    class SetSpeedProfileSrv{
    }
}

namespace c {
    class unistd{
        +open()
        +close()
        +write()
        +read()
    }
}

WhillNode --|> Node
WhillNode -- Publisher
WhillNode -- Subscription
WhillNode -- Service
WhillNode "1" *-- "1" Whill
Whill "1" *-- "1" Parser
Whill "1" *-- "1" SerialPort
SerialPort -- unistd
Whill -- WhillModelCMsg
Whill -- SpeedProfileMsg
Whill -- SetPowerSrv
Whill -- SetSpeedProfileSrv
```


| Class name | Explanation |
|:---|:---|
| WhillNode | This class provides ROS2 Node, and has responsible for RCLCPP abstraction. |
| Whill | This class has WHILL Model CR2 domain. So, this class can understand control commands and WHILL state datasets. |
| Parser | This class has the ability to parse packets from WHILL. |
| SerialPort | This class provides serial port driver, and has responsible for hardware abstraction. |

**Note:** [whill_msgs](https://github.com/WHILL/ros2_whill_interfaces) is not provided by this repository.

## License

Copyright (c) 2024 WHILL, Inc.

Released under the [MIT license](https://opensource.org/licenses/mit-license.php)
