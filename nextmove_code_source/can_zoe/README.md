# can_zoe

Decoder and encoder for ze50 bus can.

## Version

ROS_DISTRO: Humble  

### Dependencies

- pretil-zoe-ze40-can-msgs: <https://gitlab.cristal.univ-lille.fr/open-pretil/pretil_robot_msgs/pretil-zoe-ze40-can-msgs/-/tree/humble>
- pretil-zoe-ze50-can-msgs: <https://gitlab.cristal.univ-lille.fr/open-pretil/pretil_robot_msgs/pretil-zoe-ze50-can-msgs/-/tree/humble>

To work in the ze50, nodes need kvaser, the bus CAN interface. The launch file can be found in zoe_launch :
    <https://gitlab.cristal.univ-lille.fr/pretil/zoe/zoe_launch/-/tree/humble>

## DEPRECATED FILES

zoe_control is still used for ze40 but will disappear in the future.  
Speed_controller and steer_teleop are unused.

## Tree

```text
└── src
    ├── can_zoe                             # Bus CAN encoding/decoding
    ├── pretil-zoe-ze40-can-msgs
    ├── pretil-zoe-ze50-can-msgs
    └── zoe_launch                          # Optional for build and simulating, mandatory in the car
         launch
            ├── kvaser_bridge.launch.xml    # Mandatory in the car
            └── zoe_can_ze50_launch.py      # Optional: kvaser, decoder, encoder
```

## Nodes

```bash
ros2 run can_zoe can_ze50_decoder
```

```bash
ros2 run can_zoe can_ze50_encoder
```

Those nodes are yet to be merged in the future.
