# composable_player API

## Architecture

```mermaid
graph LR
    subgraph ComposableContainer
        P[PlayerNode]
        R[RecorderNode]
    end
    BAG[(bag/mcap)] -->|read| P
    P -->|publish| TOPICS{{ROS2 Topics}}
    P -->|bag /clock or synthetic| CLK[/clock]
    CLK -.->|use_sim_time| R
    TOPICS -->|subscribe| R
    R -->|write| OUT[(output bag/mcap)]
```

## Components

### composable_player::PlayerNode

Reads MCAP/rosbag files and publishes serialized messages on original topics.

#### Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `bag_uri` | string | **required** | Bag file/directory path |
| `storage_id` | string | `"mcap"` | Storage plugin: `mcap` / `sqlite3` |
| `rate` | double | `1.0` | Playback speed multiplier |
| `loop` | bool | `false` | Loop on completion |
| `start_paused` | bool | `false` | Start paused |
| `topics` | string[] | `[]` | Topic filter (empty = all) |
| `use_sim_time` | bool | `false` | Use `/clock` for `now()` |
| `publish_clock` | bool | `false` | Generate synthetic `/clock` (only when bag lacks `/clock`) |
| `clock_frequency` | double | `100.0` | Synthetic `/clock` rate (Hz) |

#### /clock Handling

```mermaid
flowchart TD
    A[open_bag] --> B{bag contains /clock?}
    B -->|Yes| C[Replay /clock from bag via GenericPublisher]
    B -->|No| D{publish_clock param?}
    D -->|true| E[Generate synthetic /clock from msg timestamps]
    D -->|false| F[No /clock published]
```

- **Bag has `/clock`**: Replayed as-is (e.g., simulation clock from Gazebo). `publish_clock` param is ignored.
- **Bag lacks `/clock`** + `publish_clock=true`: Synthetic `/clock` generated from `last_bag_time_ns_` at `clock_frequency` Hz.
- **Bag lacks `/clock`** + `publish_clock=false`: No `/clock` published.

#### Services

| Name | Type | Description |
|---|---|---|
| `~/pause` | `std_srvs/srv/Trigger` | Pause playback |
| `~/resume` | `std_srvs/srv/Trigger` | Resume playback |

#### Published Topics

| Name | Type | Condition |
|---|---|---|
| *(from bag)* | *(original types)* | Always |
| `/clock` | `rosgraph_msgs/msg/Clock` | Bag has `/clock`, or `publish_clock=true` fallback |

---

### composable_player::RecorderNode

Subscribes to ROS2 topics and writes serialized messages to MCAP/rosbag files.

#### Parameters

| Name | Type | Default | Dynamic | Description |
|---|---|---|---|---|
| `output_uri` | string | **required** | No | Output bag path |
| `storage_id` | string | `"mcap"` | No | Storage plugin: `mcap` / `sqlite3` |
| `topics` | string[] | `[]` | **Yes** | Topics to record |
| `all_topics` | bool | `false` | **Yes** | Discover and record all topics |
| `exclude_topics` | string[] | `["/rosout", "/parameter_events"]` | **Yes** | Exclude list (`all_topics` mode) |
| `use_sim_time` | bool | `false` | No | Use `/clock` for message timestamps |
| `max_bag_size` | int | `0` | No | Max file size bytes (0 = unlimited) |
| `max_bag_duration` | int | `0` | No | Max duration nanoseconds (0 = unlimited) |

Dynamic parameters can be changed at runtime via `ros2 param set`.

`/clock` is **not** excluded by default. Simulation clock is recorded for faithful replay.

#### Services

| Name | Type | Description |
|---|---|---|
| `~/stop` | `std_srvs/srv/Trigger` | Stop recording and flush |

#### Subscribed Topics

Dynamic — determined by `topics` parameter or topic discovery.

---

## Usage

### Standalone Player

```bash
ros2 launch composable_player player.launch.py bag_uri:=/path/to/bag
```

### Player with sim time (bag contains /clock)

```bash
ros2 launch composable_player player.launch.py \
  bag_uri:=/path/to/sim_bag \
  use_sim_time:=false
```

`/clock` from bag is replayed automatically.

### Player with synthetic clock (bag lacks /clock)

```bash
ros2 launch composable_player player.launch.py \
  bag_uri:=/path/to/real_bag \
  publish_clock:=true
```

### Standalone Recorder

```bash
ros2 launch composable_player recorder.launch.py \
  output_uri:=/path/to/output
```

### Recorder with specific topics

```bash
ros2 launch composable_player recorder.launch.py \
  output_uri:=/path/to/output \
  all_topics:=false

ros2 param set /recorder topics "['/camera/image', '/imu/data', '/lidar/points']"
```

### Composable Container (Single Process)

```bash
ros2 launch composable_player composable.launch.py \
  bag_uri:=/path/to/input \
  output_uri:=/path/to/output \
  use_sim_time:=true
```

### Runtime Component Loading

```bash
ros2 run rclcpp_components component_container

ros2 component load /ComponentManager composable_player composable_player::PlayerNode \
  -p bag_uri:=/path/to/bag -p rate:=2.0

ros2 component load /ComponentManager composable_player composable_player::RecorderNode \
  -p output_uri:=/path/to/output -p all_topics:=true -p use_sim_time:=true
```

### Dynamic Topic Update

```bash
ros2 param set /recorder topics "['/topic_a', '/topic_b']"
ros2 param set /recorder all_topics true
ros2 param set /recorder exclude_topics "['/rosout']"
```

### Service Calls

```bash
ros2 service call /player/pause std_srvs/srv/Trigger
ros2 service call /player/resume std_srvs/srv/Trigger
ros2 service call /recorder/stop std_srvs/srv/Trigger
```

## use_sim_time Sequence

```mermaid
sequenceDiagram
    participant Bag as Bag File
    participant Player as PlayerNode
    participant Clock as /clock
    participant Recorder as RecorderNode
    participant Other as Other Nodes

    Player->>Bag: read_next()
    alt Bag has /clock
        Player->>Clock: publish(bag /clock msg)
    else Bag lacks /clock (publish_clock=true)
        Note over Player: last_bag_time = msg.time_stamp
        Player->>Clock: publish(synthetic clock)
    end
    Clock-->>Recorder: sim time (use_sim_time=true)
    Clock-->>Other: sim time (use_sim_time=true)
    Player->>Other: publish bag topics
    Other->>Recorder: processed data
    Recorder->>Recorder: now() returns sim time
```

## Storage Formats

| `storage_id` | Format | Extension |
|---|---|---|
| `mcap` | MCAP | `.mcap` |
| `sqlite3` | ROS2 bag (SQLite3) | `.db3` |
