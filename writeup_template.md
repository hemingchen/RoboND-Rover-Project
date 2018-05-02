## Project: Search and Sample Return

---
### Notebook Analysis

Jupyter notebook was mainly used to test color thresholds to identify navigable terrain, obstacle and rock sample. Eventually, the following thresholds are used:

- Navigable terrain - rgb >= (160, 160, 160)
- Rock sample - (0, 105, 0) <= rgb <= (255, 220, 65)
- Obstacle, the opposite of navigable terrain mask

### Autonomous Navigation

#### Perception

Two new modes are added to the Rover - `APPROACH_SAMPLE` and `STUCK`. 

Rover switches to `APPROACH_SAMPLE` mode as soon as it finds rock sample in sight. In this mode Rover will graduate move towards the rock sample and eventually stops at the sample and picks it up. 

Rover switches to `STUCK` mode if throttle is applied but Rover speed is still `0`. In such mode Rover will try some maneuvers to escape from such situation, e.g. back up and turn.

All modes are now in a new class:
```
class RoverMode:
    STOP = 'stop'
    FORWARD = 'forward'
    APPROACH_SAMPLE = 'approach_sample'
    STUCK = 'stuck'
```

The `perception_step()` function was updated to follow the workflow below:

- Apply perspective transformation.
- Apply color threshold to crop out navigable terrain, obstacles and rock samples.
- Update `Rover.vision_image` to reflect navigable terrain, obstacles and rock samples.
- Do coordinate translation and update `Rover.worldmap`.
- Extract `dists` and `angles` from either navigable terrain (`FORWARD` mode) or rock samples (`APPROACH_SAMPLE` mode), and pass such information to decision module.

#### Decision

`decision_step()` now handles four modes: `FORWARD`, `STOP`, `APPROACH_SAMPLE` and `STUCK`.

In `FORWARD` mode, Rover will follow navigable terrain until it finds rock sample in sight and switches to `APPROACH_SAMPLE` mode. In case the Rover hits the end of the navigable terrain, it switches to `STOP` mode and then turns around to explore other areas. In case the Rover gets stuck at one location, it will switch to `STUCK` mode and try to back up and turn at the same time until it can resume to `FORWARD` mode.