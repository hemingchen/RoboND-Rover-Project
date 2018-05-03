## Project: Search and Sample Return

---
### Notebook Analysis

The `process_image()` function in `cell 9` of the Jupyter notebook contains the entire image processing pipeline used in the perception code:

- Create a empty output image - `line 24`.
- Attach original camera image to upper left corner of the output image - `line 27`.
- Do perspective transformation by calling the `perspect_transform()` function at `line 30`, where
```
source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset]])
```
- Generate masks for navigable terrain, rock samples and obstacle at `line 33-35`, where
```
Navigable terrain - rgb >= (160, 160, 160)
Rock sample - (0, 105, 0) <= rgb <= (255, 220, 65)
Obstacle - the opposite of navigable terrain mask
```
- Compose masked vision image and add to top right of the output image at `line 38-44`.
- Extract world coordinates of navigable terrain, rock samples and obstacle and add them to the worldmap at `line 47-72`. `rover_coords()`, `pix_to_world()` are used here.
- Overlay worldmap with ground truth map at `line 75`.
- Complete the output image by adding the processed worldmap at `line 78`.
- Add text and other information at `line 81`.

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
- Update `Rover.worldmap` to reflect navigable terrain, obstacles and rock samples after coordinate translation.
- Extract `dists` and `angles` from either navigable terrain (`FORWARD` mode) or rock samples (`APPROACH_SAMPLE` mode), and pass such information to decision module.

#### Decision

`decision_step()` now handles four modes: `FORWARD`, `STOP`, `APPROACH_SAMPLE` and `STUCK`.

In `FORWARD` mode, Rover will follow navigable terrain until it finds rock sample in sight and switches to `APPROACH_SAMPLE` mode. In case the Rover hits the end of the navigable terrain, it switches to `STOP` mode and then turns around to explore other areas. In case the Rover gets stuck at one location, it will switch to `STUCK` mode and try to back up and turn at the same time until it can resume to `FORWARD` mode.