# Robot Task — State Diagram

```mermaid
stateDiagram-v2
    direction TB

    [*] --> IDLE
    IDLE --> MOVE_HOME

    %% ── Post-home routing ────────────────────────────────────────────────────
    MOVE_HOME --> OPEN_GRIPPER    : R1 cycle 0
    MOVE_HOME --> WAIT_OWN_SENSOR : R2 always · R1 cycles 1+

    %% ── Dice pick (dice_pose) ────────────────────────────────────────────────
    state "Dice Pick  (dice_pose)" as DicePick {
        OPEN_GRIPPER   --> APPROACH_DICE
        APPROACH_DICE  --> DESCEND_DICE
        DESCEND_DICE   --> GRAB_DICE
        GRAB_DICE      --> ASCEND_FROM_DICE
    }
    ASCEND_FROM_DICE --> SAFE_HOME_TO_CAMERA  : queue empty\n(first pick or last BFS rotation)
    ASCEND_FROM_DICE --> SAFE_HOME_TO_SETDOWN : queue non-empty\n(mid-BFS)

    %% ── Conveyor pickup ──────────────────────────────────────────────────────
    state "Conveyor Pickup" as CvPick {
        WAIT_OWN_SENSOR    --> STOP_CONVEYOR
        STOP_CONVEYOR      --> OPEN_GRIPPER_CV
        OPEN_GRIPPER_CV    --> APPROACH_CV_PICKUP
        APPROACH_CV_PICKUP --> DESCEND_CV_PICKUP
        DESCEND_CV_PICKUP  --> GRAB_FROM_CONVEYOR
        GRAB_FROM_CONVEYOR --> ASCEND_CV_PICKUP
    }
    ASCEND_CV_PICKUP --> SAFE_HOME_TO_CAMERA

    %% ── Camera inspection ────────────────────────────────────────────────────
    state "Camera Inspection  (pass 0 · 1 · 2)" as Camera {
        SAFE_HOME_TO_CAMERA --> WAIT_FOR_CAMERA
        WAIT_FOR_CAMERA     --> CLAIM_CAMERA
        CLAIM_CAMERA        --> APPROACH_CAMERA  : claimed
        CLAIM_CAMERA        --> WAIT_FOR_CAMERA  : collision — retry
        APPROACH_CAMERA     --> DESCEND_TO_CAMERA
        DESCEND_TO_CAMERA   --> INSPECT_AT_CAMERA
        INSPECT_AT_CAMERA   --> ASCEND_FROM_CAMERA
        ASCEND_FROM_CAMERA  --> RELEASE_CAMERA
        RELEASE_CAMERA      --> CHECK_PIP_RESULT
    }

    %% ── CHECK_PIP_RESULT branches ────────────────────────────────────────────
    CHECK_PIP_RESULT --> MARK_PIP_DONE        : pass 0 · face_1 == target
    CHECK_PIP_RESULT --> SAFE_HOME_TO_SETDOWN : pass 0 · face_1 ≠ target\n(queue discovery rotation)
    CHECK_PIP_RESULT --> MARK_PIP_DONE        : pass 1 · face_2 == target
    CHECK_PIP_RESULT --> COMPUTE_ORIENTATION  : pass 1 · face_2 ≠ target
    CHECK_PIP_RESULT --> MARK_PIP_DONE        : pass 2 · final confirm
    CHECK_PIP_RESULT --> ERROR                : camera returned 0

    %% ── Orientation reconstruction + BFS planning ────────────────────────────
    note right of COMPUTE_ORIENTATION
        TODO: from_visible_faces(face_1, face_2)
              .apply('roll_forward') → current_state
        rotate_to_pip(target, current_state)
              → _rotation_queue
    end note
    COMPUTE_ORIENTATION --> SAFE_HOME_TO_SETDOWN

    %% ── Rotation setdown ─────────────────────────────────────────────────────
    state "Rotation Setdown  (discovery + BFS)" as Setdown {
        SAFE_HOME_TO_SETDOWN --> APPROACH_SETDOWN
        APPROACH_SETDOWN     --> DESCEND_SETDOWN
        DESCEND_SETDOWN      --> RELEASE_SETDOWN
        RELEASE_SETDOWN      --> ASCEND_SETDOWN
        ASCEND_SETDOWN       --> ROTATE_DICE
    }
    ROTATE_DICE --> OPEN_GRIPPER    : R2 (re-pick dice_pose)\nR1 cycle 0
    ROTATE_DICE --> OPEN_GRIPPER_CV : R1 cycles 1+\n(re-pick conveyor)

    %% ── Pip confirmed ────────────────────────────────────────────────────────
    note right of MARK_PIP_DONE
        Writes coil_pip_done(target_pip)
        to Modbus — shared by all
        three confirmation paths.
    end note
    MARK_PIP_DONE --> SAFE_HOME_TO_CV_DROP : R1 · R2 cycles 0 and 1
    MARK_PIP_DONE --> SAFE_HOME_TO_FINAL   : R2 cycle 2 only

    %% ── Conveyor drop ────────────────────────────────────────────────────────
    state "Conveyor Drop" as CvDrop {
        SAFE_HOME_TO_CV_DROP --> APPROACH_CV_DROP
        APPROACH_CV_DROP     --> DESCEND_CV_DROP
        DESCEND_CV_DROP      --> RELEASE_CV_DROP
        RELEASE_CV_DROP      --> ASCEND_CV_DROP
        ASCEND_CV_DROP       --> START_CONVEYOR
        START_CONVEYOR       --> ADVANCE_CYCLE
    }
    ADVANCE_CYCLE --> SAFE_HOME_TO_WAIT : more cycles remain
    ADVANCE_CYCLE --> FINAL_HOME        : all pips done
    SAFE_HOME_TO_WAIT --> WAIT_OWN_SENSOR

    %% ── R2 final placement (cycle 2 only) ────────────────────────────────────
    state "R2 Final Placement  (cycle 2)" as FinalPlace {
        SAFE_HOME_TO_FINAL   --> APPROACH_FINAL_HOME
        APPROACH_FINAL_HOME  --> DESCEND_FINAL_HOME
        DESCEND_FINAL_HOME   --> RELEASE_FINAL
        RELEASE_FINAL        --> ASCEND_FINAL_HOME
        ASCEND_FINAL_HOME    --> FINAL_HOME
    }

    %% ── Terminal ─────────────────────────────────────────────────────────────
    FINAL_HOME --> COMPLETE
    COMPLETE   --> [*]
    ERROR      --> [*]
```

---

## Path summary

| Path | Condition | Camera visits | Rotations |
|------|-----------|--------------|-----------|
| Fast 1 | face_1 == target | 1 | 0 |
| Fast 2 | face_2 == target (after discovery) | 2 | 1 |
| Full BFS | face_1 ≠ target, face_2 ≠ target | 3 | 1 + N (BFS) |

## Key conditional transitions

| State | Condition | Next |
|-------|-----------|------|
| `MOVE_HOME` | R1, cycle 0 | `OPEN_GRIPPER` |
| `MOVE_HOME` | R2 / R1 cycle 1+ | `WAIT_OWN_SENSOR` |
| `ASCEND_FROM_DICE` | `_rotation_queue` empty | `SAFE_HOME_TO_CAMERA` |
| `ASCEND_FROM_DICE` | `_rotation_queue` non-empty | `SAFE_HOME_TO_SETDOWN` |
| `ROTATE_DICE` | R2 or R1 cycle 0 | `OPEN_GRIPPER` |
| `ROTATE_DICE` | R1 cycles 1+ | `OPEN_GRIPPER_CV` |
| `MARK_PIP_DONE` | R2 cycle 2 | `SAFE_HOME_TO_FINAL` |
| `MARK_PIP_DONE` | all others | `SAFE_HOME_TO_CV_DROP` |
| `ADVANCE_CYCLE` | pips remain | `SAFE_HOME_TO_WAIT` |
| `ADVANCE_CYCLE` | all pips done | `FINAL_HOME` |
