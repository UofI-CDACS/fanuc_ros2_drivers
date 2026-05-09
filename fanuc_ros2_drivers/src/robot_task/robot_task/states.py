from enum import Enum, auto


class State(Enum):
    IDLE                 = auto()
    MOVE_HOME            = auto()

    # ── Dice pickup (R1 cycle 0 initial pick; R2/R1 after rotation setdown) ─
    OPEN_GRIPPER         = auto()
    APPROACH_DICE        = auto()
    DESCEND_DICE         = auto()
    GRAB_DICE            = auto()
    ASCEND_FROM_DICE     = auto()

    # ── Conveyor pickup: wait for dice to arrive, then grab it ────────────
    WAIT_OWN_SENSOR      = auto()   # block until this robot's prox sensor fires
    STOP_CONVEYOR        = auto()   # stop the pickup conveyor
    OPEN_GRIPPER_CV      = auto()
    APPROACH_CV_PICKUP   = auto()
    DESCEND_CV_PICKUP    = auto()
    GRAB_FROM_CONVEYOR   = auto()
    ASCEND_CV_PICKUP     = auto()

    # ── Camera inspection loop (shared) ──────────────────────────────────
    SAFE_HOME_TO_CAMERA  = auto()
    WAIT_FOR_CAMERA      = auto()
    CLAIM_CAMERA         = auto()
    APPROACH_CAMERA      = auto()
    INSPECT_AT_CAMERA    = auto()
    RELEASE_CAMERA       = auto()
    CHECK_PIP_RESULT     = auto()   # branches on _camera_pass (0=face1, 1=face2, 2=final)
    COMPUTE_ORIENTATION  = auto()   # reconstruct die orientation + BFS plan (pass 1 no-match only)

    # ── Rotation (shared) ─────────────────────────────────────────────────
    # Used for both the mandatory discovery rotation and each BFS rotation.
    SAFE_HOME_TO_SETDOWN = auto()
    APPROACH_SETDOWN     = auto()
    DESCEND_SETDOWN      = auto()
    RELEASE_SETDOWN      = auto()
    ASCEND_SETDOWN       = auto()
    ROTATE_DICE          = auto()   # pops _current_rotation from _rotation_queue

    # ── Pip confirmed: write coil then proceed to conveyor/final ──────────
    MARK_PIP_DONE        = auto()   # single convergence point for all 3 confirmation paths

    # ── Conveyor drop: place dice on drop conveyor and start belt ─────────
    SAFE_HOME_TO_CV_DROP = auto()
    APPROACH_CV_DROP     = auto()
    DESCEND_CV_DROP      = auto()
    RELEASE_CV_DROP      = auto()
    ASCEND_CV_DROP       = auto()
    START_CONVEYOR       = auto()

    # ── Cycle bookkeeping + safe return to wait position ─────────────────
    ADVANCE_CYCLE        = auto()
    SAFE_HOME_TO_WAIT    = auto()   # go home, then WAIT_OWN_SENSOR

    # ── R2 cycle 2 only: final placement at dice_pose ─────────────────────
    SAFE_HOME_TO_FINAL   = auto()
    APPROACH_FINAL_HOME  = auto()
    DESCEND_FINAL_HOME   = auto()
    RELEASE_FINAL        = auto()
    ASCEND_FINAL_HOME    = auto()

    FINAL_HOME           = auto()
    COMPLETE             = auto()
    ERROR                = auto()
