"""
Standard six-sided die orientation model.

A die's full orientation is represented as (top, front, right).
The three remaining faces are derived: bottom = 7-top, back = 7-front, left = 7-right.
This works because opposite faces on a standard die always sum to 7.

Standard Western die chirality (right-handed): top=1, front=2, right=3.
All 24 valid orientations are enumerated at import time.

Six rotation primitives
-----------------------
    roll_forward   — die tips away from viewer (top→back, front→top)
    roll_backward  — die tips toward viewer    (top→front, back→top)
    roll_right     — die tips right            (top→right→bottom→left→top cycle)
    roll_left      — die tips left             (top→left→bottom→right→top cycle)
    spin_cw        — spin clockwise from above (front→right, right→back …)
    spin_ccw       — spin counter-clockwise    (front→left,  right→front …)

These names are the strings passed to RotationSchema.execute().
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Optional

# Opposite-face lookup for a standard die (faces sum to 7)
_OPP: dict[int, int] = {1: 6, 6: 1, 2: 5, 5: 2, 3: 4, 4: 3}

ROTATION_NAMES: tuple[str, ...] = (
    'roll_forward', 'roll_backward',
    'roll_left',    'roll_right',
    'spin_cw',      'spin_ccw',
)


@dataclass(frozen=True)
class DiceState:
    """
    Full orientation of a standard six-sided die.

    Attributes:
        top:   pip count on the top face
        front: pip count on the face toward the viewer
        right: pip count on the right face
    """
    top:   int
    front: int
    right: int

    # ---- Derived faces -------------------------------------------------------

    @property
    def bottom(self) -> int:
        return _OPP[self.top]

    @property
    def back(self) -> int:
        return _OPP[self.front]

    @property
    def left(self) -> int:
        return _OPP[self.right]

    def as_dict(self) -> dict[str, int]:
        """All six faces as a dict keyed by face name."""
        return {
            'top':    self.top,    'bottom': self.bottom,
            'front':  self.front,  'back':   self.back,
            'right':  self.right,  'left':   self.left,
        }

    # ---- Six rotation primitives ---------------------------------------------

    def roll_forward(self) -> DiceState:
        """Tip away from viewer: top→back, front→top, bottom→front, back→bottom."""
        return DiceState(self.front, _OPP[self.top], self.right)

    def roll_backward(self) -> DiceState:
        """Tip toward viewer: top→front, back→top, bottom→back, front→bottom."""
        return DiceState(_OPP[self.front], self.top, self.right)

    def roll_right(self) -> DiceState:
        """Tip to the right: left→top, top→right, right→bottom, bottom→left."""
        return DiceState(_OPP[self.right], self.front, self.top)

    def roll_left(self) -> DiceState:
        """Tip to the left: right→top, top→left, left→bottom, bottom→right."""
        return DiceState(self.right, self.front, _OPP[self.top])

    def spin_cw(self) -> DiceState:
        """Spin clockwise from above: front→right, right→back, back→left, left→front."""
        return DiceState(self.top, _OPP[self.right], self.front)

    def spin_ccw(self) -> DiceState:
        """Spin counter-clockwise: front→left, left→back, back→right, right→front."""
        return DiceState(self.top, self.right, _OPP[self.front])

    def apply(self, rotation_name: str) -> DiceState:
        """Apply a rotation by name. Name must be one of ROTATION_NAMES."""
        return getattr(self, rotation_name)()

    def __repr__(self) -> str:
        return (f'DiceState(top={self.top}, front={self.front}, right={self.right}, '
                f'bottom={self.bottom}, back={self.back}, left={self.left})')


# ---------------------------------------------------------------------------
#  All 24 valid orientations — built once at import time via BFS
# ---------------------------------------------------------------------------

#: Canonical starting orientation for a right-handed Western die.
CANONICAL = DiceState(top=1, front=2, right=3)


def _enumerate_orientations() -> frozenset[DiceState]:
    seen:  set[DiceState]          = {CANONICAL}
    queue: deque[DiceState]        = deque([CANONICAL])
    while queue:
        state = queue.popleft()
        for name in ROTATION_NAMES:
            nxt = state.apply(name)
            if nxt not in seen:
                seen.add(nxt)
                queue.append(nxt)
    assert len(seen) == 24, f'Expected 24 orientations, found {len(seen)}'
    return frozenset(seen)


#: All 24 reachable orientations of a standard die.
ALL_ORIENTATIONS: frozenset[DiceState] = _enumerate_orientations()


# ---------------------------------------------------------------------------
#  Discovery rotation helpers
# ---------------------------------------------------------------------------

#: Maps a discovery rotation name to the face of the *initial* state that
#: becomes the new top after that rotation.  Used by the orientation-discovery
#: algorithm: execute the rotation, read the new top T2, then call
#: from_visible_faces(top=T1, **{DISCOVERY_FACE_REVEALED[rotation]: T2}).
#:
#: spin_cw / spin_ccw are excluded — they leave the top face unchanged and
#: therefore reveal no new information.
DISCOVERY_FACE_REVEALED: dict[str, str] = {
    'roll_forward':  'front',  # after roll_forward,  new top = initial front face
    'roll_backward': 'back',   # after roll_backward, new top = initial back  face
    'roll_right':    'left',   # after roll_right,    new top = initial left  face
    'roll_left':     'right',  # after roll_left,     new top = initial right face
}


# ---------------------------------------------------------------------------
#  Reconstruct state from partial observations
# ---------------------------------------------------------------------------

def from_visible_faces(**known_faces: int) -> Optional[DiceState]:
    """
    Reconstruct the full die orientation from two or three known face values.

    Three *adjacent* faces (e.g. top + front + right) uniquely determine the
    orientation.  Two faces that are *opposite* leave the roll angle ambiguous.

    Keyword arguments are face names mapped to pip counts:
        'top', 'bottom', 'front', 'back', 'left', 'right'

    Returns the unique matching DiceState, or None if the result is ambiguous
    or impossible.

    Examples
    --------
    Known from a corner camera (top, front, right all visible):
        state = from_visible_faces(top=3, front=1, right=5)

    Known only top and front (spin ambiguous → returns None):
        state = from_visible_faces(top=1, front=2)   # None — two matches
    """
    matches = [
        s for s in ALL_ORIENTATIONS
        if all(s.as_dict().get(face) == value for face, value in known_faces.items())
    ]
    return matches[0] if len(matches) == 1 else None
