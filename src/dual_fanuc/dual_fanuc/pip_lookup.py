"""
pip_lookup.py — Die orientation lookup tables for Robot 1 and Robot 2.
"""

# ── Pip 2 ──────────────────────────────────────────────────────────────────────
PIP2_LOOKUP = {
    # scan_top=1 
    (1, 2): 'none',
    (1, 3): 'rotate_y_pos90',
    (1, 4): 'rotate_y_neg90',
    (1, 5): 'flip_x_180',

    # scan_top=2 
    (2, 1): 'rotate_x_pos90',
    (2, 3): 'rotate_x_pos90',
    (2, 4): 'rotate_x_pos90',
    (2, 6): 'rotate_x_pos90',

    # scan_top=3
    (3, 1): 'rotate_y_neg90',
    (3, 2): 'none',
    (3, 5): 'flip_x_180',
    (3, 6): 'rotate_y_pos90',

    # scan_top=4
    (4, 1): 'rotate_y_pos90',
    (4, 2): 'none',
    (4, 5): 'flip_x_180',
    (4, 6): 'rotate_y_neg90',

    # scan_top=5 
    (5, 1): 'rotate_x_neg90',
    (5, 3): 'rotate_x_neg90',
    (5, 4): 'rotate_x_neg90',
    (5, 6): 'rotate_x_neg90',

    # scan_top=6
    (6, 2): 'none',
    (6, 3): 'rotate_y_neg90',
    (6, 4): 'rotate_y_pos90',
    (6, 5): 'flip_x_180',
}

# ── Pip 4 ──────────────────────────────────────────────────────────────────────
PIP4_LOOKUP = {
    # scan_top=1
    (1, 2): 'rotate_y_pos90',
    (1, 3): 'flip_x_180',
    (1, 4): 'none',
    (1, 5): 'rotate_y_neg90',

    # scan_top=2
    (2, 1): 'rotate_y_neg90',
    (2, 3): 'flip_x_180',
    (2, 4): 'none',
    (2, 6): 'rotate_y_pos90',

    # scan_top=3 
    (3, 1): 'rotate_x_neg90',
    (3, 2): 'rotate_x_neg90',
    (3, 5): 'rotate_x_neg90',
    (3, 6): 'rotate_x_neg90',

    # scan_top=4 
    (4, 1): 'rotate_x_pos90',
    (4, 2): 'rotate_x_pos90',
    (4, 5): 'rotate_x_pos90',
    (4, 6): 'rotate_x_pos90',

    # scan_top=5
    (5, 1): 'rotate_y_pos90',
    (5, 3): 'flip_x_180',
    (5, 4): 'none',
    (5, 6): 'rotate_y_neg90',

    # scan_top=6
    (6, 2): 'rotate_y_neg90',
    (6, 3): 'flip_x_180',
    (6, 4): 'none',
    (6, 5): 'rotate_y_pos90',
}

# ── Pip 6 ──────────────────────────────────────────────────────────────────────
PIP6_LOOKUP = {
    # scan_top=1
    (1, 2): 'rotate_x_neg90',
    (1, 3): 'rotate_x_neg90',
    (1, 4): 'rotate_x_neg90',
    (1, 5): 'rotate_x_neg90',

    # scan_top=2
    (2, 1): 'flip_x_180',
    (2, 3): 'rotate_y_pos90',
    (2, 4): 'rotate_y_neg90',
    (2, 6): 'none',

    # scan_top=3
    (3, 1): 'flip_x_180',
    (3, 2): 'rotate_y_neg90',
    (3, 5): 'rotate_y_pos90',
    (3, 6): 'none',

    # scan_top=4
    (4, 1): 'flip_x_180',
    (4, 2): 'rotate_y_pos90',
    (4, 5): 'rotate_y_neg90',
    (4, 6): 'none',

    # scan_top=5
    (5, 1): 'flip_x_180',
    (5, 3): 'rotate_y_neg90',
    (5, 4): 'rotate_y_pos90',
    (5, 6): 'none',

    # scan_top=6 
    (6, 2): 'rotate_x_pos90',
    (6, 3): 'rotate_x_pos90',
    (6, 4): 'rotate_x_pos90',
    (6, 5): 'rotate_x_pos90',
}

# ── Pip 1 ──────────────────────────────────────────────────────────────────────

PIP1_LOOKUP = {
    # scan_top=1 
    (1, 2): 'rotate_x_neg90',
    (1, 3): 'rotate_x_neg90',
    (1, 4): 'rotate_x_neg90',
    (1, 5): 'rotate_x_neg90',

    # scan_top=2
    (2, 1): 'none',
    (2, 3): 'rotate_y_pos90',
    (2, 4): 'rotate_y_neg90',
    (2, 6): 'flip_x_180',

    # scan_top=3
    (3, 1): 'none',
    (3, 2): 'rotate_y_neg90',
    (3, 5): 'rotate_y_pos90',
    (3, 6): 'flip_x_180',

    # scan_top=4
    (4, 1): 'none',
    (4, 2): 'rotate_y_pos90',
    (4, 5): 'rotate_y_neg90',
    (4, 6): 'flip_x_180',

    # scan_top=5
    (5, 1): 'none',
    (5, 3): 'rotate_y_neg90',
    (5, 4): 'rotate_y_pos90',
    (5, 6): 'flip_x_180',

    # scan_top=6 
    (6, 2): 'rotate_x_pos90',
    (6, 3): 'rotate_x_pos90',
    (6, 4): 'rotate_x_pos90',
    (6, 5): 'rotate_x_pos90',
}

# ── Pip 3 ──────────────────────────────────────────────────────────────────────
PIP3_LOOKUP = {
    # scan_top=1
    (1, 2): 'rotate_y_pos90',
    (1, 3): 'none',
    (1, 4): 'flip_x_180',
    (1, 5): 'rotate_y_neg90',

    # scan_top=2
    (2, 1): 'rotate_y_neg90',
    (2, 3): 'none',
    (2, 4): 'flip_x_180',
    (2, 6): 'rotate_y_pos90',

    # scan_top=3 
    (3, 1): 'rotate_x_neg90',
    (3, 2): 'rotate_x_neg90',
    (3, 5): 'rotate_x_neg90',
    (3, 6): 'rotate_x_neg90',

    # scan_top=4 
    (4, 1): 'rotate_x_pos90',
    (4, 2): 'rotate_x_pos90',
    (4, 5): 'rotate_x_pos90',
    (4, 6): 'rotate_x_pos90',

    # scan_top=5
    (5, 1): 'rotate_y_pos90',
    (5, 3): 'none',
    (5, 4): 'flip_x_180',
    (5, 6): 'rotate_y_neg90',

    # scan_top=6
    (6, 2): 'rotate_y_neg90',
    (6, 3): 'none',
    (6, 4): 'flip_x_180',
    (6, 5): 'rotate_y_pos90',
}

# ── Pip 5 ──────────────────────────────────────────────────────────────────────
PIP5_LOOKUP = {
    # scan_top=1
    (1, 2): 'flip_x_180',
    (1, 3): 'rotate_y_pos90',
    (1, 4): 'rotate_y_neg90',
    (1, 5): 'none',

    # scan_top=2 
    (2, 1): 'rotate_x_pos90',
    (2, 3): 'rotate_x_pos90',
    (2, 4): 'rotate_x_pos90',
    (2, 6): 'rotate_x_pos90',

    # scan_top=3
    (3, 1): 'rotate_y_neg90',
    (3, 2): 'flip_x_180',
    (3, 5): 'none',
    (3, 6): 'rotate_y_pos90',

    # scan_top=4
    (4, 1): 'rotate_y_pos90',
    (4, 2): 'flip_x_180',
    (4, 5): 'none',
    (4, 6): 'rotate_y_neg90',

    # scan_top=5 
    (5, 1): 'rotate_x_neg90',
    (5, 3): 'rotate_x_neg90',
    (5, 4): 'rotate_x_neg90',
    (5, 6): 'rotate_x_neg90',

    # scan_top=6
    (6, 2): 'flip_x_180',
    (6, 3): 'rotate_y_neg90',
    (6, 4): 'rotate_y_pos90',
    (6, 5): 'none',
}

# ── Dispatch ────────────────────────────────────────────────────────────────────
_TABLES = {1: PIP1_LOOKUP, 2: PIP2_LOOKUP, 3: PIP3_LOOKUP,
           4: PIP4_LOOKUP, 5: PIP5_LOOKUP, 6: PIP6_LOOKUP}


def get_action(target_pip: int, scan_top: int, scan_after_xflip: int):
    """Return the action string for (target_pip, scan_top, scan_after_xflip), or None."""
    table = _TABLES.get(target_pip)
    if table is None:
        return None
    return table.get((scan_top, scan_after_xflip))
