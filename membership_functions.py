"""Membership functions module."""


def i_neg(init, end, value):
    """Docstring."""
    if value < init:
        return 1
    elif value > end:
        return 0
    else:
        return (value - end)/(init - end)


def i_pos(init, end, value):
    """Docstring."""
    if value < init:
        return 0
    elif value > end:
        return 1
    else:
        return ((value - end)/(end - init) + 1)
