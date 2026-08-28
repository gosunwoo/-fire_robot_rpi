"""Parse terminal waypoint lists and validate the robot drive modes."""

import re


_WAYPOINT_PATTERN = re.compile(r'w(0*[1-9][0-9]*)', re.IGNORECASE)


def command_source_for_drive_mode(mode):
    """Map mode 1 to keyboard and modes 2 through 6 to autonomous velocity."""
    sources = {1: 1, 2: 2, 3: 2, 4: 2, 5: 2, 6: 2}
    try:
        return sources[int(mode)]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('drive mode must be 1, 2, 3, 4, 5, or 6') from error


def parse_named_waypoints(text, minimum_count=2):
    """Return canonical labels such as ``w1`` and ``w25``.

    Commas and whitespace may be mixed. A mission needs at least two
    destinations so an accidental single label cannot start the robot.
    """
    if not isinstance(text, str):
        raise ValueError('waypoint list must be text')
    tokens = [token for token in re.split(r'[\s,]+', text.strip()) if token]
    if len(tokens) < minimum_count:
        raise ValueError(
            f'enter at least {minimum_count} waypoints (example: w1,w5)'
        )
    labels = []
    for token in tokens:
        match = _WAYPOINT_PATTERN.fullmatch(token)
        if match is None:
            raise ValueError(f'invalid waypoint label: {token!r}')
        labels.append(f'w{int(match.group(1))}')
    return labels
