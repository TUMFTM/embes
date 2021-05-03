import sys


def unpack(msg_in: dict):
    """Unpack message input `msg_in` and check given information for inconsistency.

    :param msg_in:  Input format:   Character (which ES phase,  'v' - calculate v_ref
                                                                'i' - initialize the ES module
                                                                'r' - re-optimize the ES strategy)
                                    Characters (3, race track ID,   'mnt' - Monteblanco
                                                                    'mod' - Modena
                                                                    'prs' - Paris
                                                                    'ber' - Berlin
                                                                    'hok' - Hong_Kong
                                                                    'upp' - Upper_Heyford)
                                    Integer (number of race laps)

    :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

    :Created on:
        01.02.2021
    """

    if msg_in['phase'] == 'v':
        if not {'track'} <= msg_in.keys():
            print('[ERROR] No track ID specified. Exiting.')
            sys.exit(1)
    elif msg_in['phase'] == 'i':
        if not {'track', 'num_laps', 'x0'} <= msg_in.keys():
            print('[ERROR] Not all infos of "track-ID", "num_laps" or "x0" provided. Exiting.')
            sys.exit(1)
    elif msg_in['phase'] == 'r':
        if not {'s_meas', 'meas_diff'} <= msg_in.keys():
            print('[ERROR] No measurement difference and coordinate provided. Exiting.')
            sys.exit(1)

    return 0
