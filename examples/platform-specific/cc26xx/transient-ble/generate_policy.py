#!/usr/bin/env python3
# generate the policy pool file
import sys
import numpy as np
from datetime import datetime

# filename definitions
TEMPLATE_FILE = 'policy-pool.inc'
OUTPUT_FILE = 'policy-pool.h'


# check for policy file passed to the script
if len(sys.argv) > 1:
    # load policy pool from file
    POLICY_FILE = sys.argv[1]
    print('loading policy from file {:s}'.format(POLICY_FILE))
    pool = np.load(POLICY_FILE, allow_pickle=True)

    POWER_LEVELS = len(pool)
    HISTORY = list(pool.values())[0]['HISTORY']
    AGGREGATES = list(pool.values())[0]['AGGREGATES']
    BUFFER_SIZE = HISTORY + AGGREGATES
    SELECTION_SIZE = list(pool.values())[0]['SELECTION_SIZE']

    POOL_POWER = np.ones(POWER_LEVELS, dtype=np.uint32) * (2**32-1)
    POOL_DELTA = np.ones(POWER_LEVELS, dtype=np.uint16)
    POOL_CDF = np.ones([POWER_LEVELS, BUFFER_SIZE], dtype=np.uint16) * (2**16-1)
    POOL_BIN = np.ones([POWER_LEVELS, BUFFER_SIZE], dtype=np.uint8) * (2**8-1)

    # extract policies sorted by (decending) power level
    policies_sorted = sorted(pool.values(), key=lambda x: x['power'], reverse=True)
    for level, policy in enumerate(policies_sorted):
        POOL_POWER[level] = np.round(policy['power'] * 1e3)
        POOL_DELTA[level] = np.round(policy['delta'])
        for bin_index, bin_value in enumerate(policy['P_bins_deploy']):
            # sort with respect to xk-s to get proper CDFs
            xk_sorted, qpk_sorted = zip(*sorted(zip(bin_value['xk'],
                                                    bin_value['qpk']),
                                                key=lambda x: x[0]))
            # set CDF values and bin mapping
            POOL_CDF[level, xk_sorted] = np.cumsum(qpk_sorted)
            POOL_BIN[level, xk_sorted] = bin_index
else:
    # use default values
    print('using default policy')
    POLICY_FILE = '<none>'
    POWER_LEVELS = 5
    BUFFER_SIZE = 200
    AGGREGATES = 5
    SELECTION_SIZE = 4

    POOL_POWER = np.linspace(1e6 / POWER_LEVELS, 1e6, POWER_LEVELS, dtype=np.uint32)
    POOL_DELTA = np.ones(POWER_LEVELS, dtype=np.uint16)
    POOL_CDF = np.ones([POWER_LEVELS, BUFFER_SIZE], dtype=np.uint16) * (2**16-1)
    POOL_BIN = np.tile(np.arange(BUFFER_SIZE, dtype=np.uint8) % SELECTION_SIZE, (POWER_LEVELS, 1))

# generate replacement strings
np.set_printoptions(threshold=2*POWER_LEVELS*BUFFER_SIZE)
replacements = {
    'GENERATION_TIME': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
    'POLICY_FILE': POLICY_FILE,
    'POWER_LEVELS': str(POWER_LEVELS),
    'BUFFER_SIZE': str(BUFFER_SIZE),
    'AGGREGATES': str(AGGREGATES),
    'SELECTION_SIZE': str(SELECTION_SIZE),
    'POOL_POWER': np.array2string(POOL_POWER, max_line_width=80, separator=', ', prefix='  ').replace('[', '{').replace(']', '}'),
    'POOL_DELTA': np.array2string(POOL_DELTA, max_line_width=80, separator=', ', prefix='  ').replace('[', '{').replace(']', '}'),
    'POOL_CDF': np.array2string(POOL_CDF, max_line_width=80, separator=', ', prefix='  ').replace('[', '{').replace(']', '}'),
    'POOL_BIN': np.array2string(POOL_BIN, max_line_width=80, separator=', ', prefix='  ').replace('[', '{').replace(']', '}'),
}

# replace in template header file
with open(TEMPLATE_FILE, 'rt') as file_in:
    with open(OUTPUT_FILE, 'wt') as file_out:
        for line in file_in:
            for key, value in replacements.items():
                line = line.replace('${{{:s}}}'.format(key), value)
            file_out.write(line)
