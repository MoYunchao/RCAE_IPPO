#!/usr/bin/env python3
"""Supplement existing real-map IPPO results to five trials at 0.70 Ah."""
from run_ippo_8runs import main


if __name__ == '__main__':
    main(maps=('real',), battery_capacity='0.70',
         batch_folder='ippo_real_supplement_batches', description=__doc__)
