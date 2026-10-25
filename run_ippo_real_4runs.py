#!/usr/bin/env python3
"""Run IPPO on real four times at 0.70 Ah, with 30 seconds between runs."""
from run_ippo_8runs import main


if __name__ == '__main__':
    main(plan=[('real', repeat) for repeat in range(1, 5)],
         battery_capacity='0.70', batch_folder='ippo_real_batches', description=__doc__)
