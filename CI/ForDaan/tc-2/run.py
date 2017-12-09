#! /usr/bin/env python3

from pytocl.main import main
from my_driver import MyDriver

if __name__ == '__main__':
    main(MyDriver('evonets/trained', 'net_cruise_to_win_5batch_24epc', logdata=False))
