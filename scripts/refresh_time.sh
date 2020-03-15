#!/bin/bash

/etc/init.d/chrony stop
ntpdate other_computer_ip
/etc/init.d/chrony start
