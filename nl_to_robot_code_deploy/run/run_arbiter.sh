#!/bin/bash

echo $(dirname $(realpath $0))
cd $(dirname $(realpath $0))

cd /home/amrl_user/amrl/ut_jackal
./bin/autonomy_arbiter