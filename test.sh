#!/bin/bash

# 여기는 건들지 말아주세요
execute_with_timeout() {
    local command_to_run="$1"
    # Run the command with a 5-minute timeout
    timeout 300 $command_to_run
    cmd_status=$?
    if [ $cmd_status -eq 124 ]; then
        echo "Timeout! Command '$command_to_run' was forcibly terminated."
    elif [ $cmd_status -ne 0 ]; then
        echo "Unexpected termination! Command '$command_to_run' exited with status $cmd_status."
    else
        echo "Success! Command '$command_to_run' executed successfully."
    fi
}

# 여기에 세빈이형 테스트 내용 넣으면 돼요
ENVIRONMENTS=("OpenEnv")
VARIABLES=(20 25 30)
BASE_COMMAND="./build/SI-CBS"

# 여기는 건들지 말아주세요
#for env in "${ENVIRONMENTS[@]}"; do
#    for var1 in "${VARIABLES[@]}"; do
#        for var2 in {0..49}; do
#            execute_with_timeout "$BASE_COMMAND $env $var1 $var2"
#        done
#    done
#done

for test_num in {0..49}; do
  execute_with_timeout "$BASE_COMMAND -n $test_num"
done