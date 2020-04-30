#!/bin/bash
ret_val=0
for d in */ ; do
    diff="$(git diff FETCH_HEAD^ FETCH_HEAD --stat --name-only -- "$d")"
    if [[ $diff ]] && [[ $diff != *"CHANGELOG.rst"* ]]; then
        echo -e "\e[31m[Fail]\e[0m No changelog adaptions in "$d" despite file changes"
        echo "Inside $d you changed"
        echo "$diff"
        echo "but not did not adapt the CHANGELOG.rst. Please fix this!"
        ret_val=1
    else
        echo -e "\e[32m[Passed]\e[0m Correct changelog adaptions in "$d" "
    fi;
    echo "=================================="
done
exit $ret_val