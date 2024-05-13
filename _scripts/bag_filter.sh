#!/bin/bash
set -e


SH_DIR=$(dirname $0)


function help()
{
    echo -e "usage:"
    echo -e "./src/_scripts/$(basename $0)"
    echo -e "./src/_scripts/$(basename $0) -i ./bag/xxx.bag"
}


while [ -n "$1" ]
do
    case "$1" in
        --input|-i) 
            BAG_IN=$2
            shift
            ;;
        --help|-h) 
            help
            exit 1
            ;;
        *) echo "[-] Parameters of illegal !!!"
            help
            exit 1
            ;;
    esac
    shift
done


if [[ -z ${BAG_IN} ]]
then
    echo -e "\e[31mParameters invalid !!!\e[0m"
    exit 1
fi
echo "BAG_IN=${BAG_IN}"


rosbag filter ${BAG_IN} ${BAG_IN%%.*}_noimg.bag \
    "(topic != '/image_ksj_lf') and (topic != '/image_ksj_sf') and (topic != '/bs_debuger/image_res')"