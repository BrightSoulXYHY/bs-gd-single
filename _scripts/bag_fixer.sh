#!/bin/bash
set -e


SH_DIR=$(dirname $0)
BAG_OUT_DIR="${SH_DIR}/../../bag"
BAG_IN_DIR=${BAG_OUT_DIR}

function help()
{
    echo -e "usage:"
    echo -e "./src/_scripts/fix_bag.sh"
    echo -e "./src/_scripts/fix_bag.sh -i /media/brightsoul/YSJ_der_T7/bag"
}


while [ -n "$1" ]
do
    case "$1" in
        --input|-i) 
            BAG_IN_DIR=$2
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

echo "BAG_IN_DIR=${BAG_IN_DIR}"
echo "BAG_OUT_DIR=${BAG_OUT_DIR}"



for file in `ls ${BAG_IN_DIR}| grep .bag.active`;do
    file_name=${file%%.*}
    dir_name=$(dirname $file)
    echo ${BAG_IN_DIR}/${file_name}.bag.active
    rosbag reindex --output-dir=${BAG_OUT_DIR} ${BAG_IN_DIR}/${file_name}.bag.active
    pushd ${BAG_OUT_DIR}
        rosbag fix ${file_name}.bag.active ${file_name}_fixed.bag
    popd
done
