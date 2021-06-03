#!/bin/bash

# set -x

SOURCE_DIR=`pwd`

POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -m|--module)
    MODULE=$2
    shift # past argument
    shift # past value
    ;;
    -t|--type)
    TYPE=$2
    shift # past argument
    shift # past value
    ;;
    -p|--platform)
    PLATFORM=$2
    shift # past argument
    shift # past value
    ;;
    -e|--example)
    EXAMPLE=1
    shift # past argument
    shift # past value
    ;;
    -r|--release)
    RELEASE=$2
    shift # past argument
    shift # past value
    ;;
    *)    # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters

BUILD_DIR=${DIR:-./build}
BUILD_MODULE=${MODULE}
BUILD_TYPE=${TYPE:-release}
# BUILD_PLATFORM=${PLATFORM:-x86_64}
BUILD_EXAMPLE=${EXAMPLE:-0}
BUILD_RELEASE=${RELEASE:-1}

if [ "$PLATFORM" = "" ]
then
    BUILD_PLATFORM[0]=x86_64
elif [ "$PLATFORM" = "all" ]
then
    BUILD_PLATFORM[0]="x86_64"
    BUILD_PLATFORM[1]="arm"
    BUILD_PLATFORM[2]="ros"
else
    BUILD_PLATFORM[0]=$PLATFORM
fi

for PF in "${BUILD_PLATFORM[@]}"
do
    mkdir -p $BUILD_DIR/$PF/$BUILD_TYPE \
        && cd $BUILD_DIR/$PF/$BUILD_TYPE \
        && cmake \
            $SOURCE_DIR \
            -DBUILD_MODULE=$BUILD_MODULE  \
            -DBUILD_TYPE=$BUILD_TYPE \
            -DBUILD_PLATFORM=$PF    \
            -DBUILD_EXAMPLE=$BUILD_EXAMPLE  \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes \
        && make -j4

    if [ $? = 0 ] && [ $BUILD_RELEASE = 1 ] && [ ! "$BUILD_MODULE" = "all" ]
    then
        cd $SOURCE_DIR  \
        && scripts/release_to_planning.sh -m $BUILD_MODULE    \
                                            -t $BUILD_TYPE      \
                                            -p $PF
    fi

    if [ ! $? = 0 ]
    then
        exit 1
    fi
done
