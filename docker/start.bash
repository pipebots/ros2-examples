#!/bin/bash
# Start the docker container.
# $1 can be used to pass in a different code directory.
# set -x

docker_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
. ${docker_dir}/vars.bash
mkdir -p ${WORKSPACE_DIR}
cp ${docker_dir}/setup_*.bash ${WORKSPACE_DIR}

docker container inspect ${CONTAINER_NAME} &> /dev/null
if [ $? == 0 ]
then
    # Container exists.
    if [ "$( docker container inspect -f '{{.State.Status}}' ${CONTAINER_NAME} )" == "running" ]
    then
        # Container is running.
        echo "Container '${CONTAINER_NAME}' is already running."
    else
        # Container exists but is not running.
        docker container start ${CONTAINER_NAME} &> /dev/null
        echo "Container '${CONTAINER_NAME}' started."
    fi
else
    # Container does not exist so run it.
    CODE_DIR=${docker_dir}/..
    docker container run \
        --detach \
        --tty \
        --net=host \
        --name ${CONTAINER_NAME} \
        --volume ${CODE_DIR}:/home/build/code \
        --volume ${WORKSPACE_DIR}:/home/build/ws \
        ${DOCKER_HUB_USER_NAME}/${IMAGE_NAME}:${IMAGE_TAG} &> /dev/null
    if [ $? == 0 ]
    then
        echo "Container '${CONTAINER_NAME}' running."
    else
        echo "Container '${CONTAINER_NAME}' failed."
    fi
fi
