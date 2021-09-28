#!/usr/bin/env bash

IMAGE_NAME=epfl-lasa/franka-lightweight-interface
IMAGE_STAGE=remote-development
CONTAINER_NAME=franka-lightweight-interface-remote-development-ssh
CONTAINER_HOSTNAME=franka-lightweight-interface-remote-development

BUILD_FLAGS=()
SSH_PORT=1111
SSH_KEY_FILE="$HOME/.ssh/id_rsa.pub"

HELP_MESSAGE="Usage: ./dev-server.sh [--port <port>] [--key-file </path/to/id_rsa.pub>]
Build and run a docker container as an SSH toolchain server for remote development.
The server is bound to the specified port on localhost (127.0.0.1)
and uses passwordless RSA key-pair authentication. The host public key
is read from the specified key file and copied to the server on startup.
The server will run in the background as ${CONTAINER_NAME}.
You can connect with 'ssh remote@localhost -p <port>'.
Close the server with 'docker container stop ${CONTAINER_NAME}'.
Options:
  -p, --port [XXXX]        Specify the port to bind for SSH
                           connection.
                           (default: ${SSH_PORT})
  -k, --key-file [path]    Specify the path of the RSA
                           public key file.
                           (default: ${SSH_KEY_FILE})
  -r, --rebuild            Rebuild image without cache.
  -h, --help               Show this help message."

while [ "$#" -gt 0 ]; do
  case "$1" in
    -p|--port) SSH_PORT=$2; shift 2;;
    -k|--key-file) SSH_KEY_FILE=$2; shift 2;;
    -r|--rebuild) BUILD_FLAGS+=(--no-cache); shift 1;;
    -h|--help) echo "${HELP_MESSAGE}"; exit 0;;
    *) echo "Unknown option: $1" >&2; echo "${HELP_MESSAGE}"; exit 1;;
  esac
done
BUILD_FLAGS+=(--target "${IMAGE_STAGE}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}":"${IMAGE_STAGE}")

docker pull ghcr.io/epfl-lasa/control-libraries/development-dependencies
DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" . || exit 1

docker container stop "${CONTAINER_NAME}" >/dev/null 2>&1
docker rm --force "${CONTAINER_NAME}" >/dev/null 2>&1

docker run -d --cap-add sys_ptrace \
  --publish 127.0.0.1:"${SSH_PORT}":22 \
  --name "${CONTAINER_NAME}" \
  --hostname "${CONTAINER_HOSTNAME}" \
  "${IMAGE_NAME}:${IMAGE_STAGE}" "$(cat "${SSH_KEY_FILE}")"