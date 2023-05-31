IMAGE_NAME=epfl-lasa/franka-lightweight-interface
IMAGE_TAG=runtime
BUILD_FLAGS=()

HELP_MESSAGE="Usage: ./run-rt.sh
Build and run a runtime docker container with realtime permissions.
Options:
  -r, --rebuild            Rebuild the image using the docker
                           --no-cache option.
  -v, --verbose            Use the verbose option during the building
                           process.
  -h, --help               Show this help message."

BUILD_FLAGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    -r|--rebuild) BUILD_FLAGS+=(--no-cache); shift 1;;
    -v|--verbose) BUILD_FLAGS+=(--progress=plain); shift 1;;
    -h|--help) echo "${HELP_MESSAGE}"; exit 0;;
    *) echo "Unknown option: $1" >&2; echo "${HELP_MESSAGE}"; exit 1;;
  esac
done

BUILD_FLAGS+=(--target "${IMAGE_TAG}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}":"${IMAGE_TAG}")

# !!! Since rebuilding lead to errors, these lines are removed.
# docker pull ghcr.io/aica-technology/network-interfaces
# DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" . || exit 1

docker run -it --rm --privileged --cap-add=SYS_NICE \
  --net=host \
  --ulimit rtprio=99:99 \
  --ulimit memlock=102400:102400 \
  "${IMAGE_NAME}:${IMAGE_TAG}"
