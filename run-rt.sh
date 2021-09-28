IMAGE_NAME=epfl-lasa/franka-lightweight-interface
IMAGE_TAG=runtime
BUILD_FLAGS=()

while getopts 'r' opt; do
  case $opt in
    r) BUILD_FLAGS+=(--no-cache) ;;
    *) echo 'Error in command line parsing' >&2
       exit 1
  esac
done
shift "$(( OPTIND - 1 ))"

BUILD_FLAGS+=(--target "${IMAGE_TAG}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}":"${IMAGE_TAG}")

docker pull ghcr.io/epfl-lasa/control-libraries/development-dependencies
DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" . || exit 1

docker run -it --rm --privileged --cap-add=SYS_NICE \
  --net=host \
  --ulimit rtprio=99:99 \
  --ulimit memlock=102400:102400 \
  "${IMAGE_NAME}:${IMAGE_TAG}"