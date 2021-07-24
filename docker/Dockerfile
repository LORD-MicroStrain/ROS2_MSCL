FROM ros:galactic-ros-base-focal

# Install MSCL (github deb install is kind of a pain)
ENV DEBIAN_FRONTEND="noninteractive"
RUN set -ex \
    && apt-get update && apt-get install -y \
        jq \
        curl \
        libboost-dev \
    && install_github_deb () { \
        org="${1}"; \
        repo="${2}"; \
        tag="${3}"; \
        artifact="${4}"; \
        list_asset_url="https://api.github.com/repos/${org}/${repo}/releases/tags/${tag}"; \
        asset_url=$(curl -fsSl "${list_asset_url}" | jq ".assets[] | select(.name==\"${artifact}\") | .url" | sed 's/\"//g'); \
        curl -fsSLJ -H 'Accept: application/octet-stream' "${asset_url}" -o "/tmp/${artifact}"; \
        dpkg -i "/tmp/${artifact}" || apt-get install -f; \
        rm -f "/tmp/${artifact}"; \
    } \
    && install_github_deb "LORD-MicroStrain"  "MSCL" "v62.0.0" "c++-mscl_62.0.0_amd64.deb"

# Add the library to LD_LIBRARY_PATH
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/share/c++-mscl"

# Use rosdep to install the dependencies of the packages
COPY ros2_mscl /tmp/src/ros2_mscl
COPY mscl_msgs /tmp/src/mscl_msgs
RUN set -ex \
    && rosdep install --from-paths /tmp/src --ignore-src -r -y \
    && rm -rf /tmp/src
