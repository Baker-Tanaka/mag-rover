FROM rust:1.94.1-bookworm

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y --no-install-recommends \
	ca-certificates \
	curl \
	jq \
	libudev-dev \
	pkg-config \
	sudo \
	&& rm -rf /var/lib/apt/lists/* \
	&& groupadd --gid ${USER_GID} ${USERNAME} \
	&& useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} \
	&& echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
	&& chmod 0440 /etc/sudoers.d/${USERNAME}
RUN rustup target install thumbv6m-none-eabi
RUN cargo install flip-link --version 0.1.12
RUN rustup component add rustfmt
RUN rustup component add llvm-tools
RUN cargo install cargo-binutils
RUN cargo install elf2uf2-rs --locked

# Make root-created Cargo caches writable by the development user.
RUN chown -R ${USERNAME}:${USERNAME} ${CARGO_HOME}

USER ${USERNAME}
