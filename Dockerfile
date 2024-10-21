# Multi-stage build of an arch image that will be able to set up the build environment for CI testing.
# The environment needs:
#	arm-none-eabi-12.3.rel1 ARM GNU Toolchain for gcc compiler
#	pyenv and Python 3.8.X for GCC to run properly
#	make to compile the code
#	cppcheck to static analyze the code
#	STM32CubeProg to flash the code onto the microcontroller
# 
# Link to the correct ARM GNU Toolchain:
#	https://developer.arm.com/-/media/Files/downloads/gnu/12.3.rel1/binrel/arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi.tar.xz
#
# When pushing the docker images to the repo on docker hub, the name of the image needs to match the name of the repo.
# My repo on docker hub is named "sionnix/mini_sumobot"
# When testing the Dockerfile I used the name "archtest" for the built image.
# Run "docker tag archtest2 sionnix/mini_sumobot:latest" to change the name or "tag" of the image to the same as the remote repo
# Run "push sionnix/mini_sumobot:latest" to push that image to the remote repo of the same name

# Arch complains about a missing secret-key if we don't run pacman-key --init before running pacman -Syu
#
# The whole reason we need to do a multi-stage build is because installing yay traditionally requires the command makepkg -si.
# This is however not possible when running as root user and it errors out. The other solution is to use sudo commands
# with a wheel user but that is also cumbersome setting up permissions and editing the /etc/sudoers file which NEEDS
# to be edited with the visudo command.
#	
# The solution then is to build the yay package as a non-root non-sudo user by copying over the .pkg.tar file from the first
# image to the second. This is done to optimize image size to a minimum and then use pacman -U on the .pkg.tar file as root to
# successfully install yay on the system. Circumventing the headache of editing /etc/sudoers and root-user block on makepkg -si
#
# We need to run all package downloading commands into one RUN command because otherwise, you can't delete the build artifacts
# retroactively to optimize image size. The layers are immutable so download, build, and install all packages you need and clean
# it up afterwards on the same layer.
#
# Use archlinux:latest in stage 2 to further optimize image size because the archlinux:base-devel is basically 2x the size
# of the latest image. Use --noconfirm to skip all the prompts when automating the image creating process.
# pacman -Syu to update the system
# pacman -U to install a package with dependencies 
FROM archlinux:base-devel
RUN pacman-key --init &&\
	pacman -Syu --noconfirm git &&\
	useradd -m build_user
USER build_user
RUN cd $HOME &&\
	git clone https://aur.archlinux.org/yay-bin.git &&\
	cd yay-bin &&\
	makepkg

# Stage 2
FROM archlinux:latest
COPY --from=0 /home/build_user/yay-bin/yay-bin-*.pkg.* ./
RUN pacman-key --init && \
	pacman -Syu --noconfirm make cppcheck wget sudo &&\
	pacman -U --noconfirm yay-bin-*.pkg.* &&\
	rm -rf /var/lib/pacman/sync &&\
	rm -rf /var/cache/pacman/pkg &&\
	rm -rf yay-bin-*.pkg.* &&\
	useradd -m -G wheel arch &&\
	echo "%wheel ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers
USER arch
RUN cd $HOME && \
	wget https://developer.arm.com/-/media/Files/downloads/gnu/12.3.rel1/binrel/arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi.tar.xz &&\
	tar xvf arm*.tar.* &&\
	rm -rf arm*.tar.* &&\
	sudo mv ./arm-gnu-* /opt &&\
	cd /opt/arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi/arm-none-eabi/lib/thumb/ &&\
	sudo rm -rf v8* &&\
	sudo rm -rf v6*
CMD /bin/bash
