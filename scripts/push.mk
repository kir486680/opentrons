# utilities for pushing things to robots in a reusable fashion

find_robot=$(shell yarn run -s discovery find -i 169.254)
default_ssh_key := ~/.ssh/robot_key
default_ssh_opts := -o stricthostkeychecking=no -o userknownhostsfile=/dev/null
is-ot3 = $(shell ssh $(if $(2),"-i $(2)") $(3) root@$(1) systemctl status opentrons-robot-app)
# make version less than 4.4 do not use intcmp
allowed-ssh-versions="1 2 3 4 5 6 7 8"
# in order to use comma in a string we have to set it to a var
comma=,
# string manipulations to extract the int version number
ssh-version-output = $(shell ssh -V 2>&1)
ssh-version-words=$(subst _, ,$(filter OpenSSH_%, $(ssh-version-output)))
ssh-version-label=$(or $(filter %p1$(comma),$(ssh-version-words)), $(filter %p1,$(ssh-version-words)))
ssh-version-number=$(subst ., ,$(firstword $(subst p, ,$(ssh-version-label))))
checked-ssh-version=$(if $(ssh-version-number),$(ssh-version-number),"$(warning Could not find ssh version for version $(ssh-version-output), scp flags may be wrong")))
is-in-version=$(findstring $(firstword $(checked-ssh-version)),$(allowed-ssh-versions))
# when using an OpenSSH version larger than 8.9,
# we need to add a flag to use legacy scp with SFTP protocol
scp-legacy-option-flag = $(if $(is-in-version),,-O)
# when using windows, make is running against a different openSSH than the OS. 
# adding the -O flag to scp will fail if the openSSH on OS is less than 9.
# if openSSH on OS is 9 or more please add the -O flag to scp.
PLATFORM := $(shell uname -s)
is-windows=$(findstring $(PLATFORM), Windows)
$(if $(is-windows), echo "when using windows with an openSSH version larger then 9 add -O flag to scp command. see comments for more details")

# push-python-package: execute a push to the robot of a particular python
# package.
#
# argument 1 is the host to push to
# argument 2 is the identity key to use
# argument 3 is any further ssh options, quoted
# argument 4 is the path to the wheel file

define push-python-package
$(if $(is-ot3), echo "This is an OT-3. Use 'make push-ot3' instead." && exit 1)
scp -i $(2) $(scp-legacy-option-flag) $(3) "$(4)" root@$(1):/data/$(notdir $(4))
ssh -i $(2) $(3) root@$(1) \
"function cleanup () { rm -f /data/$(notdir $(4)) && mount -o remount,ro / ; } ;\
mount -o remount,rw / &&\
cd /usr/lib/python3.7/site-packages &&\
unzip -o /data/$(notdir $(4)) && cleanup || cleanup"
endef

# push-python-sdist: push an sdist to an ot3
# argument 1 is the host to push to
# argument 2 is the identity key to use, if any
# argument 3 is any further ssh options, quoted
# argument 4 is the path to the sdist locally
# argument 5 is the path to go to on the remote side
# argument 6 is the python package name
# argument 7 is an additional subdir if necessary in the sdist
# argument 8 is either egg or dist (default egg)
define push-python-sdist
$(if $(is-ot3), ,echo "This is an OT-2. Use 'make push' instead." && exit 1)
scp $(if $(2),"-i $(2)") $(scp-legacy-option-flag) $(3) $(4) root@$(1):/var/$(notdir $(4))
ssh $(if $(2),"-i $(2)") $(3) root@$(1) \
"function cleanup () { rm -f /var/$(notdir $(4)) ; rm -rf /var/$(notdir $(4))-unzip; mount -o remount,ro / ; } ;\
 mkdir -p /var/$(notdir $(4))-unzip ; \
 cd /var/$(notdir $(4))-unzip && tar xf ../$(notdir $(4)) ; \
 mount -o remount,rw / ; \
 rm -rf $(5)/$(6) $(5)/$(6)*.egg-info ; \
 mv /var/$(notdir $(4))-unzip/$(basename $(basename $(notdir $(4))))/$(if $(7),$(7)/)$(6) $(5)/ ; \
 mv /var/$(notdir $(4))-unzip/$(basename $(basename $(notdir $(4))))/$(if $(7),$(7)/)$(6)*.$(if $(8),$(8),egg)-info $(5)/$(basename $(basename $(notdir $(4)))).$(if $(8),$(8),egg)-info ; \
 cleanup \
 "
endef

# restart-service: ssh to a robot and restart one of its systemd units
#
# argument 1 is the host to push to
# argument 2 is the identity key to use
# argument 3 is any further ssh options, quoted
# argument 4 is the service name

define restart-service
ssh -i "$(2)" $(3)  root@$(1) \
"systemctl restart $(4)"
endef

# push-systemd-unit: move a systemd unit file to the robot
# 
# argument 1 is the host to push to
# argument 2 is the identity key to use
# argument 3 is any further ssh options, quoted
# argument 4 is the unit file path
define push-systemd-unit
	scp -i $(2) $(scp-legacy-option-flag) $(3) "$(4)" root@$(1):/data/
	ssh -i $(2) $(3) root@$(1) "mount -o remount,rw / && mv /data/$(notdir $(4)) /etc/systemd/system/ && systemctl daemon-reload && mount -o remount,ro / || mount -o remount,ro /"
endef
