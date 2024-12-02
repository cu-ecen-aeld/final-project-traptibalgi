
##############################################################
#
# AESD-APP
#
##############################################################

#TODO: Fill up the contents below in order to reference your assignment 3 git contents
AESD_SERVER_VERSION = '4128939002f6fd16047b796859e1b1d41593d2b1'
# Note: Be sure to reference the *ssh* repository URL here (not https) to work properly
# with ssh keys and the automated build/test system.
# Your site should start with git@github.com:
AESD_SERVER_SITE = 'git@github.com:cu-ecen-aeld/final-project-traptibalgi.git'
AESD_SERVER_SITE_METHOD = git

define AESD_SERVER_BUILD_CMDS
	$(MAKE) $(TARGET_CONFIGURE_OPTS) -C $(@D)/simple-capture all
endef

# TODO add your writer, finder and finder-test utilities/scripts to the installation steps below
define AESD_SERVER_INSTALL_TARGET_CMDS
	$(INSTALL) -m 0755 $(@D)/simple-capture/simple-capture $(TARGET_DIR)/home/server
endef

$(eval $(generic-package))