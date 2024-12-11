
##############################################################
#
# AESD-APP
#
##############################################################

AESD_SERVER_VERSION = '303c57a621f834ab0370fae6ed9321cf006b1e4d'
# Note: Be sure to reference the *ssh* repository URL here (not https) to work properly
# with ssh keys and the automated build/test system.
# Your site should start with git@github.com:
AESD_SERVER_SITE = 'git@github.com:cu-ecen-aeld/final-project-traptibalgi.git'
AESD_SERVER_SITE_METHOD = git

define AESD_SERVER_BUILD_CMDS
	$(MAKE) $(TARGET_CONFIGURE_OPTS) -C $(@D)/server all
endef

define AESD_SERVER_INSTALL_TARGET_CMDS
	$(INSTALL) -m 0755 $(@D)/server/capture $(TARGET_DIR)/home/server
endef

$(eval $(generic-package))
