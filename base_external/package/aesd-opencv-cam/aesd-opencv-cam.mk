
##############################################################
#
# AESD-APP
#
##############################################################

#TODO: Fill up the contents below in order to reference your assignment 3 git contents
AESD_OPENCV_CAM_VERSION = '32a9be5954e85845a56c2677c7edd4f034909aa6'
# Note: Be sure to reference the *ssh* repository URL here (not https) to work properly
# with ssh keys and the automated build/test system.
# Your site should start with git@github.com:
AESD_OPENCV_CAM_SITE = 'git@github.com:chwe3468/cu-ecen-5013-final-project-shared.git'
AESD_OPENCV_CAM_SITE_METHOD = git

define AESD_OPENCV_CAM_BUILD_CMDS
	$(MAKE) $(TARGET_CONFIGURE_OPTS) -C $(@D)/Chutao/aesd-cam all
endef

# TODO add your writer, finder and finder-test utilities/scripts to the installation steps below
define AESD_OPENCV_CAM_INSTALL_TARGET_CMDS
	$(INSTALL) -m 0755 $(@D)/Chutao/aesd-cam/capture $(TARGET_DIR)/home
endef

$(eval $(generic-package))
