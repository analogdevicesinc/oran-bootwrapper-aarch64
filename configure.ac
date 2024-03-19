# configure.ac - autoconf script for the AArch64 bootwrapper
#
# Copyright (c) 2014 ARM Limited. All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can
# be found in the LICENSE.txt file.

AC_INIT([boot-wrapper], [v0.2])

AM_INIT_AUTOMAKE([foreign])

AC_ARG_ENABLE([aarch32-bw],
	      AS_HELP_STRING([--enable-aarch32-bw], [build a 32-bit boot-wrapper]),
	      [BOOTWRAPPER_ES=32], [BOOTWRAPPER_ES=64])
AM_CONDITIONAL([BOOTWRAPPER_32], [test "x$BOOTWRAPPER_ES" = "x32"])

AC_ARG_ENABLE([aarch32-kernel],
	AC_HELP_STRING([--enable-aarch32-kernel],
		       [start kernel in AArch32 instead of AArch64]),
	[KERNEL_ES=32 USE_PSCI=yes AC_SUBST([KERNEL_32], [1]) AC_DEFINE([KERNEL_32])],
	[KERNEL_ES=64])
AM_CONDITIONAL([KERNEL_32], [test "x$KERNEL_ES" = x32])

AS_IF([test "x$BOOTWRAPPER_ES" = x32 -a "x$KERNEL_ES" != x32],
	[AC_MSG_ERROR([a 32-bit boot-wrapper cannot launch a 64-bit kernel])]
)

# Allow a user to pass --enable-dynamic-config
AC_ARG_ENABLE([dynamic-config],
        AS_HELP_STRING([--enable-dynamic-config], [enable dynamic configuration of kernel, etc.]),
        [USE_DYNAMIC_CONFIG=$enableval])
AM_CONDITIONAL([DYNAMIC_CONFIG], [test "x$USE_DYNAMIC_CONFIG" = "xyes"])
AS_IF([test "x$USE_DYNAMIC_CONFIG" = "xyes"], [], [USE_DYNAMIC_CONFIG=no])

# Allow a user to pass --enable-adi-platform
AC_ARG_ENABLE([adi-platform],
        AS_HELP_STRING([--enable-adi-platform], [enable ADI platform, instead of default Arm platform]),
        [USE_ADI_PLATFORM=$enableval])
AM_CONDITIONAL([ADI_PLATFORM], [test "x$USE_ADI_PLATFORM" = "xyes"])
AS_IF([test "x$USE_ADI_PLATFORM" = "xyes"], [], [USE_ADI_PLATFORM=no])

# Allow a user to pass --with-kernel-dir. Only required if USE_DYNAMIC_CONFIG isn't specified.
KERN_DIR=""
AC_ARG_WITH([kernel-dir],
	AS_HELP_STRING([--with-kernel-dir], [specify the root Linux kernel build directory (required)]),
	AC_SUBST([KERN_DIR], [$withval]))
AS_IF([test "x$USE_DYNAMIC_CONFIG" = "xyes"], [],
      [AS_IF([test "x$KERN_DIR" = "x"], [AC_MSG_ERROR([No kernel directory specified. Use --with-kernel-dir])])])
AS_IF([test "x$KERNEL_ES" = x32],
	[KERN_IMAGE=$KERN_DIR/arch/arm/boot/zImage],
	[KERN_IMAGE=$KERN_DIR/arch/arm64/boot/Image])
KERN_DTB=$KERN_DIR/arch/arm64/boot/dts/arm/fvp-base-revc.dtb
AS_IF([test "x$USE_DYNAMIC_CONFIG" = "xyes"], [KERN_IMAGE=""], [])

# Allow the user to override the default DTB
AC_ARG_WITH([dtb],
	AS_HELP_STRING([--with-dtb], [Specify a particular DTB to use]),
	[KERN_DTB="$withval"])

AC_ARG_WITH([xen],
	AS_HELP_STRING([--with-xen], [Compile for Xen, and specify a particular Xen to use]),
	X_IMAGE=$withval)

AS_IF([test "x$X_IMAGE" == "x"], [],
      [AS_IF([test ! -f "$X_IMAGE"],
	     [AC_MSG_ERROR([Could not find Xen hypervisor binary: $X_IMAGE])]
      )]
)
AC_SUBST([XEN_IMAGE], [$X_IMAGE])
AM_CONDITIONAL([XEN], [test "x$X_IMAGE" != "x"])

# Ensure that the user has provided us with a sane kernel dir. Only required if USE_DYNAMIC_CONFIG isn't specified.
AS_IF([test "x$USE_DYNAMIC_CONFIG" == "xyes"], [],
     [AS_IF([test ! -d "$KERN_DIR"], [AC_MSG_ERROR([Could not find Linux kernel dir: $KERN_DIR.])])])

AC_MSG_CHECKING([whether DTB file exists])
if ! test -f $KERN_DTB; then
	AC_MSG_RESULT([no])
	AC_MSG_ERROR([Could not find DTB file: $KERN_DTB])
else
	AC_MSG_RESULT([yes])
fi

AC_MSG_CHECKING([whether kernel image exists])
if ! test -f $KERN_IMAGE; then
	AC_MSG_RESULT([no])
else
	AC_MSG_RESULT([yes])
fi
AS_IF([test "x$USE_DYNAMIC_CONFIG" == "xyes"], [],
     [AS_IF([test ! -f "$KERN_IMAGE"], [AC_MSG_ERROR([Could not find kernel image: $KERN_IMAGE])])])

AC_SUBST([KERNEL_IMAGE], [$KERN_IMAGE])
AC_SUBST([KERNEL_DTB], [$KERN_DTB])
AS_IF([test "x$X_IMAGE" != "x"],
      [AC_SUBST([IMAGE], ["xen-system.axf"])],
      [AC_SUBST([IMAGE], ["linux-system.axf"])]
)

# Allow a user to pass --enable-psci
AC_ARG_ENABLE([psci],
	AS_HELP_STRING([--disable-psci], [disable the psci boot method]),
	[USE_PSCI=$enableval], [USE_PSCI="yes"])
AM_CONDITIONAL([PSCI], [test "x$USE_PSCI" = "xyes"])
AS_IF([test "x$USE_PSCI" = "xyes"], [], [USE_PSCI=no])

AS_IF([test "x$USE_PSCI" != "xyes" -a "x$KERNEL_ES" = "x32"],
	[AC_MSG_ERROR([With an AArch32 kernel, boot method must be PSCI.])]
)

# Allow a user to pass --with-initrd
AC_ARG_WITH([initrd],
	AS_HELP_STRING([--with-initrd], [embed an initrd in the kernel image]),
	USE_INITRD=$withval)
AC_SUBST([FILESYSTEM], [$USE_INITRD])
AM_CONDITIONAL([INITRD], [test "x$USE_INITRD" != "x"])

AS_IF([test "x$X_IMAGE" = "x"],[C_CONSOLE="ttyAMA0"],[C_CONSOLE="hvc0"])
C_CMDLINE="console=$C_CONSOLE earlycon=pl011,0x1c090000"
AC_ARG_WITH([cmdline],
	AS_HELP_STRING([--with-cmdline], [set a command line for the kernel]),
	[C_CMDLINE=$withval])
AC_SUBST([CMDLINE], [$C_CMDLINE])

X_CMDLINE="console=dtuart dtuart=serial0 no-bootscrub"
AC_ARG_WITH([xen-cmdline],
	AS_HELP_STRING([--with-xen-cmdline], [set Xen command line]),
	[X_CMDLINE=$withval])
AC_SUBST([XEN_CMDLINE], [$X_CMDLINE])

# Allow a user to pass --enable-gicv3
AC_ARG_ENABLE([gicv3],
	AS_HELP_STRING([--enable-gicv3], [enable GICv3 instead of GICv2]),
	[USE_GICV3=$enableval])
AM_CONDITIONAL([GICV3], [test "x$USE_GICV3" = "xyes"])
AS_IF([test "x$USE_GICV3" = "xyes"], [], [USE_GICV3=no])

# Allow a user to pass --enable-gic600
AC_ARG_ENABLE([gic600],
        AS_HELP_STRING([--enable-gic600], [enable GIC600]),
        [USE_GIC600=$enableval])
AM_CONDITIONAL([GIC600], [test "x$USE_GIC600" = "xyes"])
AS_IF([test "x$USE_GIC600" = "xyes"], [], [USE_GIC600=no])

# Ensure that we have all the needed programs
AC_PROG_CC
AC_PROG_CPP
AM_PROG_AS
AC_PROG_SED
AC_PROG_LN_S
AC_PATH_PROG([DTC], dtc, error, [$PATH$PATH_SEPARATOR$KERN_DIR/scripts/dtc])
if test "x$DTC" = "xerror"; then
	AC_MSG_ERROR([cannot find the device tree compiler (dtc)])
fi
AC_CHECK_TOOL(LD, ld)

AC_CONFIG_FILES([Makefile])

AC_OUTPUT

# Print the final config to the user.
echo ""
echo "  Boot wrapper configuration"
echo "  =========================="
echo ""
echo "  Linux kernel build dir:            ${KERN_DIR:-NONE}"
echo "  Device tree blob:                  ${KERN_DTB}"
echo "  Linux kernel command line:         ${CMDLINE}"
echo "  Embedded initrd:                   ${FILESYSTEM:-NONE}"
echo "  Use PSCI?                          ${USE_PSCI}"
echo "  Use GICv3?                         ${USE_GICV3}"
echo "  Boot-wrapper execution state:      AArch${BOOTWRAPPER_ES}"
echo "  Kernel execution state:            AArch${KERNEL_ES}"
echo "  Xen image                          ${XEN_IMAGE:-NONE}"
if test "x${XEN_IMAGE}" != "x"; then
echo "  Xen command line:                  ${XEN_CMDLINE}"
fi
echo ""
