# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(cm32183.c)
zephyr_library_sources_ifdef(CONFIG_CM32183_TRIGGER cm32183_trigger.c)
