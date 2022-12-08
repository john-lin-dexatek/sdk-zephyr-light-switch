# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(cm3218.c)
zephyr_library_sources_ifdef(CONFIG_CM3218_TRIGGER cm3218_trigger.c)
