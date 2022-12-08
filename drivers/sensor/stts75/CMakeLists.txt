# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(stts75.c)
zephyr_library_sources_ifdef(CONFIG_CM3218_TRIGGER stts75_trigger.c)
