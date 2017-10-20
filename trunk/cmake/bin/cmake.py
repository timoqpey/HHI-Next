#!/usr/bin/env python
#
# cmake.py
#

import pyhhi.build.common.util
import pyhhi.build.app.cmk


app = pyhhi.build.app.cmk.CmakeLauncherApp()
pyhhi.build.common.util.exec_main_default_try(app)
