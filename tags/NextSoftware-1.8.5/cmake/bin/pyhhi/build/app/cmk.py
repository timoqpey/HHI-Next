from __future__ import print_function

import sys
import re
import argparse
import os
import logging

import pyhhi.build.common.system as system
import pyhhi.build.common.util as util
import pyhhi.build.common.ver as ver
import pyhhi.build.common.cmksupp as cmksupp


class CmakeLauncherApp(object):

    def __init__(self):
        self._logger = logging.getLogger(__name__)
        # True validates the python installation on windows.
        self._sys_info = system.SystemInfo(True)
        self._cmake_launcher = cmksupp.CmakeLauncher()
        # self._dict_generator_choice = {'linux': ['umake'],
        self._dict_generator_choice = {'linux': ['umake', 'ninja'],
                                       'macosx': ['xcode', 'umake'],
                                       'windows': ['vs14', 'vs12', 'vs11', 'vs10', 'mgwmake']}
        self._top_dir = None

    def __call__(self):
        self.main(sys.argv[1:])

    def main(self, argv):
        # self._print_env()
        (params, cmake_argv) = self._parse_command_line(argv)
        if self._sys_info.is_linux() or self._sys_info.is_macosx():
            self._remove_make_env_vars()
        #if params.toolset_str.endswith('.cmake'):
        #    print("toolchain file: " + params.toolset_str)
        #    sys.exit(0)
        self._cmake_launcher.launch(params, cmake_argv)

    def _print_env(self):
        env_var_list = list(os.environ.keys())
        env_var_list.sort()
        for env_var in env_var_list:
            print(env_var, os.environ[env_var])
            # print(env_var)
            # if env_var.startswith('MAKE') or (env_var == 'MFLAGS'):
            #    print(env_var)
        # sys.exit(0)

    def _remove_make_env_vars(self):
        make_env_vars = []
        for env_var in os.environ.keys():
            if env_var.startswith('MAKE') or (env_var == 'MFLAGS'):
                make_env_vars.append(env_var)
        for env_var in make_env_vars:
            self._logger.debug("deleting environment variable: %s", env_var)
            del os.environ[env_var]

    def _parse_command_line(self, argv):

        _usage = """
%(prog)s [options] [variant=debug,release] [link=static,shared] [toolset=<toolset_spec>] [address-model=32] [unknown cmake options ...]

%(prog)s is a script front end to cmake to simplify its usage on Linux,
Windows, MacOSX using cmake's generators "Unix Makefiles", "Xcode" and
"Visual Studio 14 - Visual Studio 10" and its compilers.

arguments:
  variant:          debug if not specified
  link:             static if not specified
  toolset:          default c++ compiler if not specified
  address-model=32: windows: builds 32 bit binaries instead of 64 bit binaries

usage examples:

  # debug build
  %(prog)s -b

  # release build
  %(prog)s -b variant=release

  # release build using shared libraries
  %(prog)s -b variant=release link=shared

  # create a build tree without building anything
  %(prog)s

  # create a build tree specifying a cmake cache entry without building anything
  %(prog)s -DENABLE_SOMETHING=1

"""
        parser = argparse.ArgumentParser(usage=_usage)

        parser.add_argument("-g","-G", action="store", dest="generator", choices=self._dict_generator_choice[self._sys_info.get_platform()],
                            help="""specify a cmake generator the script has special support for.
                                    Supported generators: umake, vs14, vs12, vs11, vs10, xcode.
                                    The choices accepted are platform and installation dependent.""")

        parser.add_argument("-D", action="append", dest="cache_entries",
                            help="specify a cmake cache entry. The option will be ignored if a build tree already exists.")

        parser.add_argument("-W", action="append", dest="warning_flags", help="specify a cmake warning flag.")

        parser.add_argument("-b", action="store_true", dest="build", default=False,
                            help="""invokes cmake --build to build the default debug configuration unless overridden by additional arguments.
                                    If a build tree does not exist, it will be created.""")

        parser.add_argument("-j", action="store", dest="build_jobs", type=int, nargs='?', default=1,
                            const=str(self._cmake_launcher.get_optimal_number_cmake_jobs()),
                            help="""specify the number of parallel build jobs. If you omit the numeric argument,
                                    cmake --build ... will be invoked with -j<number_of_processors>.""")

        parser.add_argument("--target", action="store", dest="build_target",
                            help="specify a build target overriding the default target.")

        parser.add_argument("--clean-first", action="store_true", dest="clean_first", default=False,
                            help="build target clean first, then build the active target.")

        util.app_args_add_log_level(parser)

        # -j may be followed by a non-numeric argument which the parser is not able to handle.
        if '-j' in argv:
            i = argv.index('-j')
            # If the next argument is all numeric, attach it directly. In all other cases use the optimal number of parallel jobs.
            if i < (len(argv) - 1):
                if re.match(r'\d+$', argv[i + 1]):
                    argv_parsed = argv
                else:
                    # make a copy -> don't want to modify sys.argv.
                    argv_parsed = argv[:]
                    argv_parsed[i] = '-j' + str(self._cmake_launcher.get_optimal_number_cmake_jobs())
            else:
                # make a copy -> don't want to modify sys.argv.
                argv_parsed = argv[:]
                argv_parsed[i] = '-j' + str(self._cmake_launcher.get_optimal_number_cmake_jobs())
        else:
            argv_parsed = argv

        (cmake_py_options, args_left) = parser.parse_known_args(argv_parsed)

        # configure the python logger asap
        util.app_configure_logging(cmake_py_options.log_level)

        self._top_dir = os.getcwd()

        # if args_left:
        #    print("cmake args", args_left)

        # Assign cmake.py options to attributes of CmakeLauncherParams.
        launcher_params = cmksupp.CmakeLauncherParams()
        launcher_params.cmk_build = cmake_py_options.build
        launcher_params.cmk_build_jobs = cmake_py_options.build_jobs
        launcher_params.clean_first = cmake_py_options.clean_first
        launcher_params.cmk_build_target = cmake_py_options.build_target
        launcher_params.cmk_generator_alias = cmake_py_options.generator
        if cmake_py_options.cache_entries:
            launcher_params.cmk_cache_entries = ['-D' + x for x in cmake_py_options.cache_entries]
        if cmake_py_options.warning_flags:
            launcher_params.cmk_warning_flags = ['-W' + x for x in cmake_py_options.warning_flags]

        cmake_args = []
        re_cmake_py_arg = re.compile(r'(toolset|variant|link|address-model)=(\S+)')
        for arg in args_left:
            # print("processing argument " + arg)
            re_match = re_cmake_py_arg.match(arg)
            if re_match:
                arg_key = re_match.group(1)
                arg_value = re_match.group(2)
                if arg_key == 'toolset':
                    if arg_value.endswith('.cmake'):
                        # A toolset=<something>.cmake expression is supposed to be a toolchain file to enable
                        # some kind of cross compilation.
                        if not os.path.exists(arg_value):
                            print("cmake.py: error: toolchain file=" + arg_value + " does not exist.")
                            sys.exit(1)
                        launcher_params.toolset_str = os.path.abspath(arg_value)
                    else:
                        launcher_params.toolset_str = self._normalize_toolset_spec(arg_value)
                elif arg_key == 'variant':
                    build_configs = arg_value.split(',')
                    for cfg in build_configs:
                        if cfg not in ['release', 'debug', 'relwithdebinfo', 'minsizerel']:
                            print("cmake.py: error: " + arg + " is not understood.")
                            sys.exit(1)
                    launcher_params.build_configs = build_configs
                elif arg_key == 'link':
                    link_variants = arg_value.split(',')
                    for lnk in link_variants:
                        if lnk not in ['static', 'shared']:
                            print("cmake.py: error: " + arg + " is not understood.")
                            sys.exit(1)
                    launcher_params.link_variants = link_variants
                elif arg_key == 'address-model':
                    if arg_value == '32':
                        launcher_params.target_arch = 'x86'
                    elif arg_value == '64':
                        launcher_params.target_arch = 'x86_64'
                    else:
                        print("cmake.py: error: " + arg + " is not understood.")
                        sys.exit(1)
                else:
                    print("cmake.py: error: " + arg + " is not understood.")
                    sys.exit(1)
                continue
            if arg == '--':
                # Semantics seems to be tricky and I haven't seen a convincing use case yet.
                # In configuration mode all unknown arguments are passed verbatim to cmake and in build mode
                # all unknown arguments are really build tool arguments and are passed verbatim to the build tool.
                print("cmake.py: error: argument '--' encountered, this is not yet supported, please contact the maintainer.")
                sys.exit(1)
            # all other arguments are passed on to cmake verbatim.
            cmake_args.append(arg)
        return launcher_params, cmake_args

    def _normalize_toolset_spec(self, toolset_spec):
        toolset_spec_norm = toolset_spec
        if self._sys_info.is_linux():
            if toolset_spec.find('-g++') >= 0:
                # x86_64-w64-mingw32-g++-posix -> normalized to x86_64-w64-mingw32-gcc-posix
                toolset_spec_norm = toolset_spec.replace('-g++', '-gcc')
            if toolset_spec_norm.find('-gcc') >= 0:
                # looks like a cross compiler specification which requires a toolchain file matching the toolset spec and the linux system.
                toolset_spec_norm = self._find_toolchain_file(toolset_spec_norm)
        elif self._sys_info.is_windows():
            pass
        elif self._sys_info.is_macosx():
            pass
        else:
            # unsupported platform -> contact maintainer
            assert False
        return toolset_spec_norm

    def _find_toolchain_file(self, toolset_spec):
        toolchain_file = None
        toolchains_dir = os.path.join(self._get_workspace_folder(), 'BoostBuild', 'cmake', 'toolchains')
        if self._sys_info.is_linux():
            toolchain_file_suffix = '-' + self._sys_info.get_os_distro_short()
            if self._sys_info.get_os_distro_short() == 'ubuntu':
                os_version_str = ver.ubuntu_version_tuple_to_str(self._sys_info.get_os_version())
            else:
                os_version_str = self._sys_info.get_os_version()
            toolchain_file_suffix += os_version_str.replace('.', '') + '.cmake'
            if os.path.exists(os.path.join(toolchains_dir, toolset_spec + toolchain_file_suffix)):
                toolchain_file = os.path.join(toolchains_dir, toolset_spec + toolchain_file_suffix)
        if toolchain_file is None:
            msg = "toolset=" + toolset_spec + " cannot be mapped to a default toolchain file automatically. Please use a toolchain file or "
            msg += "contact technical support."
            raise Exception(msg)
        return toolchain_file

    def _get_workspace_folder(self):
        assert self._top_dir is not None
        return self._top_dir